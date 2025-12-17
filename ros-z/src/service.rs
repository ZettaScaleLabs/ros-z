#![allow(unused)]

use std::{
    collections::HashMap,
    marker::PhantomData,
    sync::{Arc, atomic::AtomicUsize},
    time::Duration,
};

use serde::Deserialize;
use zenoh::{
    Result, Session, Wait, bytes,
    key_expr::KeyExpr,
    liveliness::LivelinessToken,
    query::{Query, Reply},
    sample::Sample,
};

use std::sync::atomic::Ordering::AcqRel;

use crate::entity::TopicKE;
use crate::topic_name;

use crate::{
    Builder,
    attachment::{self, Attachment, GidArray},
    entity::EndpointEntity,
    impl_with_type_info,
    msg::{CdrSerdes, ZMessage, ZService},
};

#[derive(Debug)]
pub struct ZClientBuilder<T> {
    pub entity: EndpointEntity,
    pub session: Arc<Session>,
    pub _phantom_data: PhantomData<T>,
}

impl_with_type_info!(ZClientBuilder<T>);
impl_with_type_info!(ZServerBuilder<T>);

pub struct ZClient<T: ZService> {
    // TODO: replace this with the sample sn
    sn: AtomicUsize,
    // TODO: replace this with zenoh's global entity id
    gid: GidArray,
    inner: zenoh::query::Querier<'static>,
    lv_token: LivelinessToken,
    tx: flume::Sender<Sample>,
    pub rx: flume::Receiver<Sample>,
    _phantom_data: PhantomData<T>,
}

impl<T> Builder for ZClientBuilder<T>
where
    T: ZService,
{
    type Output = ZClient<T>;

    fn build(mut self) -> Result<Self::Output> {
        // Qualify the service name according to ROS 2 rules
        let qualified_service = topic_name::qualify_service_name(
            &self.entity.topic,
            &self.entity.node.namespace,
            &self.entity.node.name,
        )
        .map_err(|e| zenoh::Error::from(format!("Failed to qualify service: {}", e)))?;

        self.entity.topic = qualified_service;

        let key_expr = self.entity.topic_key_expr()?;
        tracing::debug!("[CLN] KE: {key_expr}");

        let inner = self.session.declare_querier(key_expr)
            .target(zenoh::query::QueryTarget::AllComplete)
            .consolidation(zenoh::query::ConsolidationMode::None)
            .timeout(Duration::from_secs(10))
            .wait()?;
        let lv_token = self
            .session
            .liveliness()
            .declare_token(self.entity.lv_token_key_expr()?)
            .wait()?;
        let (tx, rx) = flume::unbounded();
        Ok(ZClient {
            sn: AtomicUsize::new(1), // Start at 1 for ROS compatibility
            inner,
            lv_token,
            gid: self.entity.gid(),
            tx,
            rx,
            _phantom_data: Default::default(),
        })
    }
}

impl<T> ZClient<T>
where
    T: ZService,
{
    fn new_attchment(&self) -> Attachment {
        Attachment::new(self.sn.fetch_add(1, AcqRel) as _, self.gid)
    }

    pub fn take_sample(&self) -> Result<Sample> {
        match self.rx.try_recv() {
            Ok(sample) => Ok(sample),
            Err(flume::TryRecvError::Empty) => Err("No sample available".into()),
            Err(flume::TryRecvError::Disconnected) => Err("Channel disconnected".into()),
        }
    }

    pub fn take_sample_timeout(&self, timeout: Duration) -> Result<Sample> {
        Ok(self.rx.recv_timeout(timeout)?)
    }

    // For ROS-Z
    pub fn take_response(&self) -> Result<T::Response>
    where
        for<'c> T::Response: ZMessage<Serdes = CdrSerdes<T::Response>> + Deserialize<'c>,
    {
        let sample = self.take_sample()?;
        let msg = <T::Response as ZMessage>::deserialize(&sample.payload().to_bytes())
            .map_err(|e| zenoh::Error::from(e.to_string()))?;
        Ok(msg)
    }

    pub fn take_response_timeout(&self, timeout: Duration) -> Result<T::Response>
    where
        for<'c> T::Response: ZMessage<Serdes = CdrSerdes<T::Response>> + Deserialize<'c>,
    {
        let sample = self.take_sample_timeout(timeout)?;
        let msg = <T::Response as ZMessage>::deserialize(&sample.payload().to_bytes())
            .map_err(|e| zenoh::Error::from(e.to_string()))?;
        Ok(msg)
    }

    pub async fn take_response_async(&self) -> Result<T::Response>
    where
        for<'c> T::Response: ZMessage<Serdes = CdrSerdes<T::Response>> + Deserialize<'c>,
    {
        let sample = self.rx.recv_async().await?;
        let msg = <T::Response as ZMessage>::deserialize(&sample.payload().to_bytes())
            .map_err(|e| zenoh::Error::from(e.to_string()))?;
        Ok(msg)
    }
}

impl<T> ZClient<T>
where
    T: ZService,
{
    pub async fn send_request(&self, msg: &T::Request) -> Result<()> {
        let tx = self.tx.clone();
        self.inner
            .get()
            .payload(msg.serialize())
            .attachment(self.new_attchment())
            .callback(move |reply| {
                tracing::trace!("ZClient received reply in callback");
                match reply.into_result() {
                    Ok(sample) => {
                        tracing::trace!("ZClient reply OK, sending to channel");
                        tx.send(sample);
                    }
                    Err(e) => {
                        tracing::error!("ZClient reply error: {:?}", e);
                    }
                }
            })
            .await?;
        Ok(())
    }

    #[cfg(feature = "rcl-z")]
    pub fn rcl_send_request<F>(&self, msg: &T::Request, notify: F) -> Result<i64>
    where
        F: Fn() + Send + Sync + 'static,
    {
        let tx = self.tx.clone();
        let attachment = self.new_attchment();
        let sn = attachment.sequence_number;
        self.inner
            .get()
            .payload(msg.serialize())
            .attachment(attachment)
            .callback(move |reply| {
                match reply.into_result() {
                    Ok(sample) => {
                        tx.send(sample);
                        notify()
                    }
                    Err(err) => {
                        // Handle timeout and other reply errors gracefully
                        // This can happen when a service is not available or times out
                        tracing::debug!("Reply error in rcl_send_request: {:?}", err);
                    }
                }
            })
            .wait()?;
        Ok(sn)
    }
}

#[derive(Debug)]
pub struct ZServerBuilder<T> {
    pub entity: EndpointEntity,
    pub session: Arc<Session>,
    pub _phantom_data: PhantomData<T>,
}

pub struct ZServer<T: ZService> {
    // NOTE: This is biased toward RMW
    key_expr: KeyExpr<'static>,
    // TODO: replace this with the sample sn
    sn: AtomicUsize,
    // TODO: replace this with zenoh's global entity id
    gid: GidArray,
    inner: zenoh::query::Queryable<()>,
    lv_token: LivelinessToken,
    // rx: flume::Receiver<T::Request>,
    pub rx: flume::Receiver<Query>,
    pub map: HashMap<QueryKey, Query>,
    _phantom_data: PhantomData<T>,
}

impl<T> Builder for ZServerBuilder<T>
where
    T: ZService,
{
    type Output = ZServer<T>;

    fn build(mut self) -> Result<Self::Output> {
        // Qualify the service name according to ROS 2 rules
        let qualified_service = topic_name::qualify_service_name(
            &self.entity.topic,
            &self.entity.node.namespace,
            &self.entity.node.name,
        )
        .map_err(|e| zenoh::Error::from(format!("Failed to qualify service: {}", e)))?;

        self.entity.topic = qualified_service;

        let key_expr = self.entity.topic_key_expr()?;
        tracing::debug!("[SRV] KE: {key_expr}");

        let (tx, rx) = flume::unbounded();
        let inner = self
            .session
            .declare_queryable(&key_expr)
            .complete(true)
            .callback(move |query| {
                tracing::error!("RECEIVED QUERY: {}", &query.key_expr());
                tracing::error!("Selector: {}", &query.selector());
                assert!(tx.send(query).is_ok());
            })
            .wait()?;
        let lv_token = self
            .session
            .liveliness()
            .declare_token(self.entity.lv_token_key_expr()?)
            .wait()?;
        Ok(ZServer {
            key_expr,
            sn: AtomicUsize::new(1), // Start at 1 for ROS compatibility
            inner,
            lv_token,
            gid: self.entity.gid(),
            rx,
            map: HashMap::new(),
            _phantom_data: Default::default(),
        })
    }
}

impl<T> ZServerBuilder<T>
where
    T: ZService,
{
    #[cfg(feature = "rcl-z")]
    pub fn build_with_notifier<F>(mut self, notify: F) -> Result<ZServer<T>>
    where
        F: Fn() + Send + Sync + 'static,
    {
        // Qualify the service name according to ROS 2 rules
        let qualified_service = topic_name::qualify_service_name(
            &self.entity.topic,
            &self.entity.node.namespace,
            &self.entity.node.name,
        )
        .map_err(|e| zenoh::Error::from(format!("Failed to qualify service: {}", e)))?;

        self.entity.topic = qualified_service;

        let key_expr = self.entity.topic_key_expr()?;
        tracing::debug!("[SRV] KE: {key_expr}");

        let (tx, rx) = flume::unbounded();
        let inner = self
            .session
            .declare_queryable(&key_expr)
            .complete(true)
            .callback(move |query| {
                let _ = tx.send(query);
                notify();
            })
            .wait()?;
        let lv_token = self
            .session
            .liveliness()
            .declare_token(self.entity.lv_token_key_expr()?)
            .wait()?;
        Ok(ZServer {
            key_expr,
            sn: AtomicUsize::new(1), // Start at 1 for ROS compatibility
            inner,
            lv_token,
            gid: self.entity.gid(),
            rx,
            map: HashMap::new(),
            _phantom_data: Default::default(),
        })
    }
}

#[derive(Debug, PartialEq, Eq, Hash, Clone)]
pub struct QueryKey {
    pub sn: i64,
    pub gid: GidArray,
}

impl From<Attachment> for QueryKey {
    fn from(value: Attachment) -> Self {
        Self {
            sn: value.sequence_number,
            gid: value.source_gid,
        }
    }
}

impl<T> ZServer<T>
where
    T: ZService,
{
    fn new_attchment(&self) -> Attachment {
        Attachment::new(self.sn.fetch_add(1, AcqRel) as _, self.gid)
    }

    /// Retrieve the next query on the service without deserializing the payload.
    ///
    /// This method is useful when custom deserialization logic is needed.
    pub fn take_query(&self) -> Result<Query> {
        match self.rx.try_recv() {
            Ok(query) => Ok(query),
            Err(flume::TryRecvError::Empty) => Err("No query available".into()),
            Err(flume::TryRecvError::Disconnected) => Err("Channel disconnected".into()),
        }
    }

    /// Blocks waiting to receive the next request on the service and then deserializes the payload.
    ///
    /// This method may fail if the message does not deserialize as the requested type.
    pub fn take_request(&mut self) -> Result<(QueryKey, T::Request)>
    where
        for<'c> T::Request:
            ZMessage<Serdes = CdrSerdes<T::Request>> + Send + Sync + 'static + Deserialize<'c>,
    {
        let query = self.rx.recv()?;
        let attachment: Attachment = query.attachment().unwrap().try_into()?;
        let key: QueryKey = attachment.into();
        if self.map.contains_key(&key) {
            return Err("Existing query detected".into());
        }
        let msg = <T::Request as ZMessage>::deserialize(&query.payload().unwrap().to_bytes())
            .map_err(|e| zenoh::Error::from(e.to_string()))?;
        self.map.insert(key.clone(), query);

        Ok((key, msg))
    }

    /// Awaits the next request on the service and then deserializes the payload.
    ///
    /// This method may fail if the message does not deserialize as the requested type.
    pub async fn take_request_async(&mut self) -> Result<(QueryKey, T::Request)>
    where
        for<'c> T::Request:
            ZMessage<Serdes = CdrSerdes<T::Request>> + Send + Sync + 'static + Deserialize<'c>,
    {
        let query = self.rx.recv_async().await?;
        let attachment: Attachment = query.attachment().unwrap().try_into()?;
        let key: QueryKey = attachment.into();
        if self.map.contains_key(&key) {
            return Err("Existing query detected".into());
        }
        let msg = <T::Request as ZMessage>::deserialize(&query.payload().unwrap().to_bytes())
            .map_err(|e| zenoh::Error::from(e.to_string()))?;
        self.map.insert(key.clone(), query);

        Ok((key, msg))
    }

    /// Blocks sending the response to a service request.
    ///
    /// - `msg` is the response message to send.
    /// - `key` is the query key of the request to reply to and is obtained from [take_request](Self::take_request) or [take_request_async](Self::take_request_async)
    pub fn send_response(&mut self, msg: &T::Response, key: &QueryKey) -> Result<()> {
        match self.map.remove(key) {
            Some(query) => {
                // Use the sequence number and GID from the request
                let attachment = Attachment::new(key.sn, key.gid);
                query
                    .reply(&self.key_expr, msg.serialize())
                    .attachment(attachment)
                    .wait()
            }
            None => Err("Quey map doesn't contains {key}".into()),
        }
    }

    /// Awaits sending the response to a service request.
    ///
    /// - `msg` is the response message to send.
    /// - `key` is the query key of the request to reply to and is obtained from [take_request](Self::take_request) or [take_request_async](Self::take_request_async)
    pub async fn send_response_async(&mut self, msg: &T::Response, key: &QueryKey) -> Result<()> {
        match self.map.remove(key) {
            Some(query) => {
                // Use the sequence number and GID from the request
                let attachment = Attachment::new(key.sn, key.gid);
                query
                    .reply(&self.key_expr, msg.serialize())
                    .attachment(attachment)
                    .await
            }
            None => Err("Quey map doesn't contains {key}".into()),
        }
    }
}
