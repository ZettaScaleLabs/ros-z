#![allow(unused)]

use std::{
    collections::HashMap,
    marker::PhantomData,
    sync::{atomic::AtomicUsize, Arc},
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
use tracing::{error, info, debug, warn, trace};

use std::sync::atomic::Ordering;

use crate::entity::TopicKE;
use crate::topic_name;

use crate::{
    Builder,
    attachment::{self, Attachment, GidArray},
    common::DataHandler,
    entity::EndpointEntity,
    impl_with_type_info,
    msg::{CdrSerdes, ZDeserializer, ZMessage, ZService},
    qos::QosHistory,
    queue::BoundedQueue,
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
    topic: String,
    _phantom_data: PhantomData<T>,
}

impl<T> Builder for ZClientBuilder<T>
where
    T: ZService,
{
    type Output = ZClient<T>;

    #[tracing::instrument(name = "client_build", skip(self), fields(
        service = %self.entity.topic
    ))]
    fn build(mut self) -> Result<Self::Output> {
        // Qualify the service name according to ROS 2 rules
        let qualified_service = topic_name::qualify_service_name(
            &self.entity.topic,
            &self.entity.node.namespace,
            &self.entity.node.name,
        )
        .map_err(|e| zenoh::Error::from(format!("Failed to qualify service: {}", e)))?;

        self.entity.topic = qualified_service.clone();
        debug!("[CLN] Qualified service: {}", qualified_service);

        let key_expr = self.entity.topic_key_expr()?;
        debug!("[CLN] Key expression: {}", key_expr);

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
        info!("[CLN] Client ready: service={}", self.entity.topic);

        Ok(ZClient {
            sn: AtomicUsize::new(1), // Start at 1 for ROS compatibility
            inner,
            lv_token,
            gid: self.entity.gid(),
            tx,
            rx,
            topic: self.entity.topic.clone(),
            _phantom_data: Default::default(),
        })
    }
}

impl<T> ZClient<T>
where
    T: ZService,
{
    fn new_attchment(&self) -> Attachment {
        Attachment::new(self.sn.fetch_add(1, Ordering::AcqRel) as _, self.gid)
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
        T::Response: ZMessage,
        for<'a> <T::Response as ZMessage>::Serdes: ZDeserializer<Output = T::Response, Input<'a> = &'a [u8]>,
    {
        let sample = self.take_sample_timeout(timeout)?;
        let payload_bytes = sample.payload().to_bytes();
        let msg = <T::Response as ZMessage>::deserialize(&payload_bytes[..])
            .map_err(|e| zenoh::Error::from(e.to_string()))?;
        Ok(msg)
    }

    pub async fn take_response_async(&self) -> Result<T::Response>
    where
        T::Response: ZMessage,
        for<'a> <T::Response as ZMessage>::Serdes: ZDeserializer<Output = T::Response, Input<'a> = &'a [u8]>,
    {
        let sample = self.rx.recv_async().await?;
        let payload_bytes = sample.payload().to_bytes();
        let msg = <T::Response as ZMessage>::deserialize(&payload_bytes[..])
            .map_err(|e| zenoh::Error::from(e.to_string()))?;
        Ok(msg)
    }
}

impl<T> ZClient<T>
where
    T: ZService,
{
    #[tracing::instrument(name = "send_request", skip(self, msg), fields(
        service = %self.topic,
        sn = self.sn.load(Ordering::Acquire),
        payload_len = tracing::field::Empty
    ))]
    pub async fn send_request(&self, msg: &T::Request) -> Result<()> {
        let payload = msg.serialize();
        tracing::Span::current().record("payload_len", payload.len());

        debug!("[CLN] Sending request");

        let tx = self.tx.clone();
        self.inner
            .get()
            .payload(payload)
            .attachment(self.new_attchment())
            .callback(move |reply| {
                match reply.into_result() {
                    Ok(sample) => {
                        debug!("[CLN] Reply received: len={}", sample.payload().len());
                        let _ = tx.send(sample);
                    }
                    Err(e) => {
                        warn!("[CLN] Reply error: {:?}", e);
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

pub struct ZServer<T: ZService, Q = Query> {
    // NOTE: This is biased toward RMW
    key_expr: KeyExpr<'static>,
    // TODO: replace this with the sample sn
    sn: AtomicUsize,
    // TODO: replace this with zenoh's global entity id
    gid: GidArray,
    inner: zenoh::query::Queryable<()>,
    lv_token: LivelinessToken,
    pub queue: Option<Arc<BoundedQueue<Q>>>,
    pub map: HashMap<QueryKey, Query>,
    _phantom_data: PhantomData<T>,
}

impl<T, Q> ZServer<T, Q>
where
    T: ZService,
{
    /// Access the receiver queue.
    ///
    /// # Panics
    ///
    /// Panics if the server was built with `build_with_callback()` and has no queue.
    /// Action servers always have queues and will never panic.
    pub fn queue(&self) -> &Arc<BoundedQueue<Q>> {
        self.queue.as_ref().expect("Server was built with callback mode, no queue available")
    }
}

impl<T> ZServerBuilder<T>
where
    T: ZService,
{
    /// Internal method that all build variants use.
    fn build_internal<Q>(
        mut self,
        handler: DataHandler<Query>,
        queue: Option<Arc<BoundedQueue<Q>>>,
    ) -> Result<ZServer<T, Q>> {
        let qualified_service = topic_name::qualify_service_name(
            &self.entity.topic,
            &self.entity.node.namespace,
            &self.entity.node.name,
        )
        .map_err(|e| zenoh::Error::from(format!("Failed to qualify service: {}", e)))?;

        self.entity.topic = qualified_service;

        let key_expr = self.entity.topic_key_expr()?;
        tracing::debug!("[SRV] KE: {key_expr}");

        let inner = self
            .session
            .declare_queryable(&key_expr)
            .complete(true)
            .callback(move |query| {
                debug!("[SRV] Query received: ke={}, selector={}",
                    query.key_expr(), query.selector());

                if let Some(att) = query.attachment() {
                    trace!("[SRV] Query has attachment");
                }

                handler.handle(query);
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
            queue,
            map: HashMap::new(),
            _phantom_data: Default::default(),
        })
    }

    pub fn build_with_callback<F>(self, callback: F) -> Result<ZServer<T, ()>>
    where
        F: Fn(Query) + Send + Sync + 'static,
    {
        self.build_internal(DataHandler::Callback(Arc::new(callback)), None)
    }

    #[cfg(feature = "rcl-z")]
    pub fn build_with_notifier<F>(self, notify: F) -> Result<ZServer<T>>
    where
        F: Fn() + Send + Sync + 'static,
    {
        let queue_size = match self.entity.qos.history {
            QosHistory::KeepLast(depth) => depth,
            QosHistory::KeepAll => usize::MAX,
        };
        let queue = Arc::new(BoundedQueue::new(queue_size));
        self.build_internal(
            DataHandler::QueueWithNotifier {
                queue: queue.clone(),
                notifier: Arc::new(notify),
            },
            Some(queue),
        )
    }
}

impl<T> Builder for ZServerBuilder<T>
where
    T: ZService,
{
    type Output = ZServer<T>;

    fn build(self) -> Result<Self::Output> {
        let queue_size = match self.entity.qos.history {
            QosHistory::KeepLast(depth) => depth,
            QosHistory::KeepAll => usize::MAX,
        };
        let queue = Arc::new(BoundedQueue::new(queue_size));
        self.build_internal(DataHandler::Queue(queue.clone()), Some(queue))
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

impl<T> ZServer<T, Query>
where
    T: ZService,
{
    fn new_attchment(&self) -> Attachment {
        Attachment::new(self.sn.fetch_add(1, Ordering::AcqRel) as _, self.gid)
    }

    /// Retrieve the next query on the service without deserializing the payload.
    ///
    /// This method is useful when custom deserialization logic is needed.
    pub fn take_query(&self) -> Result<Query> {
        let queue = self.queue.as_ref()
            .ok_or_else(|| zenoh::Error::from("Server was built with callback, no queue available"))?;
        queue.try_recv()
            .ok_or_else(|| zenoh::Error::from("No query available"))
    }

    /// Blocks waiting to receive the next request on the service and then deserializes the payload.
    ///
    /// This method may fail if the message does not deserialize as the requested type.
    #[tracing::instrument(name = "take_request", skip(self), fields(
        service = %self.key_expr,
        sn = tracing::field::Empty,
        payload_len = tracing::field::Empty
    ))]
    pub fn take_request(&mut self) -> Result<(QueryKey, T::Request)>
    where
        T::Request: ZMessage + Send + Sync + 'static,
        for<'a> <T::Request as ZMessage>::Serdes: ZDeserializer<Output = T::Request, Input<'a> = &'a [u8]>,
    {
        trace!("[SRV] Waiting for request");

        let queue = self.queue.as_ref()
            .ok_or_else(|| zenoh::Error::from("Server was built with callback, no queue available"))?;
        let query = queue.recv();
        let attachment: Attachment = query.attachment().unwrap().try_into()?;
        let key: QueryKey = attachment.into();

        tracing::Span::current().record("sn", key.sn);

        let payload_bytes = query.payload().unwrap().to_bytes();
        tracing::Span::current().record("payload_len", payload_bytes.len());

        if self.map.contains_key(&key) {
            warn!("[SRV] Duplicate request: sn={}", key.sn);
            return Err("Existing query detected".into());
        }

        debug!("[SRV] Processing request");

        let msg = <T::Request as ZMessage>::deserialize(&payload_bytes[..])
            .map_err(|e| zenoh::Error::from(e.to_string()))?;
        self.map.insert(key.clone(), query);

        Ok((key, msg))
    }

    /// Awaits the next request on the service and then deserializes the payload.
    ///
    /// This method may fail if the message does not deserialize as the requested type.
    pub async fn take_request_async(&mut self) -> Result<(QueryKey, T::Request)>
    where
        T::Request: ZMessage + Send + Sync + 'static,
        for<'a> <T::Request as ZMessage>::Serdes: ZDeserializer<Output = T::Request, Input<'a> = &'a [u8]>,
    {
        let queue = self.queue.as_ref()
            .ok_or_else(|| zenoh::Error::from("Server was built with callback, no queue available"))?;
        let query = queue.recv_async().await;
        let attachment: Attachment = query.attachment().unwrap().try_into()?;
        let key: QueryKey = attachment.into();
        if self.map.contains_key(&key) {
            return Err("Existing query detected".into());
        }
        let payload_bytes = query.payload().unwrap().to_bytes();
        let msg = <T::Request as ZMessage>::deserialize(&payload_bytes[..])
            .map_err(|e| zenoh::Error::from(e.to_string()))?;
        self.map.insert(key.clone(), query);

        Ok((key, msg))
    }

    /// Blocks sending the response to a service request.
    ///
    /// - `msg` is the response message to send.
    /// - `key` is the query key of the request to reply to and is obtained from [take_request](Self::take_request) or [take_request_async](Self::take_request_async)
    #[tracing::instrument(name = "send_response", skip(self, msg), fields(
        service = %self.key_expr,
        sn = %key.sn,
        payload_len = tracing::field::Empty
    ))]
    pub fn send_response(&mut self, msg: &T::Response, key: &QueryKey) -> Result<()> {
        match self.map.remove(key) {
            Some(query) => {
                let payload = msg.serialize();
                tracing::Span::current().record("payload_len", payload.len());

                debug!("[SRV] Sending response");

                // Use the sequence number and GID from the request
                let attachment = Attachment::new(key.sn, key.gid);
                query
                    .reply(&self.key_expr, payload)
                    .attachment(attachment)
                    .wait()
            }
            None => {
                error!("[SRV] No query found for sn={}", key.sn);
                Err("Query map doesn't contain key".into())
            }
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
