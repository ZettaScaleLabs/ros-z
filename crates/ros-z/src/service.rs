#![allow(unused)]

use std::{
    collections::HashMap,
    marker::PhantomData,
    sync::{Arc, atomic::AtomicUsize},
    time::Duration,
};

use serde::Deserialize;
use tracing::{debug, error, info, trace, warn};
use zenoh::{
    Result, Session, Wait, bytes,
    key_expr::KeyExpr,
    liveliness::LivelinessToken,
    query::{Query, Reply},
    sample::Sample,
};

use std::sync::atomic::Ordering;

use crate::entity::TopicKE;
use crate::topic_name;

use crate::{
    Builder,
    attachment::{self, Attachment, GidArray},
    common::DataHandler,
    entity::EndpointEntity,
    impl_with_type_info,
    msg::{SerdeCdrSerdes, ZDeserializer, ZMessage, ZService},
    qos::QosHistory,
    queue::BoundedQueue,
};

#[derive(Debug)]
pub struct ZClientBuilder<T> {
    pub(crate) entity: EndpointEntity,
    pub(crate) session: Arc<Session>,
    pub(crate) keyexpr_format: ros_z_protocol::KeyExprFormat,
    pub(crate) _phantom_data: PhantomData<T>,
}

impl_with_type_info!(ZClientBuilder<T>);
impl_with_type_info!(ZServerBuilder<T>);

/// A ROS 2-style service client that sends typed requests and receives typed responses.
///
/// Create a client via [`ZNode::create_client`](crate::node::ZNode::create_client).
/// Send a request with [`send_request`](ZClient::send_request) (async), then retrieve
/// the response with [`take_response`](ZClient::take_response) (non-blocking),
/// [`take_response_timeout`](ZClient::take_response_timeout) (waits up to a deadline),
/// or [`async_take_response`](ZClient::async_take_response) (async wait).
///
/// # Example
///
/// ```rust,ignore
/// use ros_z::prelude::*;
/// use std::time::Duration;
///
/// // client: ZClient<MyService>
/// client.send_request(&request).await?;
/// let response = client.take_response_timeout(Duration::from_secs(5))?;
/// ```
pub struct ZClient<T: ZService> {
    // TODO: replace this with the sample sn
    sn: AtomicUsize,
    // TODO: replace this with zenoh's global entity id
    gid: GidArray,
    inner: zenoh::query::Querier<'static>,
    lv_token: LivelinessToken,
    tx: flume::Sender<Sample>,
    pub(crate) rx: flume::Receiver<Sample>,
    topic: String,
    _phantom_data: PhantomData<T>,
}

impl<T: ZService> std::fmt::Debug for ZClient<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ZClient")
            .field("topic", &self.topic)
            .finish_non_exhaustive()
    }
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

        let topic_ke = self.keyexpr_format.topic_key_expr(&self.entity)?;
        let key_expr = (*topic_ke).clone(); // Deref and clone the KeyExpr
        debug!("[CLN] Key expression: {}", key_expr);

        let inner = self
            .session
            .declare_querier(key_expr)
            .target(zenoh::query::QueryTarget::AllComplete)
            .consolidation(zenoh::query::ConsolidationMode::None)
            .timeout(Duration::from_secs(10))
            .wait()?;
        let lv_ke = self
            .keyexpr_format
            .liveliness_key_expr(&self.entity, &self.session.zid())?;
        let lv_token = self
            .session
            .liveliness()
            .declare_token((*lv_ke).clone())
            .wait()?;
        // Use bounded channel based on QoS depth
        let depth = match self.entity.qos.history {
            ros_z_protocol::qos::QosHistory::KeepLast(n) => n,
            ros_z_protocol::qos::QosHistory::KeepAll => 1000, // Default reasonable limit for KeepAll
        };
        let (tx, rx) = flume::bounded(depth);
        debug!("[CLN] Client ready: service={}", self.entity.topic);

        Ok(ZClient {
            sn: AtomicUsize::new(1), // Start at 1 for ROS compatibility
            inner,
            lv_token,
            gid: crate::entity::endpoint_gid(&self.entity),
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
    fn new_attachment(&self) -> Attachment {
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

    /// Retrieve the next response without blocking.
    ///
    /// Returns `Err` immediately if no response has arrived yet. Use
    /// [`take_response_timeout`](ZClient::take_response_timeout) to wait up to a
    /// deadline, or [`async_take_response`](ZClient::async_take_response) to await
    /// indefinitely in an async context.
    // For ROS-Z
    pub fn take_response(&self) -> Result<T::Response>
    where
        T::Response: ZMessage,
        for<'a> <T::Response as ZMessage>::Serdes:
            ZDeserializer<Output = T::Response, Input<'a> = &'a [u8]>,
    {
        let sample = self.take_sample()?;
        let msg = <T::Response as ZMessage>::deserialize(&sample.payload().to_bytes())
            .map_err(|e| zenoh::Error::from(e.to_string()))?;
        Ok(msg)
    }

    /// Wait for the next response, up to `timeout`. Returns `Err` if no response
    /// arrives within the deadline.
    pub fn take_response_timeout(&self, timeout: Duration) -> Result<T::Response>
    where
        T::Response: ZMessage,
        for<'a> <T::Response as ZMessage>::Serdes:
            ZDeserializer<Output = T::Response, Input<'a> = &'a [u8]>,
    {
        let sample = self.take_sample_timeout(timeout)?;
        let payload_bytes = sample.payload().to_bytes();
        let msg = <T::Response as ZMessage>::deserialize(&payload_bytes[..])
            .map_err(|e| zenoh::Error::from(e.to_string()))?;
        Ok(msg)
    }
    /// Asynchronously wait for the next response. Awaits indefinitely until a
    /// response arrives or the channel is disconnected.
    pub async fn async_take_response(&self) -> Result<T::Response>
    where
        T::Response: ZMessage,
        for<'a> <T::Response as ZMessage>::Serdes:
            ZDeserializer<Output = T::Response, Input<'a> = &'a [u8]>,
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
    /// Send a typed request to the service server.
    ///
    /// This is an `async fn` — it must be `.await`ed. The call resolves once the
    /// Zenoh query is dispatched; it does **not** wait for a response. Retrieve the
    /// response separately with [`take_response`](ZClient::take_response),
    /// [`take_response_timeout`](ZClient::take_response_timeout), or
    /// [`async_take_response`](ZClient::async_take_response).
    ///
    /// Succeeds even when no server is running (fire-and-forget dispatch).
    #[tracing::instrument(name = "send_request", skip(self, msg), fields(
        service = %self.topic,
        sn = self.sn.load(Ordering::Acquire),
        payload_len = tracing::field::Empty
    ))]
    pub async fn send_request(&self, msg: &T::Request) -> Result<()> {
        let payload = msg.serialize();
        tracing::Span::current().record("payload_len", payload.len());

        // Log the key expression being queried
        let query_ke = self.inner.key_expr();
        info!("[CLN] Sending request to key expression: {}", query_ke);
        debug!("[CLN] Sending request");

        let tx = self.tx.clone();
        self.inner
            .get()
            .payload(payload)
            .attachment(self.new_attachment())
            .callback(move |reply| {
                match reply.into_result() {
                    Ok(sample) => {
                        info!(
                            "[CLN] Reply received: len={}, kind={:?}",
                            sample.payload().len(),
                            sample.kind()
                        );
                        debug!("[CLN] Reply received: len={}", sample.payload().len());
                        // Use try_send for bounded channel - if full, drop the response (QoS depth enforcement)
                        if tx.try_send(sample).is_err() {
                            tracing::warn!(
                                "Client response queue full, dropping response (QoS depth enforced)"
                            );
                        }
                    }
                    Err(e) => {
                        warn!("[CLN] Reply error: {:?}", e);
                    }
                }
            })
            .await?;

        Ok(())
    }

    #[cfg(feature = "rmw")]
    pub fn rmw_send_request<F>(&self, msg: &T::Request, notify: F) -> Result<i64>
    where
        F: Fn() + Send + Sync + 'static,
    {
        let tx = self.tx.clone();
        let attachment = self.new_attachment();
        let sn = attachment.sequence_number;
        self.inner
            .get()
            .payload(msg.serialize())
            .attachment(attachment)
            .callback(move |reply| {
                match reply.into_result() {
                    Ok(sample) => {
                        // Use try_send for bounded channel - if full, drop the response (QoS depth enforcement)
                        if tx.try_send(sample).is_err() {
                            tracing::warn!(
                                "Client response queue full, dropping response (QoS depth enforced)"
                            );
                        }
                        notify();
                    }
                    Err(err) => {
                        // Handle timeout and other reply errors gracefully
                        // This can happen when a service is not available or times out
                        tracing::debug!("Client reply error: {:?}", err);
                    }
                }
            })
            .wait()?;
        Ok(sn)
    }
}

#[derive(Debug)]
pub struct ZServerBuilder<T> {
    pub(crate) entity: EndpointEntity,
    pub(crate) session: Arc<Session>,
    pub(crate) keyexpr_format: ros_z_protocol::KeyExprFormat,
    pub(crate) _phantom_data: PhantomData<T>,
}

impl<T> ZClientBuilder<T> {
    /// Set the QoS profile for this client.
    pub fn with_qos(mut self, qos: crate::qos::QosProfile) -> Self {
        self.entity.qos = qos.to_protocol_qos();
        self
    }

    /// Get a reference to the entity (for internal and rmw use).
    pub fn entity(&self) -> &EndpointEntity {
        &self.entity
    }
}

impl<T> ZServerBuilder<T> {
    /// Set the QoS profile for this server.
    pub fn with_qos(mut self, qos: crate::qos::QosProfile) -> Self {
        self.entity.qos = qos.to_protocol_qos();
        self
    }

    /// Get a reference to the entity (for internal and rmw use).
    pub fn entity(&self) -> &EndpointEntity {
        &self.entity
    }
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
    pub(crate) queue: Option<Arc<BoundedQueue<Q>>>,
    pub(crate) map: HashMap<QueryKey, Query>,
    _phantom_data: PhantomData<T>,
}

impl<T: ZService, Q> std::fmt::Debug for ZServer<T, Q> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ZServer")
            .field("key_expr", &self.key_expr.as_str())
            .finish_non_exhaustive()
    }
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
        self.queue
            .as_ref()
            .expect("Server was built with callback mode, no queue available")
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

        let topic_ke = self.keyexpr_format.topic_key_expr(&self.entity)?;
        let key_expr = (*topic_ke).clone(); // Deref and clone the KeyExpr
        tracing::debug!("[SRV] KE: {key_expr}");

        info!("[SRV] Declaring queryable on key expression: {}", key_expr);

        let inner = self
            .session
            .declare_queryable(&key_expr)
            .complete(true)
            .callback(move |query| {
                info!(
                    "[SRV] Query received: ke={}, selector={}, parameters={}",
                    query.key_expr(),
                    query.selector(),
                    query.parameters()
                );

                if let Some(att) = query.attachment() {
                    info!("[SRV] Query has attachment: {} bytes", att.len());
                } else {
                    info!("[SRV] Query has NO attachment");
                }

                if let Some(payload) = query.payload() {
                    info!("[SRV] Query has payload: {} bytes", payload.len());
                } else {
                    info!("[SRV] Query has NO payload");
                }

                handler.handle(query);
            })
            .wait()?;

        let lv_ke = self
            .keyexpr_format
            .liveliness_key_expr(&self.entity, &self.session.zid())?;
        let lv_token = self
            .session
            .liveliness()
            .declare_token((*lv_ke).clone())
            .wait()?;

        Ok(ZServer {
            key_expr,
            sn: AtomicUsize::new(1), // Start at 1 for ROS compatibility
            inner,
            lv_token,
            gid: crate::entity::endpoint_gid(&self.entity),
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

    #[cfg(feature = "rmw")]
    pub fn build_with_notifier<F>(self, notify: F) -> Result<ZServer<T>>
    where
        F: Fn() + Send + Sync + 'static,
    {
        let queue_size = match self.entity.qos.history {
            ros_z_protocol::qos::QosHistory::KeepLast(depth) => depth,
            ros_z_protocol::qos::QosHistory::KeepAll => usize::MAX,
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
            ros_z_protocol::qos::QosHistory::KeepLast(depth) => depth,
            ros_z_protocol::qos::QosHistory::KeepAll => usize::MAX,
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
    fn new_attachment(&self) -> Attachment {
        Attachment::new(self.sn.fetch_add(1, Ordering::AcqRel) as _, self.gid)
    }

    /// Retrieve the next query on the service without deserializing the payload.
    ///
    /// This method is useful when custom deserialization logic is needed.
    pub fn take_query(&self) -> Result<Query> {
        let queue = self.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Server was built with callback, no queue available")
        })?;
        queue
            .try_recv()
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
        for<'a> <T::Request as ZMessage>::Serdes:
            ZDeserializer<Output = T::Request, Input<'a> = &'a [u8]>,
    {
        trace!("[SRV] Waiting for request");

        let queue = self.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Server was built with callback, no queue available")
        })?;
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
    pub async fn async_take_request(&mut self) -> Result<(QueryKey, T::Request)>
    where
        T::Request: ZMessage + Send + Sync + 'static,
        for<'a> <T::Request as ZMessage>::Serdes:
            ZDeserializer<Output = T::Request, Input<'a> = &'a [u8]>,
    {
        let queue = self.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Server was built with callback, no queue available")
        })?;
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
    /// - `key` is the query key of the request to reply to and is obtained from [take_request](Self::take_request) or [async_take_request](Self::async_take_request)
    #[tracing::instrument(name = "send_response", skip(self, msg), fields(
        service = %self.key_expr,
        sn = %key.sn,
        payload_len = tracing::field::Empty
    ))]
    pub fn send_response(&mut self, msg: &T::Response, key: &QueryKey) -> Result<()> {
        debug!(
            "[SRV] Looking for query with key sn:{}, gid:{:?}",
            key.sn, key.gid
        );
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
    /// - `key` is the query key of the request to reply to and is obtained from [take_request](Self::take_request) or [async_take_request](Self::async_take_request)
    pub async fn async_send_response(&mut self, msg: &T::Response, key: &QueryKey) -> Result<()> {
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

#[cfg(test)]
mod tests {
    // -----------------------------------------------------------------------
    // Topic name qualification for service names
    // Service names follow the same rules as topic names
    // -----------------------------------------------------------------------

    #[test]
    fn test_qualify_service_absolute_unchanged() {
        let result = crate::topic_name::qualify_service_name("/add_two_ints", "/", "node").unwrap();
        assert_eq!(result, "/add_two_ints");
    }

    #[test]
    fn test_qualify_service_relative_adds_slash() {
        let result = crate::topic_name::qualify_service_name("add_two_ints", "/", "node").unwrap();
        assert_eq!(result, "/add_two_ints");
    }

    #[test]
    fn test_qualify_service_with_namespace() {
        let result =
            crate::topic_name::qualify_service_name("add_two_ints", "/ns", "node").unwrap();
        assert_eq!(result, "/ns/add_two_ints");
    }

    #[test]
    fn test_qualify_service_multipart_name() {
        let result =
            crate::topic_name::qualify_service_name("/my/service/name", "/", "node").unwrap();
        assert_eq!(result, "/my/service/name");
    }

    // -----------------------------------------------------------------------
    // QoS stored in builder entity reflects the protocol values
    // -----------------------------------------------------------------------

    #[test]
    fn test_protocol_qos_default_is_reliable() {
        let qos = crate::qos::QosProfile::default();
        let proto = qos.to_protocol_qos();
        assert_eq!(
            proto.reliability,
            ros_z_protocol::qos::QosReliability::Reliable
        );
    }

    #[test]
    fn test_protocol_qos_default_is_volatile() {
        let qos = crate::qos::QosProfile::default();
        let proto = qos.to_protocol_qos();
        assert_eq!(
            proto.durability,
            ros_z_protocol::qos::QosDurability::Volatile
        );
    }
}
