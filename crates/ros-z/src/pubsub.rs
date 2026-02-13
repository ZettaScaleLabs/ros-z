use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;
use std::{marker::PhantomData, sync::Arc};

use tracing::{debug, trace, warn};
use zenoh::liveliness::LivelinessToken;
use zenoh::{Result, Session, Wait, sample::Sample};

use crate::Builder;
use crate::attachment::{Attachment, GidArray};
use crate::common::DataHandler;
use crate::entity::EndpointEntity;
use crate::event::EventsManager;
use crate::impl_with_type_info;
use crate::queue::BoundedQueue;
use crate::topic_name;

use crate::msg::{CdrSerdes, ZDeserializer, ZMessage, ZSerializer};
use crate::qos::QosProfile;
use ros_z_protocol::qos::{QosDurability, QosHistory, QosReliability};
use std::sync::Mutex;

pub struct ZPub<T: ZMessage, S: ZSerializer> {
    pub entity: EndpointEntity,
    // TODO: replace this with the sample sn
    sn: AtomicUsize,
    // TODO: replace this with zenoh's global entity id
    gid: GidArray,
    inner: zenoh::pubsub::Publisher<'static>,
    _lv_token: LivelinessToken,
    with_attachment: bool,
    events_mgr: Arc<Mutex<EventsManager>>,
    shm_config: Option<Arc<crate::shm::ShmConfig>>,
    /// Schema for dynamic message publishing.
    pub dyn_schema: Option<Arc<crate::dynamic::schema::MessageSchema>>,
    _phantom_data: PhantomData<(T, S)>,
}

#[derive(Debug)]
pub struct ZPubBuilder<T, S = CdrSerdes<T>> {
    pub entity: EndpointEntity,
    pub session: Arc<Session>,
    pub with_attachment: bool,
    pub(crate) shm_config: Option<Arc<crate::shm::ShmConfig>>,
    pub(crate) keyexpr_format: ros_z_protocol::KeyExprFormat,
    /// Schema for dynamic message publishing.
    /// When set, the schema will be registered with the type description service.
    pub dyn_schema: Option<Arc<crate::dynamic::schema::MessageSchema>>,
    pub _phantom_data: PhantomData<(T, S)>,
}

impl_with_type_info!(ZPubBuilder<T, S>);
impl_with_type_info!(ZSubBuilder<T, S>);

impl<T, S> ZPubBuilder<T, S> {
    pub fn with_qos(mut self, qos: QosProfile) -> Self {
        self.entity.qos = qos.to_protocol_qos();
        self
    }

    pub fn with_attachment(mut self, with_attachment: bool) -> Self {
        self.with_attachment = with_attachment;
        self
    }

    /// Override SHM configuration for this publisher only.
    ///
    /// This overrides any SHM configuration inherited from the node or context.
    ///
    /// # Example
    /// ```no_run
    /// use ros_z::shm::{ShmConfig, ShmProviderBuilder};
    /// use ros_z::Builder;
    /// use std::sync::Arc;
    ///
    /// # fn main() -> zenoh::Result<()> {
    /// # let ctx = ros_z::context::ZContextBuilder::default().build()?;
    /// # let node = ctx.create_node("test").build()?;
    /// let provider = Arc::new(ShmProviderBuilder::new(20 * 1024 * 1024).build()?);
    /// let config = ShmConfig::new(provider).with_threshold(5_000);
    ///
    /// let pub = node.create_pub::<ros_z_msgs::std_msgs::String>("topic")
    ///     .with_shm_config(config)
    ///     .build()?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn with_shm_config(mut self, config: crate::shm::ShmConfig) -> Self {
        self.shm_config = Some(Arc::new(config));
        self
    }

    /// Disable SHM for this publisher.
    ///
    /// Even if SHM is enabled at the node or context level, this publisher
    /// will not use shared memory.
    ///
    /// # Example
    /// ```no_run
    /// use ros_z::Builder;
    ///
    /// # fn main() -> zenoh::Result<()> {
    /// # let ctx = ros_z::context::ZContextBuilder::default().with_shm_enabled()?.build()?;
    /// # let node = ctx.create_node("test").build()?;
    /// // Context has SHM enabled, but disable for this publisher
    /// let pub = node.create_pub::<ros_z_msgs::std_msgs::String>("small_messages")
    ///     .without_shm()
    ///     .build()?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn without_shm(mut self) -> Self {
        self.shm_config = None;
        self
    }

    pub fn with_serdes<S2>(self) -> ZPubBuilder<T, S2> {
        ZPubBuilder {
            entity: self.entity,
            session: self.session,
            with_attachment: self.with_attachment,
            shm_config: self.shm_config,
            keyexpr_format: self.keyexpr_format,
            dyn_schema: self.dyn_schema,
            _phantom_data: PhantomData,
        }
    }

    /// Set the dynamic message schema for runtime-typed publishers.
    ///
    /// When a schema is set and the node has a type description service enabled,
    /// the schema will be automatically registered with the service during build.
    /// This allows other nodes to query for this type's description.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let publisher = node
    ///     .create_pub_impl::<DynamicMessage>("topic", None)
    ///     .with_serdes::<DynamicCdrSerdes>()
    ///     .with_dyn_schema(schema)
    ///     .build()?;
    /// ```
    pub fn with_dyn_schema(mut self, schema: Arc<crate::dynamic::schema::MessageSchema>) -> Self {
        use crate::dynamic::MessageSchemaTypeDescription;

        // Only compute and set type_info if it hasn't been set already
        // (e.g., from create_dyn_sub_auto which provides the publisher's hash)
        if self.entity.type_info.is_none() {
            // Compute TypeInfo from schema for proper key expression matching with ROS 2
            // Convert ROS 2 canonical name to DDS name
            // "std_msgs/msg/String" → "std_msgs::msg::dds_::String_"
            let dds_name = schema
                .type_name
                .replace("/msg/", "::msg::dds_::")
                .replace("/srv/", "::srv::dds_::")
                .replace("/action/", "::action::dds_::")
                + "_";

            // Convert schema TypeHash to entity TypeHash via RIHS string
            let type_hash = match schema.compute_type_hash() {
                Ok(hash) => {
                    let rihs_string = hash.to_rihs_string();
                    crate::entity::TypeHash::from_rihs_string(&rihs_string)
                        .unwrap_or_else(crate::entity::TypeHash::zero)
                }
                Err(_) => crate::entity::TypeHash::zero(),
            };

            self.entity.type_info = Some(crate::entity::TypeInfo {
                name: dds_name,
                hash: type_hash,
            });
        }

        self.dyn_schema = Some(schema);
        self
    }
}

impl<T, S> Builder for ZPubBuilder<T, S>
where
    T: ZMessage + 'static,
    S: for<'a> ZSerializer<Input<'a> = &'a T> + 'static,
{
    type Output = ZPub<T, S>;

    #[tracing::instrument(name = "pub_build", skip(self), fields(
        topic = %self.entity.topic,
        qos_reliability = ?self.entity.qos.reliability,
        qos_durability = ?self.entity.qos.durability
    ))]
    fn build(mut self) -> Result<Self::Output> {
        // Qualify the topic name according to ROS 2 rules
        let qualified_topic = topic_name::qualify_topic_name(
            &self.entity.topic,
            &self.entity.node.namespace,
            &self.entity.node.name,
        )
        .map_err(|e| zenoh::Error::from(format!("Failed to qualify topic: {}", e)))?;

        self.entity.topic = qualified_topic.clone();
        debug!("[PUB] Qualified topic: {}", qualified_topic);

        let topic_ke = self.keyexpr_format.topic_key_expr(&self.entity)?;
        let key_expr = (*topic_ke).clone(); // Deref and clone the KeyExpr
        debug!("[PUB] Key expression: {}", key_expr);

        // Map QoS to Zenoh publisher settings
        let mut pub_builder = self.session.declare_publisher(key_expr);

        // Map reliability: Reliable uses Block, BestEffort uses Drop
        match self.entity.qos.reliability {
            QosReliability::Reliable => {
                pub_builder = pub_builder.congestion_control(zenoh::qos::CongestionControl::Block);
                debug!("[PUB] QoS: Reliable (Block)");
            }
            QosReliability::BestEffort => {
                pub_builder = pub_builder.congestion_control(zenoh::qos::CongestionControl::Drop);
                debug!("[PUB] QoS: BestEffort (Drop)");
            }
        }

        // Map durability: TransientLocal uses express=true for caching
        match self.entity.qos.durability {
            QosDurability::TransientLocal => {
                pub_builder = pub_builder.express(true);
                debug!("[PUB] Durability: TransientLocal (express)");
            }
            QosDurability::Volatile => {
                pub_builder = pub_builder.express(false);
                debug!("[PUB] Durability: Volatile");
            }
        }

        let inner = pub_builder.wait()?;
        debug!("[PUB] Publisher ready: topic={}", self.entity.topic);

        let lv_ke = self
            .keyexpr_format
            .liveliness_key_expr(&self.entity, &self.session.zid())?;
        let lv_token = self
            .session
            .liveliness()
            .declare_token((*lv_ke).clone())
            .wait()?;
        let gid = crate::entity::endpoint_gid(&self.entity);

        Ok(ZPub {
            entity: self.entity,
            sn: AtomicUsize::new(0),
            inner,
            _lv_token: lv_token,
            gid,
            events_mgr: Arc::new(Mutex::new(EventsManager::new(gid))),
            with_attachment: self.with_attachment,
            shm_config: self.shm_config,
            dyn_schema: self.dyn_schema,
            _phantom_data: Default::default(),
        })
    }
}

impl<T, S> ZPub<T, S>
where
    T: ZMessage + 'static,
    S: for<'a> ZSerializer<Input<'a> = &'a T> + 'static,
{
    fn new_attchment(&self) -> Attachment {
        let sn = self.sn.fetch_add(1, Ordering::Relaxed);
        trace!(
            "[PUB] Creating attachment: sn={}, gid={:02x?}",
            sn,
            &self.gid[..4]
        );
        Attachment::new(sn as _, self.gid)
    }

    #[tracing::instrument(name = "publish", skip(self, msg), fields(
        topic = %self.entity.topic,
        sn = self.sn.load(Ordering::Acquire),
        payload_len = tracing::field::Empty,
        used_shm = tracing::field::Empty
    ))]
    pub fn publish(&self, msg: &T) -> Result<()> {
        use zenoh_buffers::buffer::Buffer;

        // Try direct SHM serialization if configured
        let (zbuf, actual_size) = if let Some(ref shm_cfg) = self.shm_config {
            let estimated_size = msg.estimated_serialized_size();

            // Only use SHM if estimated size meets threshold
            if estimated_size >= shm_cfg.threshold() {
                match S::serialize_to_shm(msg, estimated_size, shm_cfg.provider()) {
                    Ok((zbuf, actual_size)) => {
                        tracing::Span::current().record("used_shm", true);
                        debug!(
                            "[PUB] Serialized {}B directly to SHM (estimated: {}B)",
                            actual_size, estimated_size
                        );
                        (zbuf, actual_size)
                    }
                    Err(e) => {
                        tracing::Span::current().record("used_shm", false);
                        warn!(
                            "[PUB] Direct SHM serialization failed: {}. Using regular memory",
                            e
                        );
                        let zbuf = S::serialize_to_zbuf(msg);
                        let size = zbuf.len();
                        (zbuf, size)
                    }
                }
            } else {
                tracing::Span::current().record("used_shm", false);
                trace!(
                    "[PUB] Estimated size {}B < threshold {}B, using regular memory",
                    estimated_size,
                    shm_cfg.threshold()
                );
                let zbuf = S::serialize_to_zbuf(msg);
                let size = zbuf.len();
                (zbuf, size)
            }
        } else {
            tracing::Span::current().record("used_shm", false);
            let zbuf = S::serialize_to_zbuf(msg);
            let size = zbuf.len();
            (zbuf, size)
        };

        tracing::Span::current().record("payload_len", actual_size);

        let zbytes = zenoh::bytes::ZBytes::from(zbuf);

        let mut put_builder = self.inner.put(zbytes);
        if self.with_attachment {
            let att = self.new_attchment();
            let sn = att.sequence_number;
            put_builder = put_builder.attachment(att);
            trace!("[PUB] Attached sn={}", sn);
        }

        put_builder.wait()
    }

    pub async fn async_publish(&self, msg: &T) -> Result<()> {
        // Try direct SHM serialization if configured
        let zbuf = if let Some(ref shm_cfg) = self.shm_config {
            let estimated_size = msg.estimated_serialized_size();

            if estimated_size >= shm_cfg.threshold() {
                match S::serialize_to_shm(msg, estimated_size, shm_cfg.provider()) {
                    Ok((zbuf, _actual_size)) => zbuf,
                    Err(_) => S::serialize_to_zbuf(msg),
                }
            } else {
                S::serialize_to_zbuf(msg)
            }
        } else {
            S::serialize_to_zbuf(msg)
        };

        let zbytes = zenoh::bytes::ZBytes::from(zbuf);
        let mut put_builder = self.inner.put(zbytes);
        if self.with_attachment {
            put_builder = put_builder.attachment(self.new_attchment());
        }
        put_builder.await
    }

    /// Publish pre-serialized data directly
    ///
    /// Accepts any type that implements `Into<ZBytes>`:
    /// - `&[u8]` - byte slice
    /// - `Vec<u8>` - owned bytes
    /// - `ZBuf` - zero-copy buffer (preferred for performance)
    /// - `ZBytes` - zenoh bytes
    pub fn publish_serialized(&self, data: impl Into<zenoh::bytes::ZBytes>) -> Result<()> {
        let mut put_builder = self.inner.put(data);
        if self.with_attachment {
            put_builder = put_builder.attachment(self.new_attchment());
        }
        put_builder.wait()
    }

    pub fn publish_sample(&self, msg: &Sample) -> Result<()> {
        let payload = msg.payload().to_bytes();
        // NOTE: pass by reference to avoid copy
        let mut put_builder = self.inner.put(&payload);
        if self.with_attachment {
            put_builder = put_builder.attachment(self.new_attchment());
        }
        put_builder.wait()
    }

    pub fn events_mgr(&self) -> &Arc<Mutex<EventsManager>> {
        &self.events_mgr
    }
}

// Specialized implementation for DynamicMessage publisher
impl ZPub<crate::dynamic::DynamicMessage, crate::dynamic::DynamicCdrSerdes> {
    /// Get the dynamic schema used by this publisher.
    ///
    /// Returns `None` if the publisher was not created with `.with_dyn_schema()`.
    pub fn schema(&self) -> Option<&crate::dynamic::schema::MessageSchema> {
        self.dyn_schema.as_ref().map(|s| s.as_ref())
    }
}

pub struct ZSubBuilder<T, S = CdrSerdes<T>> {
    pub entity: EndpointEntity,
    pub session: Arc<Session>,
    pub(crate) keyexpr_format: ros_z_protocol::KeyExprFormat,
    pub dyn_schema: Option<Arc<crate::dynamic::schema::MessageSchema>>,
    pub locality: Option<zenoh::sample::Locality>,
    pub _phantom_data: PhantomData<(T, S)>,
}

impl<T, S> ZSubBuilder<T, S>
where
    T: ZMessage,
{
    pub fn with_qos(mut self, qos: QosProfile) -> Self {
        self.entity.qos = qos.to_protocol_qos();
        self
    }

    pub fn with_serdes<S2>(self) -> ZSubBuilder<T, S2> {
        ZSubBuilder {
            entity: self.entity,
            session: self.session,
            keyexpr_format: self.keyexpr_format,
            dyn_schema: self.dyn_schema,
            locality: self.locality,
            _phantom_data: PhantomData,
        }
    }

    /// Set the locality restriction for this subscription.
    ///
    /// This restricts the subscription to only receive samples from publishers
    /// with the specified locality (local/remote/any).
    ///
    /// # Example
    ///
    /// ```ignore
    /// use zenoh::sample::Locality;
    ///
    /// let subscriber = node
    ///     .create_sub::<String>("/topic")
    ///     .with_locality(Locality::Remote)  // Only receive from remote publishers
    ///     .build()?;
    /// ```
    pub fn with_locality(mut self, locality: zenoh::sample::Locality) -> Self {
        self.locality = Some(locality);
        self
    }

    /// Set the dynamic message schema for runtime-typed messages.
    ///
    /// This is required when using `DynamicMessage` with `DynamicCdrSerdes`.
    /// The schema will be used to deserialize incoming messages.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let subscriber = node
    ///     .create_sub::<DynamicMessage>("/topic")
    ///     .with_serdes::<DynamicCdrSerdes>()
    ///     .with_dyn_schema(schema)
    ///     .build()?;
    /// ```
    pub fn with_dyn_schema(mut self, schema: Arc<crate::dynamic::schema::MessageSchema>) -> Self {
        use crate::dynamic::MessageSchemaTypeDescription;

        // Only compute and set type_info if it hasn't been set already
        // (e.g., from create_dyn_sub_auto which provides the publisher's hash)
        if self.entity.type_info.is_none() {
            // Compute TypeInfo from schema for proper key expression matching with ROS 2
            // Convert ROS 2 canonical name to DDS name
            // "std_msgs/msg/String" → "std_msgs::msg::dds_::String_"
            let dds_name = schema
                .type_name
                .replace("/msg/", "::msg::dds_::")
                .replace("/srv/", "::srv::dds_::")
                .replace("/action/", "::action::dds_::")
                + "_";

            // Convert schema TypeHash to entity TypeHash via RIHS string
            let type_hash = match schema.compute_type_hash() {
                Ok(hash) => {
                    let rihs_string = hash.to_rihs_string();
                    crate::entity::TypeHash::from_rihs_string(&rihs_string)
                        .unwrap_or_else(crate::entity::TypeHash::zero)
                }
                Err(_) => crate::entity::TypeHash::zero(),
            };

            self.entity.type_info = Some(crate::entity::TypeInfo {
                name: dds_name,
                hash: type_hash,
            });
        }

        self.dyn_schema = Some(schema);
        self
    }

    /// Internal method that all build variants use.
    fn build_internal<Q>(
        mut self,
        handler: DataHandler<Sample>,
        queue: Option<Arc<BoundedQueue<Q>>>,
    ) -> Result<ZSub<T, Q, S>>
    where
        S: ZDeserializer,
    {
        let qualified_topic = topic_name::qualify_topic_name(
            &self.entity.topic,
            &self.entity.node.namespace,
            &self.entity.node.name,
        )
        .map_err(|e| zenoh::Error::from(format!("Failed to qualify topic: {}", e)))?;

        self.entity.topic = qualified_topic.clone();
        debug!("[SUB] Qualified topic: {}", qualified_topic);

        let topic_ke = self.keyexpr_format.topic_key_expr(&self.entity)?;
        let key_expr = (*topic_ke).clone(); // Deref and clone the KeyExpr
        debug!(
            "[SUB] Key expression: {}, qos={:?}",
            key_expr, self.entity.qos
        );

        let mut sub_builder = self
            .session
            .declare_subscriber(key_expr)
            .callback(move |sample| handler.handle(sample));

        // Apply locality restriction if specified
        if let Some(locality) = self.locality {
            sub_builder = sub_builder.allowed_origin(locality);
            debug!("[SUB] Locality restriction: {:?}", locality);
        }

        let inner = sub_builder.wait()?;

        let gid = crate::entity::endpoint_gid(&self.entity);
        let lv_ke = self
            .keyexpr_format
            .liveliness_key_expr(&self.entity, &self.session.zid())?;
        let lv_token = self
            .session
            .liveliness()
            .declare_token((*lv_ke).clone())
            .wait()?;

        debug!("[SUB] Subscriber ready: topic={}", self.entity.topic);

        Ok(ZSub {
            entity: self.entity,
            _inner: inner,
            _lv_token: lv_token,
            queue,
            events_mgr: Arc::new(Mutex::new(EventsManager::new(gid))),
            dyn_schema: self.dyn_schema,
            _phantom_data: Default::default(),
        })
    }

    /// Build a subscriber with a callback that processes deserialized messages directly.
    ///
    /// This method creates a subscriber that invokes the provided callback for each
    /// received message, bypassing the internal queue. The callback receives the
    /// deserialized message directly. Liveliness tokens and event management are
    /// preserved.
    ///
    /// # Arguments
    ///
    /// * `callback` - A function that will be called with each deserialized message
    ///
    /// # Returns
    ///
    /// A `ZSub` with no internal queue (callback-only mode)
    pub fn build_with_callback<F>(self, callback: F) -> Result<ZSub<T, (), S>>
    where
        F: Fn(S::Output) + Send + Sync + 'static,
        S: for<'a> ZDeserializer<Input<'a> = &'a [u8]> + 'static,
    {
        let callback = Arc::new(move |sample: Sample| {
            let payload = sample.payload().to_bytes();
            match S::deserialize(&payload) {
                Ok(msg) => callback(msg),
                Err(e) => tracing::error!("Failed to deserialize message: {}", e),
            }
        });

        self.build_internal(DataHandler::Callback(callback), None)
    }

    #[cfg(feature = "rmw")]
    pub fn build_with_notifier<F>(self, notify: F) -> Result<ZSub<T, Sample, S>>
    where
        F: Fn() + Send + Sync + 'static,
        S: ZDeserializer,
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

impl<T, S> Builder for ZSubBuilder<T, S>
where
    T: ZMessage + 'static + Sync + Send,
    S: ZDeserializer,
{
    type Output = ZSub<T, Sample, S>;

    fn build(self) -> Result<Self::Output> {
        let queue_size = match self.entity.qos.history {
            QosHistory::KeepLast(depth) => depth,
            QosHistory::KeepAll => usize::MAX,
        };
        let queue = Arc::new(BoundedQueue::new(queue_size));

        self.build_internal(DataHandler::Queue(queue.clone()), Some(queue))
    }
}

pub struct ZSub<T: ZMessage, Q, S: ZDeserializer> {
    pub entity: EndpointEntity,
    pub queue: Option<Arc<BoundedQueue<Q>>>,
    _inner: zenoh::pubsub::Subscriber<()>,
    _lv_token: LivelinessToken,
    events_mgr: Arc<Mutex<EventsManager>>,
    /// Schema for dynamic message deserialization.
    /// Required when using `DynamicMessage` with `DynamicCdrSerdes`.
    pub dyn_schema: Option<Arc<crate::dynamic::schema::MessageSchema>>,
    _phantom_data: PhantomData<(T, Q, S)>,
}

impl<T, S> ZSub<T, Sample, S>
where
    T: ZMessage,
    S: ZDeserializer,
{
    /// Receive the next serialized message (raw sample)
    pub fn recv_serialized(&self) -> Result<Sample> {
        let queue = self.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Subscriber was built with callback, no queue available")
        })?;
        Ok(queue.recv())
    }

    /// Async receive the next serialized message (raw sample)
    pub async fn async_recv_serialized(&self) -> Result<Sample> {
        let queue = self.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Subscriber was built with callback, no queue available")
        })?;
        Ok(queue.recv_async().await)
    }

    /// Receive the next serialized message with timeout
    pub fn recv_serialized_timeout(&self, timeout: Duration) -> Result<Sample> {
        let queue = self.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Subscriber was built with callback, no queue available")
        })?;
        queue
            .recv_timeout(timeout)
            .ok_or_else(|| zenoh::Error::from("Receive timed out"))
    }

    pub fn events_mgr(&self) -> &Arc<Mutex<EventsManager>> {
        &self.events_mgr
    }

    /// Check if there are messages available in the queue
    pub fn is_ready(&self) -> bool {
        self.queue.as_ref().map(|q| !q.is_empty()).unwrap_or(false)
    }
}

impl<T, S> ZSub<T, Sample, S>
where
    T: ZMessage,
    S: for<'a> ZDeserializer<Input<'a> = &'a [u8]>,
{
    /// Receive and deserialize the next message (aligned with ROS behavior)
    #[tracing::instrument(name = "recv", skip(self), fields(
        topic = %self.entity.topic,
        payload_len = tracing::field::Empty
    ))]
    pub fn recv(&self) -> Result<S::Output> {
        trace!("[SUB] Waiting for message");

        let queue = self.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Subscriber was built with callback, no queue available")
        })?;
        let sample = queue.recv();
        let payload = sample.payload().to_bytes();

        tracing::Span::current().record("payload_len", payload.len());
        debug!("[SUB] Received message");

        S::deserialize(&payload).map_err(|e| zenoh::Error::from(e.to_string()))
    }

    pub fn recv_timeout(&self, timeout: Duration) -> Result<S::Output> {
        let queue = self.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Subscriber was built with callback, no queue available")
        })?;
        let sample = queue
            .recv_timeout(timeout)
            .ok_or_else(|| zenoh::Error::from("Receive timed out"))?;
        let payload = sample.payload().to_bytes();
        S::deserialize(&payload).map_err(|e| zenoh::Error::from(e.to_string()))
    }

    /// Async receive and deserialize the next message
    pub async fn async_recv(&self) -> Result<S::Output> {
        let queue = self.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Subscriber was built with callback, no queue available")
        })?;
        let sample = queue.recv_async().await;
        let payload = sample.payload().to_bytes();
        S::deserialize(&payload).map_err(|e| zenoh::Error::from(e.to_string()))
    }
}

// Specialized implementation for DynamicMessage
impl ZSub<crate::dynamic::DynamicMessage, Sample, crate::dynamic::DynamicCdrSerdes> {
    /// Receive and deserialize the next dynamic message.
    ///
    /// This method requires that the subscriber was built with `.with_dyn_schema()`.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - The subscriber was built with a callback (no queue available)
    /// - The `dyn_schema` was not set via `.with_dyn_schema()`
    /// - Deserialization fails
    #[tracing::instrument(name = "recv_dynamic", skip(self), fields(
        topic = %self.entity.topic,
        payload_len = tracing::field::Empty
    ))]
    pub fn recv(&self) -> Result<crate::dynamic::DynamicMessage> {
        let schema = self.dyn_schema.as_ref().ok_or_else(|| {
            zenoh::Error::from(
                "dyn_schema required for DynamicMessage (use .with_dyn_schema() when building)",
            )
        })?;

        let queue = self.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Subscriber was built with callback, no queue available")
        })?;

        trace!("[SUB] Waiting for dynamic message");
        let sample = queue.recv();
        let payload = sample.payload().to_bytes();

        tracing::Span::current().record("payload_len", payload.len());
        debug!("[SUB] Received dynamic message");

        crate::dynamic::DynamicCdrSerdes::deserialize((&payload, schema))
            .map_err(|e| zenoh::Error::from(e.to_string()))
    }

    /// Receive a dynamic message with timeout.
    pub fn recv_timeout(&self, timeout: Duration) -> Result<crate::dynamic::DynamicMessage> {
        let schema = self
            .dyn_schema
            .as_ref()
            .ok_or_else(|| zenoh::Error::from("dyn_schema required for DynamicMessage"))?;

        let queue = self.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Subscriber was built with callback, no queue available")
        })?;

        let sample = queue
            .recv_timeout(timeout)
            .ok_or_else(|| zenoh::Error::from("Receive timed out"))?;
        let payload = sample.payload().to_bytes();

        crate::dynamic::DynamicCdrSerdes::deserialize((&payload, schema))
            .map_err(|e| zenoh::Error::from(e.to_string()))
    }

    /// Async receive a dynamic message.
    pub async fn async_recv(&self) -> Result<crate::dynamic::DynamicMessage> {
        let schema = self
            .dyn_schema
            .as_ref()
            .ok_or_else(|| zenoh::Error::from("dyn_schema required for DynamicMessage"))?;

        let queue = self.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Subscriber was built with callback, no queue available")
        })?;

        let sample = queue.recv_async().await;
        let payload = sample.payload().to_bytes();

        crate::dynamic::DynamicCdrSerdes::deserialize((&payload, schema))
            .map_err(|e| zenoh::Error::from(e.to_string()))
    }

    /// Try to receive a dynamic message without blocking.
    pub fn try_recv(&self) -> Option<Result<crate::dynamic::DynamicMessage>> {
        let schema = self.dyn_schema.as_ref()?;
        let queue = self.queue.as_ref()?;

        match queue.try_recv() {
            Some(sample) => {
                let payload = sample.payload().to_bytes();
                Some(
                    crate::dynamic::DynamicCdrSerdes::deserialize((&payload, schema))
                        .map_err(|e| zenoh::Error::from(e.to_string())),
                )
            }
            None => None,
        }
    }

    /// Get the dynamic schema.
    pub fn schema(&self) -> Option<&crate::dynamic::schema::MessageSchema> {
        self.dyn_schema.as_ref().map(|s| s.as_ref())
    }
}
