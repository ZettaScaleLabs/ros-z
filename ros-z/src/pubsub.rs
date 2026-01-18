use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;
use std::{marker::PhantomData, sync::Arc};

use zenoh::liveliness::LivelinessToken;
use zenoh::{Result, Session, Wait, sample::Sample};
use tracing::{info, debug, trace};

use crate::Builder;
use crate::attachment::{Attachment, GidArray};
use crate::common::DataHandler;
use crate::entity::EndpointEntity;
use crate::event::EventsManager;
use crate::impl_with_type_info;
use crate::topic_name;

use crate::msg::{CdrSerdes, ZDeserializer, ZMessage, ZSerializer};
use crate::qos::{QosDurability, QosHistory, QosProfile, QosReliability};
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
    typical_msg_size: AtomicUsize,
    _phantom_data: PhantomData<(T, S)>,
}

#[derive(Debug)]
pub struct ZPubBuilder<T, S = CdrSerdes<T>> {
    pub entity: EndpointEntity,
    pub session: Arc<Session>,
    pub with_attachment: bool,
    pub _phantom_data: PhantomData<(T, S)>,
}

impl_with_type_info!(ZPubBuilder<T, S>);
impl_with_type_info!(ZSubBuilder<T, S>);

impl<T, S> ZPubBuilder<T, S> {
    pub fn with_qos(mut self, qos: QosProfile) -> Self {
        self.entity.qos = qos;
        self
    }

    pub fn with_attachment(mut self, with_attachment: bool) -> Self {
        self.with_attachment = with_attachment;
        self
    }

    pub fn with_serdes<S2>(self) -> ZPubBuilder<T, S2> {
        ZPubBuilder {
            entity: self.entity,
            session: self.session,
            with_attachment: self.with_attachment,
            _phantom_data: PhantomData,
        }
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

        let key_expr = self.entity.topic_key_expr()?;
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
        info!("[PUB] Publisher ready: topic={}", self.entity.topic);

        let lv_token = self
            .session
            .liveliness()
            .declare_token(self.entity.lv_token_key_expr()?)
            .wait()?;
        let gid = self.entity.gid();

        Ok(ZPub {
            entity: self.entity,
            sn: AtomicUsize::new(0),
            inner,
            _lv_token: lv_token,
            gid,
            events_mgr: Arc::new(Mutex::new(EventsManager::new(gid))),
            with_attachment: self.with_attachment,
            typical_msg_size: AtomicUsize::new(4096),
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
        trace!("[PUB] Creating attachment: sn={}, gid={:02x?}", sn, &self.gid[..4]);
        Attachment::new(sn as _, self.gid)
    }

    #[tracing::instrument(name = "publish", skip(self, msg), fields(
        topic = %self.entity.topic,
        sn = self.sn.load(Ordering::Acquire),
        payload_len = tracing::field::Empty
    ))]
    pub fn publish(&self, msg: &T) -> Result<()> {
        // Serialize directly to ZBuf for zero-copy publishing
        let zbuf = S::serialize_to_zbuf(msg);

        // Update size estimate for next publish (used for metrics/debugging)
        use zenoh_buffers::buffer::Buffer;
        let actual_size = zbuf.len();
        self.typical_msg_size.store(actual_size, Ordering::Relaxed);

        tracing::Span::current().record("payload_len", actual_size);
        debug!("[PUB] Publishing message");

        // Convert ZBuf to ZBytes and publish
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
        // Serialize directly to ZBuf for zero-copy publishing
        let zbuf = S::serialize_to_zbuf(msg);

        // Update size estimate
        use zenoh_buffers::buffer::Buffer;
        let actual_size = zbuf.len();
        self.typical_msg_size.store(actual_size, Ordering::Relaxed);

        // Convert ZBuf to ZBytes and publish
        let zbytes = zenoh::bytes::ZBytes::from(zbuf);
        let mut put_builder = self.inner.put(zbytes);
        if self.with_attachment {
            put_builder = put_builder.attachment(self.new_attchment());
        }
        put_builder.await
    }

    pub fn publish_serialized_message(&self, msg: &[u8]) -> Result<()> {
        let mut put_builder = self.inner.put(msg);
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

pub struct ZSubBuilder<T, S = CdrSerdes<T>> {
    pub entity: EndpointEntity,
    pub session: Arc<Session>,
    pub _phantom_data: PhantomData<(T, S)>,
}

impl<T, S> ZSubBuilder<T, S>
where
    T: ZMessage,
{
    pub fn with_qos(mut self, qos: QosProfile) -> Self {
        self.entity.qos = qos;
        self
    }

    pub fn with_serdes<S2>(self) -> ZSubBuilder<T, S2> {
        ZSubBuilder {
            entity: self.entity,
            session: self.session,
            _phantom_data: PhantomData,
        }
    }

    /// Internal method that all build variants use.
    fn build_internal<Q>(
        mut self,
        handler: DataHandler<Sample>,
        create_queue: impl FnOnce() -> Option<flume::Receiver<Q>>,
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

        let key_expr = self.entity.topic_key_expr()?;
        debug!("[SUB] Key expression: {}, qos={:?}", key_expr, self.entity.qos);

        let inner = self
            .session
            .declare_subscriber(key_expr)
            .callback(move |sample| handler.handle(sample))
            .wait()?;

        let gid = self.entity.gid();
        let lv_token = self
            .session
            .liveliness()
            .declare_token(self.entity.lv_token_key_expr()?)
            .wait()?;

        info!("[SUB] Subscriber ready: topic={}", self.entity.topic);

        Ok(ZSub {
            entity: self.entity,
            _inner: inner,
            _lv_token: lv_token,
            queue: create_queue(),
            events_mgr: Arc::new(Mutex::new(EventsManager::new(gid))),
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

        self.build_internal(DataHandler::Callback(callback), || None)
    }

    #[cfg(feature = "rcl-z")]
    pub fn build_with_notifier<F>(self, notify: F) -> Result<ZSub<T, Sample, S>>
    where
        F: Fn() + Send + Sync + 'static,
        S: ZDeserializer,
    {
        let queue_size = match self.entity.qos.history {
            QosHistory::KeepLast(depth) => depth,
            QosHistory::KeepAll => usize::MAX,
        };
        let (tx, rx) = flume::bounded(queue_size);

        self.build_internal(
            DataHandler::QueueWithNotifier {
                sender: tx,
                notifier: Arc::new(notify),
            },
            || Some(rx),
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
        let (tx, rx) = flume::bounded(queue_size);

        self.build_internal(DataHandler::Queue(tx), || Some(rx))
    }
}

pub struct ZSub<T: ZMessage, Q, S: ZDeserializer> {
    pub entity: EndpointEntity,
    pub queue: Option<flume::Receiver<Q>>,
    _inner: zenoh::pubsub::Subscriber<()>,
    _lv_token: LivelinessToken,
    events_mgr: Arc<Mutex<EventsManager>>,
    _phantom_data: PhantomData<(T, Q, S)>,
}

impl<T, S> ZSub<T, Sample, S>
where
    T: ZMessage,
    S: ZDeserializer,
{
    /// Receive the next serialized message (raw sample)
    pub fn recv_serialized(&self) -> Result<Sample> {
        let queue = self.queue.as_ref()
            .ok_or_else(|| zenoh::Error::from("Subscriber was built with callback, no queue available"))?;
        let msg = queue.recv()?;
        Ok(msg)
    }

    /// Async receive the next serialized message (raw sample)
    pub async fn async_recv_serialized(&self) -> Result<Sample> {
        let queue = self.queue.as_ref()
            .ok_or_else(|| zenoh::Error::from("Subscriber was built with callback, no queue available"))?;
        let msg = queue.recv_async().await?;
        Ok(msg)
    }

    pub fn events_mgr(&self) -> &Arc<Mutex<EventsManager>> {
        &self.events_mgr
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

        let queue = self.queue.as_ref()
            .ok_or_else(|| zenoh::Error::from("Subscriber was built with callback, no queue available"))?;
        let sample = queue.recv()?;
        let payload = sample.payload().to_bytes();

        tracing::Span::current().record("payload_len", payload.len());
        debug!("[SUB] Received message");

        S::deserialize(&payload).map_err(|e| zenoh::Error::from(e.to_string()))
    }

    pub fn recv_timeout(&self, timeout: Duration) -> Result<S::Output> {
        let queue = self.queue.as_ref()
            .ok_or_else(|| zenoh::Error::from("Subscriber was built with callback, no queue available"))?;
        let sample = queue.recv_timeout(timeout)?;
        let payload = sample.payload().to_bytes();
        S::deserialize(&payload).map_err(|e| zenoh::Error::from(e.to_string()))
    }

    /// Async receive and deserialize the next message
    pub async fn async_recv(&self) -> Result<S::Output> {
        let queue = self.queue.as_ref()
            .ok_or_else(|| zenoh::Error::from("Subscriber was built with callback, no queue available"))?;
        let sample = queue.recv_async().await?;
        let payload = sample.payload().to_bytes();
        S::deserialize(&payload).map_err(|e| zenoh::Error::from(e.to_string()))
    }
}
