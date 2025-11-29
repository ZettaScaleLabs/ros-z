use std::sync::atomic::AtomicUsize;
use std::sync::atomic::Ordering::AcqRel;
use std::time::Duration;
use std::{marker::PhantomData, sync::Arc};

use zenoh::liveliness::LivelinessToken;
use zenoh::{Result, Session, Wait, sample::Sample};

use crate::Builder;
use crate::attachment::{Attachment, GidArray};
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

    fn build(mut self) -> Result<Self::Output> {
        // Qualify the topic name according to ROS 2 rules
        let qualified_topic = topic_name::qualify_topic_name(
            &self.entity.topic,
            &self.entity.node.namespace,
            &self.entity.node.name,
        )
        .map_err(|e| zenoh::Error::from(format!("Failed to qualify topic: {}", e)))?;

        self.entity.topic = qualified_topic;

        let key_expr = self.entity.topic_key_expr()?;
        tracing::debug!("[PUB] KE: {key_expr}");

        // Map QoS to Zenoh publisher settings
        let mut pub_builder = self.session.declare_publisher(key_expr);

        // Map reliability: Reliable uses Block, BestEffort uses Drop
        match self.entity.qos.reliability {
            QosReliability::Reliable => {
                pub_builder = pub_builder.congestion_control(zenoh::qos::CongestionControl::Block);
            }
            QosReliability::BestEffort => {
                pub_builder = pub_builder.congestion_control(zenoh::qos::CongestionControl::Drop);
            }
        }

        // Map durability: TransientLocal uses express=true for caching
        match self.entity.qos.durability {
            QosDurability::TransientLocal => {
                pub_builder = pub_builder.express(true);
            }
            QosDurability::Volatile => {
                pub_builder = pub_builder.express(false);
            }
        }

        let inner = pub_builder.wait()?;
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
        Attachment::new(self.sn.fetch_add(1, AcqRel) as _, self.gid)
    }

    pub fn publish(&self, msg: &T) -> Result<()> {
        let mut put_builder = self.inner.put(S::serialize(msg));
        if self.with_attachment {
            put_builder = put_builder.attachment(self.new_attchment());
        }
        put_builder.wait()
    }

    pub async fn async_publish(&self, msg: &T) -> Result<()> {
        let mut put_builder = self.inner.put(S::serialize(msg));
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
        let mut put_builder = self.inner.put(msg.payload().to_bytes());
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

    #[cfg(feature = "rcl-z")]
    pub fn build_with_notifier<F>(mut self, notify: F) -> Result<ZSub<T, Sample, S>>
    where
        F: Fn() + Send + Sync + 'static,
        S: ZDeserializer,
    {
        // Qualify the topic name according to ROS 2 rules
        let qualified_topic = topic_name::qualify_topic_name(
            &self.entity.topic,
            &self.entity.node.namespace,
            &self.entity.node.name,
        )
        .map_err(|e| zenoh::Error::from(format!("Failed to qualify topic: {}", e)))?;

        self.entity.topic = qualified_topic;

        // Map QoS history to queue size
        let queue_size = match self.entity.qos.history {
            QosHistory::KeepLast(depth) => depth,
            QosHistory::KeepAll => 100, // Use a reasonable default for KeepAll
        };

        let (tx, rx) = flume::bounded(queue_size);
        let inner = self
            .session
            .declare_subscriber(self.entity.topic_key_expr()?)
            .callback(move |sample| {
                let _ = tx.send(sample);
                notify();
            })
            .wait()?;
        let gid = self.entity.gid();
        let lv_token = self
            .session
            .liveliness()
            .declare_token(self.entity.lv_token_key_expr()?)
            .wait()?;
        Ok(ZSub {
            entity: self.entity,
            _inner: inner,
            _lv_token: lv_token,
            queue: rx,
            events_mgr: Arc::new(Mutex::new(EventsManager::new(gid))),
            _phantom_data: Default::default(),
        })
    }
}

impl<T, S> Builder for ZSubBuilder<T, S>
where
    T: ZMessage + 'static + Sync + Send,
    S: ZDeserializer,
{
    type Output = ZSub<T, Sample, S>;

    fn build(mut self) -> Result<Self::Output> {
        // Qualify the topic name according to ROS 2 rules
        let qualified_topic = topic_name::qualify_topic_name(
            &self.entity.topic,
            &self.entity.node.namespace,
            &self.entity.node.name,
        )
        .map_err(|e| zenoh::Error::from(format!("Failed to qualify topic: {}", e)))?;

        self.entity.topic = qualified_topic;

        // Map QoS history to queue size
        let queue_size = match self.entity.qos.history {
            QosHistory::KeepLast(depth) => depth,
            QosHistory::KeepAll => 100, // Use a reasonable default for KeepAll
        };

        let (tx, rx) = flume::bounded(queue_size);
        let inner = self
            .session
            .declare_subscriber(self.entity.topic_key_expr()?)
            .callback(move |sample| {
                let _ = tx.send(sample);
            })
            .wait()?;
        let gid = self.entity.gid();
        let lv_token = self
            .session
            .liveliness()
            .declare_token(self.entity.lv_token_key_expr()?)
            .wait()?;
        Ok(Self::Output {
            entity: self.entity,
            _inner: inner,
            _lv_token: lv_token,
            queue: rx,
            events_mgr: Arc::new(Mutex::new(EventsManager::new(gid))),
            _phantom_data: Default::default(),
        })
    }
}

pub struct ZSub<T: ZMessage, Q, S: ZDeserializer> {
    pub entity: EndpointEntity,
    pub queue: flume::Receiver<Q>,
    _inner: zenoh::pubsub::Subscriber<()>,
    _lv_token: LivelinessToken,
    events_mgr: Arc<Mutex<EventsManager>>,
    _phantom_data: PhantomData<(T, S)>,
}

impl<T, S> ZSub<T, Sample, S>
where
    T: ZMessage,
    S: ZDeserializer,
{
    /// Receive the next serialized message (raw sample)
    pub fn recv_serialized(&self) -> Result<Sample> {
        let msg = self.queue.recv()?;
        Ok(msg)
    }

    /// Async receive the next serialized message (raw sample)
    pub async fn async_recv_serialized(&self) -> Result<Sample> {
        let msg = self.queue.recv_async().await?;
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
    pub fn recv(&self) -> Result<S::Output> {
        let sample = self.queue.recv()?;
        let payload = sample.payload().to_bytes();
        Ok(S::deserialize(&payload))
    }

    pub fn recv_timeout(&self, timeout: Duration) -> Result<S::Output> {
        let sample = self.queue.recv_timeout(timeout)?;
        let payload = sample.payload().to_bytes();
        Ok(S::deserialize(&payload))
    }

    /// Async receive and deserialize the next message
    pub async fn async_recv(&self) -> Result<S::Output> {
        let sample = self.queue.recv_async().await?;
        let payload = sample.payload().to_bytes();
        Ok(S::deserialize(&payload))
    }
}
