use std::sync::atomic::AtomicUsize;
use std::sync::atomic::Ordering::AcqRel;
use std::{marker::PhantomData, sync::Arc};

use serde::Deserialize;
use zenoh::liveliness::LivelinessToken;
use zenoh::{Result, Session, Wait, sample::Sample};

use crate::Builder;
use crate::attachment::{Attachment, GidArray};
use crate::entity::EndpointEntity;
use crate::impl_with_type_info;

use crate::qos::{QosProfile, QosReliability, QosDurability, QosHistory};
use crate::msg::{CdrSerdes, ZMessage, ZSerializer, ZDeserializer};
#[cfg(feature = "protobuf")]
use crate::msg::ProtobufSerdes;

pub struct ZPub<T: ZMessage, S: ZSerializer> {
    // TODO: replace this with the sample sn
    sn: AtomicUsize,
    // TODO: replace this with zenoh's global entity id
    gid: GidArray,
    inner: zenoh::pubsub::Publisher<'static>,
    _lv_token: LivelinessToken,
    _phantom_data: PhantomData<(T, S)>,
}

#[derive(Debug)]
pub struct ZPubBuilder<T, S = CdrSerdes<T>> {
    pub entity: EndpointEntity,
    pub session: Arc<Session>,
    pub _phantom_data: PhantomData<(T, S)>,
}

impl_with_type_info!(ZPubBuilder<T, S>);
impl_with_type_info!(ZSubBuilder<T, S>);

impl<T, S> ZPubBuilder<T, S> {
    pub fn with_qos(mut self, qos: QosProfile) -> Self {
        self.entity.qos = qos;
        self
    }

    pub fn with_serdes<S2>(self) -> ZPubBuilder<T, S2> {
        ZPubBuilder {
            entity: self.entity,
            session: self.session,
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

    fn build(self) -> Result<Self::Output> {
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
        Ok(ZPub {
            sn: AtomicUsize::new(0),
            inner,
            _lv_token: lv_token,
            gid: self.entity.gid(),
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
        self.inner
            .put(S::serialize(msg))
            .attachment(self.new_attchment())
            .wait()
    }

    pub async fn async_publish(&self, msg: &T) -> Result<()> {
        self.inner.put(S::serialize(msg)).await
    }

    pub fn publish_serialized_message(&self, msg: &[u8]) -> Result<()> {
        self.inner.put(msg).wait()
    }

    pub fn publish_sample(&self, msg: &Sample) -> Result<()> {
        self.inner.put(msg.payload().to_bytes()).wait()
    }
}

pub struct ZSubBuilder<T, S = CdrSerdes<T>, const POST_DESERIALIZATION: bool = false> {
    pub entity: EndpointEntity,
    pub session: Arc<Session>,
    pub _phantom_data: PhantomData<(T, S)>,
}

impl<T, S> ZSubBuilder<T, S, false>
where
    T: ZMessage,
{
    pub fn with_qos(mut self, qos: QosProfile) -> Self {
        self.entity.qos = qos;
        self
    }

    pub fn post_deserialization(self) -> ZSubBuilder<T, S, true> {
        ZSubBuilder {
            entity: self.entity,
            session: self.session,
            _phantom_data: self._phantom_data,
        }
    }

    pub fn with_serdes<S2>(self) -> ZSubBuilder<T, S2, false> {
        ZSubBuilder {
            entity: self.entity,
            session: self.session,
            _phantom_data: PhantomData,
        }
    }
}

impl<T, S> Builder for ZSubBuilder<T, S, false>
where
    for<'c> T: ZMessage<Serdes = CdrSerdes<T>> + Deserialize<'c> + Send + Sync + 'static,
    S: for<'a> ZDeserializer<Input<'a> = &'a [u8], Output = T> + Send + Sync + 'static,
{
    type Output = ZSub<T, T, S>;

    fn build(self) -> Result<Self::Output> {
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
                let msg = S::deserialize(&sample.payload().to_bytes());
                tx.send(msg).unwrap();
            })
            .wait()?;
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
            _phantom_data: Default::default(),
        })
    }
}

impl<T, S> ZSubBuilder<T, S, true>
where
    T: ZMessage,
{
    pub fn with_qos(mut self, qos: QosProfile) -> Self {
        self.entity.qos = qos;
        self
    }
}

impl<T, S> Builder for ZSubBuilder<T, S, true>
where
    T: ZMessage + 'static + Sync + Send,
    S: for<'a> ZDeserializer<Input<'a> = &'a [u8], Output = T> + Send + Sync + 'static,
{
    type Output = ZSub<T, Sample, S>;

    fn build(self) -> Result<Self::Output> {
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
            _phantom_data: Default::default(),
        })
    }
}

pub struct ZSub<T: ZMessage, Q, S: ZDeserializer> {
    pub entity: EndpointEntity,
    pub queue: flume::Receiver<Q>,
    _inner: zenoh::pubsub::Subscriber<()>,
    _lv_token: LivelinessToken,
    _phantom_data: PhantomData<(T, S)>,
}

impl<T, Q, S> ZSub<T, Q, S>
where
    T: ZMessage,
    S: ZDeserializer,
{
    pub fn recv(&self) -> Result<Q> {
        let msg = self.queue.recv()?;
        Ok(msg)
    }

    pub async fn async_recv(&self) -> Result<Q> {
        let msg = self.queue.recv_async().await?;
        Ok(msg)
    }
}

impl<T, S> ZSub<T, Sample, S>
where
    T: ZMessage,
    S: ZDeserializer,
{
    pub fn recv_sample(&self) -> Result<Sample> {
        let msg = self.queue.recv()?;
        Ok(msg)
    }
}
