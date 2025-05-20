use std::sync::atomic::AtomicUsize;
use std::sync::atomic::Ordering::AcqRel;
use std::time::{SystemTime, UNIX_EPOCH};
use std::{marker::PhantomData, sync::Arc};

use serde::Deserialize;
use zenoh::{Result, Session, Wait, sample::Sample};

use crate::Builder;
use crate::attachment::{Attachment, GidArray};
use crate::entity::{EndpointEntity, TypeInfo};
use crate::msg::{CdrSerdes, ZMessage};

pub struct ZPub<T: ZMessage> {
    // TODO: replace this with the sample sn
    sn: AtomicUsize,
    // TODO: replace this with zenoh's global entity id
    gid: GidArray,
    inner: zenoh::pubsub::Publisher<'static>,
    _phantom_data: PhantomData<T>,
}

#[derive(Debug)]
pub struct ZPubBuilder<T> {
    pub entity: EndpointEntity,
    pub session: Arc<Session>,
    pub _phantom_data: PhantomData<T>,
}

macro_rules! impl_with_type_info {
    ($type:ident<$t:ident>) => {
        impl<$t> $type<$t> {
            pub fn with_type_info(mut self, type_info: TypeInfo) -> Self {
                self.entity.type_info = Some(type_info);
                self
            }
        }
    };
}

impl_with_type_info!(ZPubBuilder<T>);
impl_with_type_info!(ZSubBuilder<T>);
// impl_with_type_info!(ZSubBuilder<T, true>);

impl<T> Builder for ZPubBuilder<T>
where
    T: ZMessage,
{
    type Output = ZPub<T>;

    fn build(self) -> Result<Self::Output> {
        let key_expr = self.entity.topic_key_expr()?;
        tracing::error!("[PUB] KE: {key_expr}");
        let zpub = self.session.declare_publisher(key_expr).wait()?;
        let gid = self.entity.gid();
        Ok(ZPub {
            sn: AtomicUsize::new(0),
            inner: zpub,
            gid,
            _phantom_data: Default::default(),
        })
    }
}

impl<T> ZPub<T>
where
    T: ZMessage,
{
    pub fn new_attchment(&self) -> Attachment {
        Attachment {
            sequence_number: self.sn.fetch_add(1, AcqRel) as _,
            source_timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map_or(0, |v| v.as_nanos() as _),
            source_gid: self.gid,
        }
    }

    pub fn publish(&self, msg: &T) -> Result<()> {
        self.inner
            .put(msg.serialize())
            .attachment(self.new_attchment())
            .wait()
    }

    pub async fn async_publish(&self, msg: &T) -> Result<()> {
        self.inner.put(msg.serialize()).await
    }

    pub fn publish_serialized_message(&self, msg: &[u8]) -> Result<()> {
        self.inner.put(msg).wait()
    }

    pub fn publish_sample(&self, msg: &Sample) -> Result<()> {
        self.inner.put(msg.payload().to_bytes()).wait()
    }
}

pub struct ZSubBuilder<T, const POST_DESERIALIZATION: bool = false> {
    pub entity: EndpointEntity,
    pub session: Arc<Session>,
    pub _phantom_data: PhantomData<T>,
}

impl<T> ZSubBuilder<T, false>
where
    T: ZMessage,
{
    pub fn post_deserialization(self) -> ZSubBuilder<T, true> {
        ZSubBuilder {
            entity: self.entity,
            session: self.session,
            _phantom_data: self._phantom_data,
        }
    }
}

impl<T> Builder for ZSubBuilder<T, false>
where
    for<'c> T: ZMessage<Serdes = CdrSerdes<T>> + Deserialize<'c> + Send + Sync + 'static,
{
    type Output = ZSub<T, T>;

    fn build(self) -> Result<Self::Output> {
        let (tx, rx) = flume::bounded(10);
        let inner = self
            .session
            .declare_subscriber(self.entity.topic_key_expr()?)
            .callback(move |sample| {
                let msg = <T as ZMessage>::deserialize(&sample.payload().to_bytes());
                tx.send(msg).unwrap();
            })
            .wait()?;
        Ok(ZSub {
            _inner: inner,
            queue: rx,
            _phantom_data: Default::default(),
        })
    }
}

impl<T> ZSubBuilder<T, true>
where
    T: ZMessage,
{
    pub fn build_with_notifier<F>(self, notify: F) -> Result<ZSub<T, Sample>>
    where
        F: Fn() + Send + Sync + 'static,
    {
        let (tx, rx) = flume::bounded(10);
        let inner = self
            .session
            .declare_subscriber(self.entity.topic_key_expr()?)
            .callback(move |sample| {
                let _ = tx.send(sample);
                notify();
            })
            .wait()?;
        Ok(ZSub {
            _inner: inner,
            queue: rx,
            _phantom_data: Default::default(),
        })
    }
}

impl<T> Builder for ZSubBuilder<T, true>
where
    T: ZMessage + 'static + Sync + Send,
{
    type Output = ZSub<T, Sample>;

    fn build(self) -> Result<Self::Output> {
        let (tx, rx) = flume::bounded(10);
        let inner = self
            .session
            .declare_subscriber(self.entity.topic_key_expr()?)
            .callback(move |sample| {
                let _ = tx.send(sample);
            })
            .wait()?;
        Ok(Self::Output {
            _inner: inner,
            queue: rx,
            _phantom_data: Default::default(),
        })
    }
}

pub struct ZSub<T: ZMessage, Q> {
    _inner: zenoh::pubsub::Subscriber<()>,
    pub queue: flume::Receiver<Q>,
    _phantom_data: PhantomData<T>,
}

impl<T, Q> ZSub<T, Q>
where
    T: ZMessage,
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

impl<T> ZSub<T, Sample>
where
    T: ZMessage,
{
    pub fn recv_sample(&self) -> Result<Sample> {
        let msg = self.queue.recv()?;
        Ok(msg)
    }
}
