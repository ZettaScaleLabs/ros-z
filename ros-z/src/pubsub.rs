use std::sync::atomic::AtomicUsize;
use std::sync::atomic::Ordering::AcqRel;
use std::{marker::PhantomData, sync::Arc};

use zenoh::liveliness::LivelinessToken;
use zenoh::{Result, Session, Wait, sample::Sample};

use crate::Builder;
use crate::attachment::{Attachment, GidArray};
use crate::entity::EndpointEntity;
use crate::impl_with_type_info;
use crate::msg::{ZDeserializer, ZMessage, ZSerializer};

pub struct ZPub<T, S = <T as ZMessage>::Serdes>
where
    T: ZMessage,
    S: for<'a> ZSerializer<Input<'a> = &'a T>,
{
    // TODO: replace this with the sample sn
    sn: AtomicUsize,
    // TODO: replace this with zenoh's global entity id
    gid: GidArray,
    inner: zenoh::pubsub::Publisher<'static>,
    _lv_token: LivelinessToken,
    _phantom_data: PhantomData<(T, S)>,
}

#[derive(Debug)]
pub struct ZPubBuilder<T, S = <T as ZMessage>::Serdes>
where
    T: ZMessage,
    S: for<'a> ZSerializer<Input<'a> = &'a T>,
{
    pub entity: EndpointEntity,
    pub session: Arc<Session>,
    pub _phantom_data: PhantomData<(T, S)>,
}

impl_with_type_info!(ZPubBuilder<T, S>);
impl_with_type_info!(ZSubBuilder<T, S>);

impl<T, S> Builder for ZPubBuilder<T, S>
where
    T: ZMessage,
    S: for<'a> ZSerializer<Input<'a> = &'a T>,
{
    type Output = ZPub<T, S>;

    fn build(self) -> Result<Self::Output> {
        let key_expr = self.entity.topic_key_expr()?;
        tracing::debug!("[PUB] KE: {key_expr}");
        let inner = self.session.declare_publisher(key_expr).wait()?;
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
    T: ZMessage,
    S: for<'a> ZSerializer<Input<'a> = &'a T>,
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

pub struct ZSubBuilder<T, S = <T as ZMessage>::Serdes>
where
    T: ZMessage,
    S: ZDeserializer,
{
    pub entity: EndpointEntity,
    pub session: Arc<Session>,
    pub _phantom_data: PhantomData<(T, S)>,
}

impl<T, S> Builder for ZSubBuilder<T, S>
where
    T: ZMessage + Send + Sync + 'static,
    S: ZDeserializer + Send + Sync + 'static,
{
    type Output = ZSub<T, S>;

    fn build(self) -> Result<Self::Output> {
        let (tx, rx) = flume::bounded(10);
        let inner = self
            .session
            .declare_subscriber(self.entity.topic_key_expr()?)
            .callback(move |sample| {
                tx.send(sample).unwrap();
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

impl<T, S> ZSubBuilder<T, S>
where
    T: ZMessage,
    S: ZDeserializer,
{
    /// Creates a ZSub with a notification callback.
    /// This is primarily used by rcl-z for ROS 2 integration.
    #[cfg(feature = "rcl-z")]
    pub fn build_with_notifier<F>(self, notify: F) -> Result<ZSub<T, S>>
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

pub struct ZSub<T: ZMessage, S = <T as ZMessage>::Serdes>
where
    S: ZDeserializer,
{
    pub entity: EndpointEntity,
    pub queue: flume::Receiver<Sample>,
    _inner: zenoh::pubsub::Subscriber<()>,
    _lv_token: LivelinessToken,
    _phantom_data: PhantomData<(T, S)>,
}

impl<T, S> ZSub<T, S>
where
    T: ZMessage,
    S: ZDeserializer,
{
    /// Receives the raw Sample without deserialization
    pub fn recv_sample(&self) -> Result<Sample> {
        let sample = self.queue.recv()?;
        Ok(sample)
    }

    /// Receives the raw Sample asynchronously without deserialization
    pub async fn async_recv_sample(&self) -> Result<Sample> {
        let sample = self.queue.recv_async().await?;
        Ok(sample)
    }

    pub fn into_stream<'a>(self) -> flume::r#async::RecvStream<'a, Sample> {
        self.queue.into_stream()
    }
}

impl<T, S> ZSub<T, S>
where
    T: ZMessage,
    S: ZDeserializer,
    S::Output: Into<T>,
    for<'a> S::Input<'a>: From<&'a [u8]>,
{
    /// Receives a deserialized message
    pub fn recv(&self) -> Result<T> {
        let sample = self.queue.recv()?;
        let bytes = sample.payload().to_bytes();
        let input = <_>::from(&*bytes);
        let msg = S::deserialize(input).into();
        Ok(msg)
    }

    /// Receives a deserialized message asynchronously
    pub async fn async_recv(&self) -> Result<T> {
        let sample = self.queue.recv_async().await?;
        let bytes = sample.payload().to_bytes();
        let input = <_>::from(&*bytes);
        let msg = S::deserialize(input).into();
        Ok(msg)
    }
}
