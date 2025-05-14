use crate::msg::{CdrSerdes, ZMessage};
use serde::Deserialize;
use std::{marker::PhantomData, sync::Arc};
use zenoh::{Result, Session, Wait, key_expr::KeyExpr, sample::Sample};

pub trait Builder {
    type Output;
    fn build(self) -> Result<Self::Output>;
}

pub struct ZPub<T: ZMessage> {
    inner: zenoh::pubsub::Publisher<'static>,
    _phantom_data: PhantomData<T>,
}

pub struct ZPubBuilder<T> {
    pub session: Arc<Session>,
    pub key_expr: KeyExpr<'static>,
    pub _phantom_data: PhantomData<T>,
}

impl<T> Builder for ZPubBuilder<T>
where
    T: ZMessage,
{
    type Output = ZPub<T>;

    fn build(self) -> Result<Self::Output> {
        let zpub = self.session.declare_publisher(self.key_expr).wait()?;
        Ok(ZPub {
            inner: zpub,
            _phantom_data: Default::default(),
        })
    }
}

impl<T> ZPub<T>
where
    T: ZMessage,
{
    pub fn publish(&self, msg: &T) -> Result<()> {
        self.inner.put(msg.serialize()).wait()
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

pub struct ZSubBuilder<'ke, T, const POST_DESERIALIZATION: bool = false> {
    pub session: Arc<Session>,
    pub key_expr: KeyExpr<'ke>,
    pub _phantom_data: PhantomData<T>,
}

impl<'b, T> ZSubBuilder<'b, T, false>
where
    T: ZMessage
{
    pub fn post_deserialization(self) -> ZSubBuilder<'b, T, true> {
        ZSubBuilder {
            session: self.session,
            key_expr: self.key_expr,
            _phantom_data: self._phantom_data,
        }
    }
}

impl<T> Builder for ZSubBuilder<'_, T, false>
where
    for<'c> T: ZMessage<Serdes = CdrSerdes<T>> + Deserialize<'c> + Send + Sync + 'static,
{
    type Output = ZSub<T, T>;

    fn build(self) -> Result<Self::Output> {
        let (tx, rx) = flume::bounded(10);
        let inner = self
            .session
            .declare_subscriber(&self.key_expr)
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

impl<T> ZSubBuilder<'_, T, true>
where
    T: ZMessage
{
    pub fn build_with_notifier<F>(self, notify: F) -> Result<ZSub<T, Sample>>
    where F: Fn() + Send + Sync + 'static
    {
        let (tx, rx) = flume::bounded(10);
        let inner = self
            .session
            .declare_subscriber(&self.key_expr)
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

impl<T> Builder for ZSubBuilder<'_, T, true>
where
    T: ZMessage + 'static + Sync + Send,
{
    type Output = ZSub<T, Sample>;

    fn build(self) -> Result<Self::Output> {
        let (tx, rx) = flume::bounded(10);
        let inner = self
            .session
            .declare_subscriber(self.key_expr)
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
