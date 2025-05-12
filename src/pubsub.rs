use crate::msg::{CdrDeserializer, CdrSerder, ZMessage as Message};
use serde::{Deserialize, Serialize};
use std::marker::PhantomData;
use zenoh::{Result, Session, Wait, sample::Sample};

pub struct ZPub<'a, T: Message> {
    inner: zenoh::pubsub::Publisher<'a>,
    _phantom_data: PhantomData<T>,
}

impl<'a, T> ZPub<'a, T>
where
    T: Message,
{
    pub fn new(session: &Session, keyexpr: &'a str) -> Result<Self> {
        let zpub = session.declare_publisher(keyexpr).wait()?;
        Ok(ZPub {
            inner: zpub,
            _phantom_data: PhantomData::default(),
        })
    }

    pub fn publish(&self, msg: &T) -> Result<()> {
        self.inner.put(msg.serialize()).wait()
    }

    pub async fn async_publish(&self, msg: &T) -> Result<()> {
        self.inner.put(msg.serialize()).await
    }

    pub fn publish_serialized_message(&self, msg: &[u8]) -> Result<()> {
        self.inner.put(msg).wait()
    }
}

pub struct ZSub<T: Message> {
    inner: zenoh::pubsub::Subscriber<()>,
    queue: flume::Receiver<Sample>,
    _phantom_data: PhantomData<T>,
}

// impl<T> ZSub<T>
// where
//     T: Message + 'static + Sync + Send,
// {
//     pub fn new(session: &Session, keyexpr: &str, x: ()) -> Result<Self> {
//         let (tx, rx) = flume::bounded(10);
//         let inner = session
//             .declare_subscriber(keyexpr)
//             .callback(move |sample| {
//                 let _ = tx.send(sample);
//             })
//             .wait()?;
//         Ok(ZSub {
//             inner,
//             queue: rx,
//             _phantom_data: PhantomData::default(),
//         })
//     }
// }

impl<T> ZSub<T>
where
    // T: Message<Serder = CdrSerder<T>> + 'static + Sync + Send,
    T: Message + 'static + Sync + Send,
{
    pub fn new(session: &Session, keyexpr: &str) -> Result<Self> {
        let (tx, rx) = flume::bounded(10);
        let inner = session
            .declare_subscriber(keyexpr)
            .callback(move |sample| {
                // let msg = <T as Message>::deserialize(&sample.payload().to_bytes().to_vec());
                // let _ = tx.send(msg);
                let _ = tx.send(sample);
            })
            .wait()?;
        Ok(ZSub {
            inner,
            queue: rx,
            _phantom_data: PhantomData::default(),
        })
    }

    pub fn recv(&self) -> Result<Sample> {
        let msg = self.queue.recv()?;
        Ok(msg)
    }
}

pub struct ZSubPreDes<T: Message> {
    inner: zenoh::pubsub::Subscriber<()>,
    queue: flume::Receiver<T>,
    _phantom_data: PhantomData<T>,
}

impl<T> ZSubPreDes<T>
where
    for<'a> T: Message<Serder = CdrSerder<T>> + Deserialize<'a> + Send + Sync + 'static,
{
    pub fn new(session: &Session, keyexpr: &str) -> Result<Self> {
        let (tx, rx) = flume::bounded(10);
        let inner = session
            .declare_subscriber(keyexpr)
            .callback(move |sample| {
                let msg = <T as Message>::deserialize(&sample.payload().to_bytes().to_vec());
                let _ = tx.send(msg);
            })
            .wait()?;
        Ok(ZSubPreDes {
            inner,
            queue: rx,
            _phantom_data: PhantomData::default(),
        })
    }

    pub fn recv(&self) -> Result<T> {
        let msg = self.queue.recv()?;
        Ok(msg)
    }
}
