#![allow(unused)]

use std::{
    collections::HashMap,
    marker::PhantomData,
    sync::{Arc, atomic::AtomicUsize},
};

use serde::Deserialize;
use zenoh::{
    Result, Session, Wait, bytes,
    key_expr::KeyExpr,
    query::{Query, Reply},
    sample::Sample,
};

use std::sync::atomic::Ordering::AcqRel;

use crate::impl_with_type_info;

use crate::{
    Builder,
    attachment::{self, Attachment, GidArray},
    entity::EndpointEntity,
    msg::{CdrSerdes, ZMessage, ZService},
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
    // tx: flume::Sender<T::Response>,
    // rx: flume::Receiver<T::Response>,
    tx: flume::Sender<Sample>,
    pub rx: flume::Receiver<Sample>,
    _phantom_data: PhantomData<T>,
}

impl<T> Builder for ZClientBuilder<T>
where
    T: ZService,
{
    type Output = ZClient<T>;

    fn build(self) -> Result<Self::Output> {
        let key_expr = self.entity.topic_key_expr()?;
        tracing::error!("[CLN] KE: {key_expr}");

        let (tx, rx) = flume::unbounded();
        Ok(ZClient {
            sn: AtomicUsize::new(0),
            inner: self.session.declare_querier(key_expr).wait()?,
            gid: self.entity.gid(),
            tx,
            rx,
            _phantom_data: Default::default(),
        })
    }
}

impl<T> ZClient<T>
where
    T: ZService,
{
    fn new_attchment(&self) -> Attachment {
        Attachment::new(self.sn.fetch_add(1, AcqRel) as _, self.gid)
    }

    pub fn take_sample(&self) -> Result<Sample> {
        Ok(self.rx.recv()?)
    }

    pub fn take_response(&self) -> Result<T::Response>
    where
        for<'c> T::Response: ZMessage<Serdes = CdrSerdes<T::Response>> + Deserialize<'c>,
    {
        let sample = self.take_sample()?;
        let msg = <T::Response as ZMessage>::deserialize(&sample.payload().to_bytes());
        Ok(msg)
    }
}

impl<T> ZClient<T>
where
    T: ZService,
{
    pub fn send_request(&self, msg: &T::Request) -> Result<()> {
        let tx = self.tx.clone();
        self.inner
            .get()
            .payload(msg.serialize())
            .attachment(self.new_attchment())
            .callback(move |reply| {
                let sample = reply.into_result().unwrap();
                tx.send(sample);
                // let msg = <T::Response as ZMessage>::deserialize(&sample.payload().to_bytes());
                // tx.send(msg);
            })
            .wait()?;
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
                let sample = reply.into_result().unwrap();
                tx.send(sample);
                notify()
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

pub struct ZServer<T: ZService> {
    // NOTE: This is biased toward RMW
    key_expr: KeyExpr<'static>,
    // TODO: replace this with the sample sn
    sn: AtomicUsize,
    // TODO: replace this with zenoh's global entity id
    gid: GidArray,
    inner: zenoh::query::Queryable<()>,
    // rx: flume::Receiver<T::Request>,
    pub rx: flume::Receiver<Query>,
    pub map: HashMap<QueryKey, Query>,
    _phantom_data: PhantomData<T>,
}

impl<T> Builder for ZServerBuilder<T>
where
    T: ZService,
{
    type Output = ZServer<T>;

    fn build(self) -> Result<Self::Output> {
        let key_expr = self.entity.topic_key_expr()?;
        tracing::error!("[SRV] KE: {key_expr}");

        let (tx, rx) = flume::unbounded();
        let qable = self
            .session
            .declare_queryable(&key_expr)
            .callback(move |query| {
                let _ = tx.send(query);
            })
            .wait()?;
        Ok(ZServer {
            key_expr,
            sn: AtomicUsize::new(0),
            inner: qable,
            gid: self.entity.gid(),
            rx,
            map: HashMap::new(),
            _phantom_data: Default::default(),
        })
    }
}

impl<T> ZServerBuilder<T>
    where T: ZService
{
    #[cfg(feature = "rcl-z")]
    pub fn build_with_notifier<F>(self, notify: F) -> Result<ZServer<T>>
    where
        F: Fn() + Send + Sync + 'static,
    {
        let key_expr = self.entity.topic_key_expr()?;
        tracing::error!("[SRV] KE: {key_expr}");

        let (tx, rx) = flume::unbounded();
        let qable = self
            .session
            .declare_queryable(&key_expr)
            .callback(move |query| {
                let _ = tx.send(query);
                notify();
            })
            .wait()?;
        Ok(ZServer {
            key_expr,
            sn: AtomicUsize::new(0),
            inner: qable,
            gid: self.entity.gid(),
            rx,
            map: HashMap::new(),
            _phantom_data: Default::default(),
        })
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

impl<T> ZServer<T>
where
    T: ZService,
{
    fn new_attchment(&self) -> Attachment {
        Attachment::new(self.sn.fetch_add(1, AcqRel) as _, self.gid)
    }

    pub fn take_query(&self) -> Result<Query> {
        Ok(self.rx.recv()?)
    }

    pub fn take_request(&mut self) -> Result<(QueryKey, T::Request)>
    where
        for<'c> T::Request:
            ZMessage<Serdes = CdrSerdes<T::Request>> + Send + Sync + 'static + Deserialize<'c>,
    {
        let query = self.rx.recv()?;
        let attachment: Attachment = query.attachment().unwrap().try_into()?;
        let key: QueryKey = attachment.into();
        if self.map.contains_key(&key) {
            return Err("Existing query detected".into());
        }
        let msg = <T::Request as ZMessage>::deserialize(&query.payload().unwrap().to_bytes());
        self.map.insert(key.clone(), query);

        Ok((key, msg))
    }

    pub fn send_response(&mut self, msg: &T::Response, key: &QueryKey) -> Result<()> {
        match self.map.remove(key) {
            Some(query) => query
                .reply(&self.key_expr, msg.serialize())
                .attachment(self.new_attchment())
                .wait(),
            None => Err("Quey map doesn't contains {key}".into()),
        }
    }
}
