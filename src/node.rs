use std::sync::Arc;
use zenoh::{Result, Session};
use serde::Deserialize;

use crate::msg::{ZMessage as Message, CdrSerder};
use crate::pubsub::{ZPub, ZSub, ZSubPreDes};

pub struct ZNode {
    pub session: Arc<Session>,
}

impl ZNode {
    pub fn new(session: Arc<Session>) -> Self {
        Self { session }
    }

    pub fn create_pub<'a, T>(&'a self, keyexpr: &'a str) -> Result<ZPub<'a, T>>
    where
        T: Message,
    {
        ZPub::new(&self.session, keyexpr)
    }

    pub fn create_sub_predes<T>(&self, keyexpr: &str) -> Result<ZSubPreDes<T>>
    where
        for<'a> T: Message<Serder = CdrSerder<T>> + Deserialize<'a> + Send + Sync + 'static,
    {
        ZSubPreDes::new(&self.session, keyexpr)
    }

    pub fn create_sub<T>(&self, keyexpr: &str) -> Result<ZSub<T>>
    where
        T: Message + Sync + Send + 'static,
    {
        ZSub::new(&self.session, keyexpr)
    }
}
