use std::sync::Arc;
use zenoh::Session;

use crate::msg::ZMessage;
use crate::pubsub::{ZPubBuilder, ZSubBuilder};

pub struct ZNode {
    pub session: Arc<Session>,
}

impl ZNode {
    pub fn new(session: Arc<Session>) -> Self {
        Self { session }
    }

    pub fn create_pub<T>(&self, key_expr: &'static str) -> ZPubBuilder<T>
    where
        T: ZMessage,
    {
        ZPubBuilder {
            session: self.session.clone(),
            key_expr: key_expr.try_into().unwrap(),
            _phantom_data: Default::default(),
        }
    }

    pub fn create_sub<T>(&self, key_expr: &'static str) -> ZSubBuilder<T>
    {
        ZSubBuilder {
            session: self.session.clone(),
            key_expr: key_expr.try_into().unwrap(),
            _phantom_data: Default::default(),
        }
    }
}
