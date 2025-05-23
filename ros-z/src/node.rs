use std::sync::Arc;
use zenoh::{Result, Session};

use crate::{
    Builder,
    context::GlobalCounter,
    msg::ZMessage,
    pubsub::{ZPubBuilder, ZSubBuilder},
    entity::*,
};

pub struct ZNode {
    entity: NodeEntity,
    session: Arc<Session>,
    counter: Arc<GlobalCounter>,
}

pub struct ZNodeBuilder<'a> {
    pub name: &'a str,
    pub session: Arc<Session>,
    pub counter: Arc<GlobalCounter>,
}

impl Builder for ZNodeBuilder<'_> {
    type Output = ZNode;
    fn build(self) -> Result<ZNode> {
        let id = self.counter.increment();
        let node = NodeEntity::new(0, self.session.zid(), id, self.name.to_owned(), "%".to_string());
        Ok(ZNode {
            entity: node,
            session: self.session,
            counter: self.counter,
        })
    }
}

impl ZNode {
    pub fn create_pub<T>(&self, topic: &str) -> ZPubBuilder<T>
    where
        T: ZMessage,
    {
        let entity = EndpointEntity {
            id: self.counter.increment(),
            node: self.entity.clone(),
            topic: topic.to_string(),
            kind: crate::entity::EndpointKind::Publisher,
            type_info: None
        };
        ZPubBuilder {
            entity,
            session: self.session.clone(),
            _phantom_data: Default::default(),
        }
    }

    pub fn create_sub<T>(&self, topic: &str) -> ZSubBuilder<T> {
        let entity = EndpointEntity {
            id: self.counter.increment(),
            node: self.entity.clone(),
            topic: topic.to_string(),
            kind: crate::entity::EndpointKind::Subscription,
            type_info: None
        };
        ZSubBuilder {
            entity,
            session: self.session.clone(),
            _phantom_data: Default::default(),
        }
    }
}
