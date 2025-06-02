use std::sync::Arc;
use zenoh::{Result, Session};

use crate::{
    Builder,
    context::GlobalCounter,
    entity::*,
    msg::ZMessage,
    pubsub::{ZPubBuilder, ZSubBuilder},
    service::{ZClientBuilder, ZServerBuilder},
};

pub struct ZNode {
    pub entity: NodeEntity,
    session: Arc<Session>,
    counter: Arc<GlobalCounter>,
}

pub struct ZNodeBuilder {
    pub name: String,
    pub namespace: Option<String>,
    pub session: Arc<Session>,
    pub counter: Arc<GlobalCounter>,
}

impl ZNodeBuilder {
    pub fn with_namespace<S: AsRef<str>>(mut self, namespace: S) -> Self {
        self.namespace = Some(namespace.as_ref().to_owned());
        self
    }
}

impl Builder for ZNodeBuilder {
    type Output = ZNode;
    fn build(self) -> Result<ZNode> {
        let id = self.counter.increment();
        let node = NodeEntity::new(
            0,
            self.session.zid(),
            id,
            self.name,
            self.namespace.to_owned(),
        );
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
            type_info: None,
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
            type_info: None,
        };
        ZSubBuilder {
            entity,
            session: self.session.clone(),
            _phantom_data: Default::default(),
        }
    }

    pub fn create_service<T>(&self, topic: &str) -> ZServerBuilder<T> {
        let entity = EndpointEntity {
            id: self.counter.increment(),
            node: self.entity.clone(),
            topic: topic.to_string(),
            kind: crate::entity::EndpointKind::Service,
            type_info: None,
        };
        ZServerBuilder {
            entity,
            session: self.session.clone(),
            _phantom_data: Default::default(),
        }
    }

    pub fn create_client<T>(&self, topic: &str) -> ZClientBuilder<T> {
        let entity = EndpointEntity {
            id: self.counter.increment(),
            node: self.entity.clone(),
            topic: topic.to_string(),
            kind: crate::entity::EndpointKind::Client,
            type_info: None,
        };
        ZClientBuilder {
            entity,
            session: self.session.clone(),
            _phantom_data: Default::default(),
        }
    }
}
