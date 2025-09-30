use crate::entity::EntityKind;
use crate::graph::Graph;
use std::sync::Arc;
use zenoh::liveliness::LivelinessToken;
use zenoh::{Result, Session, Wait};

use crate::{
    Builder,
    context::GlobalCounter,
    entity::*,
    msg::{ZDeserializer, ZMessage, ZSerializer},
    pubsub::{ZPubBuilder, ZSubBuilder},
    service::{ZClientBuilder, ZServerBuilder},
};

pub struct ZNode {
    pub entity: NodeEntity,
    session: Arc<Session>,
    counter: Arc<GlobalCounter>,
    pub graph: Arc<Graph>,
    _lv_token: LivelinessToken,
}

pub struct ZNodeBuilder {
    pub domain_id: usize,
    pub name: String,
    pub namespace: String,
    pub session: Arc<Session>,
    pub counter: Arc<GlobalCounter>,
    pub graph: Arc<Graph>,
}

impl ZNodeBuilder {
    pub fn with_namespace<S: AsRef<str>>(mut self, namespace: S) -> Self {
        self.namespace = namespace.as_ref().to_owned();
        self
    }
}

impl Builder for ZNodeBuilder {
    type Output = ZNode;
    fn build(self) -> Result<ZNode> {
        let id = self.counter.increment();
        let node = NodeEntity::new(
            self.domain_id,
            self.session.zid(),
            id,
            self.name,
            self.namespace,
        );
        let lv_token = self
            .session
            .liveliness()
            .declare_token(node.lv_token_key_expr()?)
            .wait()?;
        Ok(ZNode {
            entity: node,
            session: self.session,
            counter: self.counter,
            _lv_token: lv_token,
            graph: self.graph,
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
            kind: EntityKind::Publisher,
            type_info: None,
            ..Default::default()
        };
        ZPubBuilder {
            entity,
            session: self.session.clone(),
            _phantom_data: Default::default(),
        }
    }

    pub fn create_pub_with_serdes<T, S>(&self, topic: &str) -> ZPubBuilder<T, S>
    where
        T: ZMessage,
        S: for<'a> ZSerializer<Input<'a> = &'a T>,
    {
        let entity = EndpointEntity {
            id: self.counter.increment(),
            node: self.entity.clone(),
            topic: topic.to_string(),
            kind: EntityKind::Publisher,
            type_info: None,
            ..Default::default()
        };
        ZPubBuilder {
            entity,
            session: self.session.clone(),
            _phantom_data: Default::default(),
        }
    }

    pub fn create_sub<T>(&self, topic: &str) -> ZSubBuilder<T>
    where
        T: ZMessage,
    {
        let entity = EndpointEntity {
            id: self.counter.increment(),
            node: self.entity.clone(),
            topic: topic.to_string(),
            kind: EntityKind::Subscription,
            ..Default::default()
        };
        ZSubBuilder {
            entity,
            session: self.session.clone(),
            _phantom_data: Default::default(),
        }
    }

    pub fn create_sub_with_serdes<T, S>(&self, topic: &str) -> ZSubBuilder<T, S>
    where
        T: ZMessage,
        S: ZDeserializer,
    {
        let entity = EndpointEntity {
            id: self.counter.increment(),
            node: self.entity.clone(),
            topic: topic.to_string(),
            kind: EntityKind::Subscription,
            ..Default::default()
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
            kind: EntityKind::Service,
            ..Default::default()
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
            kind: EntityKind::Client,
            ..Default::default()
        };
        ZClientBuilder {
            entity,
            session: self.session.clone(),
            _phantom_data: Default::default(),
        }
    }
}
