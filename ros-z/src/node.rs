use crate::entity::EntityKind;
use crate::graph::Graph;
use std::sync::Arc;
use zenoh::liveliness::LivelinessToken;
use zenoh::{Result, Session, Wait};

use crate::{
    Builder,
    context::GlobalCounter,
    entity::*,
    msg::{ZMessage, ZService},
    pubsub::{ZPubBuilder, ZSubBuilder},
    ros_msg::{self, WithTypeInfo},
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
    /// Create a publisher for the given topic
    /// If T implements WithTypeInfo, type information will be automatically populated
    pub fn create_pub<T>(&self, topic: &str) -> ZPubBuilder<T>
    where
        T: ZMessage + WithTypeInfo,
    {
        self.create_pub_impl(topic, Some(T::type_info()))
    }

    fn create_pub_impl<T>(
        &self,
        topic: &str,
        type_info: Option<crate::entity::TypeInfo>,
    ) -> ZPubBuilder<T>
    where
        T: ZMessage,
    {
        let entity = EndpointEntity {
            id: self.counter.increment(),
            node: self.entity.clone(),
            topic: topic.to_string(),
            kind: EntityKind::Publisher,
            type_info,
            ..Default::default()
        };
        ZPubBuilder {
            entity,
            session: self.session.clone(),
            _phantom_data: Default::default(),
        }
    }

    /// Create a subscriber for the given topic
    /// If T implements WithTypeInfo, type information will be automatically populated
    pub fn create_sub<T>(&self, topic: &str) -> ZSubBuilder<T>
    where
        T: WithTypeInfo,
    {
        self.create_sub_impl(topic, Some(T::type_info()))
    }

    fn create_sub_impl<T>(
        &self,
        topic: &str,
        type_info: Option<crate::entity::TypeInfo>,
    ) -> ZSubBuilder<T> {
        let entity = EndpointEntity {
            id: self.counter.increment(),
            node: self.entity.clone(),
            topic: topic.to_string(),
            kind: EntityKind::Subscription,
            type_info,
            ..Default::default()
        };
        ZSubBuilder {
            entity,
            session: self.session.clone(),
            _phantom_data: Default::default(),
        }
    }

    /// Create a service for the given topic
    /// If T is a tuple (Req, Resp) where both implement WithTypeInfo, type information will be automatically populated
    pub fn create_service<T>(&self, topic: &str) -> ZServerBuilder<T>
    where
        T: ZService + ros_msg::ServiceTypeInfo,
    {
        self.create_service_impl(topic, Some(T::service_type_info()))
    }

    fn create_service_impl<T>(
        &self,
        topic: &str,
        type_info: Option<crate::entity::TypeInfo>,
    ) -> ZServerBuilder<T> {
        let entity = EndpointEntity {
            id: self.counter.increment(),
            node: self.entity.clone(),
            topic: topic.to_string(),
            kind: EntityKind::Service,
            type_info,
            ..Default::default()
        };
        ZServerBuilder {
            entity,
            session: self.session.clone(),
            _phantom_data: Default::default(),
        }
    }

    /// Create a client for the given topic
    /// If T is a tuple (Req, Resp) where both implement WithTypeInfo, type information will be automatically populated
    pub fn create_client<T>(&self, topic: &str) -> ZClientBuilder<T>
    where
        T: ZService + ros_msg::ServiceTypeInfo,
    {
        self.create_client_impl(topic, Some(T::service_type_info()))
    }

    fn create_client_impl<T>(
        &self,
        topic: &str,
        type_info: Option<crate::entity::TypeInfo>,
    ) -> ZClientBuilder<T> {
        let entity = EndpointEntity {
            id: self.counter.increment(),
            node: self.entity.clone(),
            topic: topic.to_string(),
            kind: EntityKind::Client,
            type_info,
            ..Default::default()
        };
        ZClientBuilder {
            entity,
            session: self.session.clone(),
            _phantom_data: Default::default(),
        }
    }
}
