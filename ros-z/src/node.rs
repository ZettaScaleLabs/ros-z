use crate::entity::EntityKind;
use crate::graph::Graph;
use std::sync::Arc;
use zenoh::liveliness::LivelinessToken;
use zenoh::{Result, Session, Wait};

use crate::{
    action::{client::ZActionClientBuilder, server::ZActionServerBuilder},
    Builder,
    context::{GlobalCounter, RemapRules},
    entity::*,
    msg::{ZMessage, ZService},
    pubsub::{ZPubBuilder, ZSubBuilder},
    service::{ZClientBuilder, ZServerBuilder},
    ServiceTypeInfo,
    WithTypeInfo,
};

pub struct ZNode {
    pub entity: NodeEntity,
    pub session: Arc<Session>,
    counter: Arc<GlobalCounter>,
    pub graph: Arc<Graph>,
    pub remap_rules: RemapRules,
    _lv_token: LivelinessToken,
}

pub struct ZNodeBuilder {
    pub domain_id: usize,
    pub name: String,
    pub namespace: String,
    pub session: Arc<Session>,
    pub counter: Arc<GlobalCounter>,
    pub graph: Arc<Graph>,
    pub remap_rules: RemapRules,
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
            remap_rules: self.remap_rules,
        })
    }
}

impl ZNode {
    /// Create a publisher for the given topic
    /// If T implements WithTypeInfo, type information will be automatically populated
    ///
    /// The topic name will be qualified according to ROS 2 rules:
    /// - Absolute topics (starting with '/') are used as-is
    /// - Private topics (starting with '~') are expanded to /<namespace>/<node_name>/<topic>
    /// - Relative topics are expanded to /<namespace>/<topic>
    pub fn create_pub<T>(&self, topic: &str) -> ZPubBuilder<T>
    where
        T: ZMessage + WithTypeInfo,
    {
        self.create_pub_impl(topic, Some(T::type_info()))
    }

    pub(crate) fn create_pub_impl<T>(
        &self,
        topic: &str,
        type_info: Option<crate::entity::TypeInfo>,
    ) -> ZPubBuilder<T>
    where
        T: ZMessage,
    {
        // Note: Topic qualification happens in ZPubBuilder::build()
        // to allow error handling in the Result type
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
            with_attachment: true,
            _phantom_data: Default::default(),
        }
    }

    /// Create a subscriber for the given topic
    /// If T implements WithTypeInfo, type information will be automatically populated
    ///
    /// The topic name will be qualified according to ROS 2 rules:
    /// - Absolute topics (starting with '/') are used as-is
    /// - Private topics (starting with '~') are expanded to /<namespace>/<node_name>/<topic>
    /// - Relative topics are expanded to /<namespace>/<topic>
    pub fn create_sub<T>(&self, topic: &str) -> ZSubBuilder<T>
    where
        T: WithTypeInfo,
    {
        self.create_sub_impl(topic, Some(T::type_info()))
    }

    pub(crate) fn create_sub_impl<T>(
        &self,
        topic: &str,
        type_info: Option<crate::entity::TypeInfo>,
    ) -> ZSubBuilder<T> {
        // Note: Topic qualification happens in ZSubBuilder::build()
        // to allow error handling in the Result type
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

    /// Create a service for the given service name
    /// If T is a tuple (Req, Resp) where both implement WithTypeInfo, type information will be automatically populated
    ///
    /// The service name will be qualified according to ROS 2 rules:
    /// - Absolute service names (starting with '/') are used as-is
    /// - Private service names (starting with '~') are expanded to /<namespace>/<node_name>/<service>
    /// - Relative service names are expanded to /<namespace>/<service>
    pub fn create_service<T>(&self, topic: &str) -> ZServerBuilder<T>
    where
        T: ZService + ServiceTypeInfo,
    {
        self.create_service_impl(topic, Some(T::service_type_info()))
    }

    pub(crate) fn create_service_impl<T>(
        &self,
        topic: &str,
        type_info: Option<crate::entity::TypeInfo>,
    ) -> ZServerBuilder<T> {
        // Note: Service name qualification happens in ZServerBuilder::build()
        // to allow error handling in the Result type
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

    /// Create a client for the given service name
    /// If T is a tuple (Req, Resp) where both implement WithTypeInfo, type information will be automatically populated
    ///
    /// The service name will be qualified according to ROS 2 rules:
    /// - Absolute service names (starting with '/') are used as-is
    /// - Private service names (starting with '~') are expanded to /<namespace>/<node_name>/<service>
    /// - Relative service names are expanded to /<namespace>/<service>
    pub fn create_client<T>(&self, topic: &str) -> ZClientBuilder<T>
    where
        T: ZService + ServiceTypeInfo,
    {
        self.create_client_impl(topic, Some(T::service_type_info()))
    }

    pub(crate) fn create_client_impl<T>(
        &self,
        topic: &str,
        type_info: Option<crate::entity::TypeInfo>,
    ) -> ZClientBuilder<T> {
        // Note: Service name qualification happens in ZClientBuilder::build()
        // to allow error handling in the Result type
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

    /// Create an action client for the given action name
    pub fn create_action_client<A>(&self, action_name: &str) -> ZActionClientBuilder<'_, A>
    where
        A: crate::action::ZAction,
    {
        ZActionClientBuilder::new(action_name, self)
    }

    /// Create an action server for the given action name
    pub fn create_action_server<A>(&self, action_name: &str) -> ZActionServerBuilder<'_, A>
    where
        A: crate::action::ZAction,
    {
        ZActionServerBuilder::new(action_name, self)
    }
}
