use crate::entity::EntityKind;
use crate::graph::Graph;
use std::sync::Arc;
use tracing::debug;
use zenoh::liveliness::LivelinessToken;
use zenoh::{Result, Session, Wait};

use crate::{
    Builder, ServiceTypeInfo, WithTypeInfo,
    action::{client::ZActionClientBuilder, server::ZActionServerBuilder},
    context::{GlobalCounter, RemapRules},
    entity::*,
    msg::{ZMessage, ZService},
    pubsub::{ZPubBuilder, ZSubBuilder},
    service::{ZClientBuilder, ZServerBuilder},
};

pub struct ZNode {
    pub entity: NodeEntity,
    pub session: Arc<Session>,
    counter: Arc<GlobalCounter>,
    pub graph: Arc<Graph>,
    pub remap_rules: RemapRules,
    _lv_token: LivelinessToken,
    pub(crate) shm_config: Option<Arc<crate::shm::ShmConfig>>,
}

pub struct ZNodeBuilder {
    pub domain_id: usize,
    pub name: String,
    pub namespace: String,
    pub enclave: String,
    pub session: Arc<Session>,
    pub counter: Arc<GlobalCounter>,
    pub graph: Arc<Graph>,
    pub remap_rules: RemapRules,
    pub(crate) shm_config: Option<Arc<crate::shm::ShmConfig>>,
}

impl ZNodeBuilder {
    pub fn with_namespace<S: AsRef<str>>(mut self, namespace: S) -> Self {
        // Normalize namespace: "/" (root namespace) should be treated as "" (empty)
        // This ensures consistent HashMap lookups across local and remote entities
        let ns = namespace.as_ref();
        self.namespace = if ns == "/" {
            String::new()
        } else {
            ns.to_owned()
        };
        self
    }

    /// Override SHM configuration for this node (and its publishers).
    ///
    /// This overrides the context-level SHM configuration for all publishers
    /// created from this node.
    ///
    /// # Example
    /// ```no_run
    /// use ros_z::shm::{ShmConfig, ShmProviderBuilder};
    /// use ros_z::Builder;
    /// use std::sync::Arc;
    ///
    /// # fn main() -> zenoh::Result<()> {
    /// # let ctx = ros_z::context::ZContextBuilder::default().build()?;
    /// let provider = Arc::new(ShmProviderBuilder::new(20 * 1024 * 1024).build()?);
    /// let config = ShmConfig::new(provider).with_threshold(5_000);
    ///
    /// let node = ctx.create_node("my_node")
    ///     .with_shm_config(config)
    ///     .build()?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn with_shm_config(mut self, config: crate::shm::ShmConfig) -> Self {
        self.shm_config = Some(Arc::new(config));
        self
    }
}

impl Builder for ZNodeBuilder {
    type Output = ZNode;

    #[tracing::instrument(name = "node_build", skip(self), fields(
        name = %self.name,
        namespace = %self.namespace,
        id = tracing::field::Empty
    ))]
    fn build(self) -> Result<ZNode> {
        let id = self.counter.increment();
        tracing::Span::current().record("id", id);

        debug!(
            "[NOD] Creating node: {}/{}, id={}",
            self.namespace, self.name, id
        );

        let node = NodeEntity::new(
            self.domain_id,
            self.session.zid(),
            id,
            self.name.clone(),
            self.namespace.clone(),
            self.enclave,
        );
        let lv_token_ke = node.lv_token_key_expr()?;
        debug!("[NOD] Liveliness token KE: {}", lv_token_ke);

        let lv_token = self
            .session
            .liveliness()
            .declare_token(lv_token_ke)
            .wait()?;
        debug!("[NOD] Node ready: {}/{}", self.namespace, self.name);

        Ok(ZNode {
            entity: node,
            session: self.session,
            counter: self.counter,
            _lv_token: lv_token,
            graph: self.graph,
            remap_rules: self.remap_rules,
            shm_config: self.shm_config,
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
        debug!("[NOD] Creating publisher: topic={}", topic);
        self.create_pub_impl(topic, Some(T::type_info()))
    }

    pub fn create_pub_impl<T>(
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
            shm_config: self.shm_config.clone(),
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
        debug!("[NOD] Creating subscriber: topic={}", topic);
        self.create_sub_impl(topic, Some(T::type_info()))
    }

    pub fn create_sub_impl<T>(
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
            dyn_schema: None,
            locality: None,
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
        debug!("[NOD] Creating service: topic={}", topic);
        self.create_service_impl(topic, Some(T::service_type_info()))
    }

    pub fn create_service_impl<T>(
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
        debug!("[NOD] Creating client: topic={}", topic);
        self.create_client_impl(topic, Some(T::service_type_info()))
    }

    pub fn create_client_impl<T>(
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
