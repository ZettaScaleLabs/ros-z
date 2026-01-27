use crate::entity::EntityKind;
use crate::graph::Graph;
use std::sync::Arc;
use std::time::Duration;
use zenoh::liveliness::LivelinessToken;
use zenoh::sample::Sample;
use zenoh::{Result, Session, Wait};
use tracing::{info, debug, warn};

use crate::{
    Builder, ServiceTypeInfo, WithTypeInfo,
    action::{client::ZActionClientBuilder, server::ZActionServerBuilder},
    context::{GlobalCounter, RemapRules},
    dynamic::{
        DynamicCdrSerdes, DynamicMessage, MessageSchema, TypeDescriptionClient,
        TypeDescriptionService,
    },
    entity::*,
    msg::{ZMessage, ZService},
    pubsub::{ZPub, ZPubBuilder, ZSub, ZSubBuilder},
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
    /// Optional type description service for this node.
    /// Enabled via `ZNodeBuilder::with_type_description_service()`.
    /// The service uses callback mode and requires no background task.
    type_desc_service: Option<TypeDescriptionService>,
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
    /// Whether to enable the type description service for this node.
    pub enable_type_desc_service: bool,
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

    /// Enable the type description service for this node.
    ///
    /// When enabled, the node will expose a `~get_type_description` service
    /// that allows other nodes to query type descriptions for schemas
    /// registered with this node's publishers.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let node = ctx
    ///     .create_node("my_node")
    ///     .with_type_description_service()
    ///     .build()?;
    ///
    /// // Now publishers created from this node can auto-register their schemas
    /// ```
    pub fn with_type_description_service(mut self) -> Self {
        self.enable_type_desc_service = true;
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

        // Create type description service if enabled
        let type_desc_service = if self.enable_type_desc_service {
            debug!("[NOD] Creating type description service");
            let service = TypeDescriptionService::new(
                self.session.clone(),
                &self.name,
                &self.namespace,
                id,
                &self.counter,
            )?;

            info!("[NOD] TypeDescriptionService created (callback mode)");

            Some(service)
        } else {
            None
        };

        debug!("[NOD] Node ready: {}/{}", self.namespace, self.name);

        Ok(ZNode {
            entity: node,
            session: self.session,
            counter: self.counter,
            _lv_token: lv_token,
            graph: self.graph,
            remap_rules: self.remap_rules,
            shm_config: self.shm_config,
            type_desc_service,
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
            dyn_schema: None,
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

    /// Get a reference to this node's type description service, if enabled.
    ///
    /// Returns `None` if the node was not created with `.with_type_description_service()`.
    pub fn type_description_service(&self) -> Option<&TypeDescriptionService> {
        self.type_desc_service.as_ref()
    }

    /// Get a mutable reference to this node's type description service, if enabled.
    ///
    /// Returns `None` if the node was not created with `.with_type_description_service()`.
    pub fn type_description_service_mut(&mut self) -> Option<&mut TypeDescriptionService> {
        self.type_desc_service.as_mut()
    }

    /// Check if this node has a type description service.
    pub fn has_type_description_service(&self) -> bool {
        self.type_desc_service.is_some()
    }

    /// Get access to the global counter for entity ID generation.
    pub fn counter(&self) -> &Arc<GlobalCounter> {
        &self.counter
    }

    // ========================================================================
    // Dynamic Message API
    // ========================================================================

    /// Create a dynamic publisher for the given topic.
    ///
    /// If this node has a type description service enabled, the schema will be
    /// automatically registered, allowing other nodes to discover it via the
    /// `GetTypeDescription` service.
    ///
    /// # Arguments
    ///
    /// * `topic` - The topic name to publish on
    /// * `schema` - The message schema for serialization
    ///
    /// # Example
    ///
    /// ```ignore
    /// let schema = MessageSchema::builder("std_msgs/msg/String")
    ///     .field("data", FieldType::String)
    ///     .build()?;
    ///
    /// let publisher = node.create_dyn_pub("chatter", schema)?;
    ///
    /// let mut msg = DynamicMessage::new(publisher.schema());
    /// msg.set("data", "Hello, world!")?;
    /// publisher.publish(&msg)?;
    /// ```
    pub fn create_dyn_pub(
        &self,
        topic: &str,
        schema: Arc<MessageSchema>,
    ) -> Result<ZPub<DynamicMessage, DynamicCdrSerdes>> {
        // Register schema with type description service if enabled
        if let Some(service) = &self.type_desc_service {
            if let Err(e) = service.register_schema(schema.clone()) {
                warn!(
                    "[NOD] Failed to register schema {} with type description service: {}",
                    schema.type_name, e
                );
            } else {
                debug!(
                    "[NOD] Registered schema {} with type description service",
                    schema.type_name
                );
            }
        }

        // Create TypeInfo from schema for proper key expression matching
        // Convert ROS 2 canonical name to DDS name
        // "std_msgs/msg/String" → "std_msgs::msg::dds_::String_"
        let dds_name = schema.type_name
            .replace("/msg/", "::msg::dds_::")
            .replace("/srv/", "::srv::dds_::")
            .replace("/action/", "::action::dds_::")
            + "_";

        // Compute type hash and convert to entity::TypeHash format
        use crate::dynamic::MessageSchemaTypeDescription;
        let type_hash = match schema.compute_type_hash() {
            Ok(hash) => {
                let rihs_string = hash.to_rihs_string();
                crate::entity::TypeHash::from_rihs_string(&rihs_string)
                    .unwrap_or_else(crate::entity::TypeHash::zero)
            }
            Err(e) => {
                warn!("[NOD] Failed to compute type hash for {}: {}", schema.type_name, e);
                crate::entity::TypeHash::zero()
            }
        };

        let type_info = Some(crate::entity::TypeInfo {
            name: dds_name,
            hash: type_hash,
        });

        // Build the publisher
        self.create_pub_impl::<DynamicMessage>(topic, type_info)
            .with_serdes::<DynamicCdrSerdes>()
            .with_dyn_schema(schema)
            .build()
    }

    /// Create a dynamic subscriber with automatic schema discovery.
    ///
    /// This method queries publishers on the topic for their type description
    /// and creates a subscriber using the discovered schema. This is useful
    /// when you don't know the message type at compile time.
    ///
    /// # Arguments
    ///
    /// * `topic` - The topic name to subscribe to
    /// * `discovery_timeout` - How long to wait for schema discovery
    ///
    /// # Returns
    ///
    /// A tuple of (subscriber, schema) on success. The schema is returned
    /// so you can use it to create messages or inspect the type.
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Discover schema from publishers and create subscriber
    /// let (subscriber, schema) = node.create_dyn_sub_auto(
    ///     "chatter",
    ///     Duration::from_secs(5),
    /// ).await?;
    ///
    /// println!("Discovered type: {}", schema.type_name);
    ///
    /// // Receive messages
    /// let msg = subscriber.recv()?;
    /// let data: String = msg.get("data")?;
    /// ```
    pub async fn create_dyn_sub_auto(
        &self,
        topic: &str,
        discovery_timeout: Duration,
    ) -> Result<(
        ZSub<DynamicMessage, Sample, DynamicCdrSerdes>,
        Arc<MessageSchema>,
    )> {
        debug!(
            "[NOD] Creating dynamic subscriber with auto-discovery for topic: {}",
            topic
        );

        // Create a TypeDescriptionClient to discover the schema
        let client =
            TypeDescriptionClient::with_graph(self.session.clone(), self.counter.clone(), self.graph.clone());

        // Discover schema from topic publishers
        let (schema, type_hash) = client
            .get_type_description_for_topic(topic, discovery_timeout)
            .await
            .map_err(|e| zenoh::Error::from(format!("Schema discovery failed: {}", e)))?;

        info!(
            "[NOD] Discovered schema for topic {}: {} (hash: {})",
            topic, schema.type_name, type_hash
        );

        // Create TypeInfo from discovered schema for proper key expression matching
        // Convert ROS 2 canonical name to DDS name
        // "std_msgs/msg/String" → "std_msgs::msg::dds_::String_"
        let dds_name = schema.type_name
            .replace("/msg/", "::msg::dds_::")
            .replace("/srv/", "::srv::dds_::")
            .replace("/action/", "::action::dds_::")
            + "_";

        let type_info = Some(crate::entity::TypeInfo {
            name: dds_name,
            hash: crate::entity::TypeHash::from_rihs_string(&type_hash)
                .unwrap_or_else(crate::entity::TypeHash::zero),
        });

        // Build the subscriber with the discovered schema
        let subscriber = self
            .create_sub_impl::<DynamicMessage>(topic, type_info)
            .with_serdes::<DynamicCdrSerdes>()
            .with_dyn_schema(schema.clone())
            .build()?;

        Ok((subscriber, schema))
    }

    /// Create a dynamic subscriber with a known schema.
    ///
    /// Use this when you already have the schema (e.g., loaded from a file
    /// or built programmatically).
    ///
    /// # Arguments
    ///
    /// * `topic` - The topic name to subscribe to
    /// * `schema` - The message schema for deserialization
    ///
    /// # Example
    ///
    /// ```ignore
    /// let schema = MessageSchema::builder("std_msgs/msg/String")
    ///     .field("data", FieldType::String)
    ///     .build()?;
    ///
    /// let subscriber = node.create_dyn_sub("chatter", schema)?;
    /// let msg = subscriber.recv()?;
    /// ```
    pub fn create_dyn_sub(
        &self,
        topic: &str,
        schema: Arc<MessageSchema>,
    ) -> Result<ZSub<DynamicMessage, Sample, DynamicCdrSerdes>> {
        // Create TypeInfo from schema for proper key expression matching
        // Convert ROS 2 canonical name to DDS name
        // "std_msgs/msg/String" → "std_msgs::msg::dds_::String_"
        let dds_name = schema.type_name
            .replace("/msg/", "::msg::dds_::")
            .replace("/srv/", "::srv::dds_::")
            .replace("/action/", "::action::dds_::")
            + "_";

        // Compute type hash and convert to entity::TypeHash format
        use crate::dynamic::MessageSchemaTypeDescription;
        let type_hash = match schema.compute_type_hash() {
            Ok(hash) => {
                let rihs_string = hash.to_rihs_string();
                crate::entity::TypeHash::from_rihs_string(&rihs_string)
                    .unwrap_or_else(crate::entity::TypeHash::zero)
            }
            Err(e) => {
                warn!("[NOD] Failed to compute type hash for {}: {}", schema.type_name, e);
                crate::entity::TypeHash::zero()
            }
        };

        let type_info = Some(crate::entity::TypeInfo {
            name: dds_name,
            hash: type_hash,
        });

        // Build the subscriber with proper type info
        self.create_sub_impl::<DynamicMessage>(topic, type_info)
            .with_serdes::<DynamicCdrSerdes>()
            .with_dyn_schema(schema)
            .build()
    }
}
