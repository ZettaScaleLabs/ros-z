//! Entity types for ROS 2 entities in ros-z.
//!
//! This module re-exports entity types from ros-z-protocol and adds
//! ros-z-specific extensions.

// Re-export all entity types from ros-z-protocol
pub use ros_z_protocol::entity::*;

use zenoh::{Result, key_expr::KeyExpr};

// Constants for ros-z-specific functionality
pub const ADMIN_SPACE: &str = "@ros2_lv";

// Type aliases
type NodeName = String;
type NodeNamespace = String;
pub type NodeKey = (NodeNamespace, NodeName);
pub type Topic = String;

// Extension functions for NodeEntity (can't use impl due to orphan rules)

/// Get the key for this node (namespace, name)
pub fn node_key(entity: &NodeEntity) -> NodeKey {
    // Normalize namespace: "/" (root namespace) should be treated as "" (empty)
    // This ensures consistent HashMap lookups across local and remote entities
    let normalized_namespace = if entity.namespace == "/" {
        String::new()
    } else {
        entity.namespace.clone()
    };
    (normalized_namespace, entity.name.clone())
}

/// Get the liveliness token key expression for a node
pub fn node_lv_token_key_expr(entity: &NodeEntity) -> Result<KeyExpr<'static>> {
    let ke = node_to_liveliness_ke(entity)?;
    Ok(ke.0)
}

// Extension functions for EndpointEntity

/// Get the GID (globally unique identifier) for this endpoint
pub fn endpoint_gid(entity: &EndpointEntity) -> crate::attachment::GidArray {
    use sha2::Digest;
    let mut hasher = sha2::Sha256::new();
    // ZenohId has to_le_bytes() method
    hasher.update(entity.node.z_id.to_le_bytes());
    hasher.update(&entity.id.to_le_bytes());
    let hash = hasher.finalize();
    let mut gid = [0u8; 16];
    gid.copy_from_slice(&hash[..16]);
    gid
}

// Helper functions for converting entities to LivelinessKE
// Note: Can't implement TryFrom due to orphan rules (both types are from ros-z-protocol)

/// Convert a NodeEntity to a LivelinessKE using the default format
pub fn node_to_liveliness_ke(entity: &NodeEntity) -> Result<LivelinessKE> {
    let format = ros_z_protocol::KeyExprFormat::default();
    format.node_liveliness_key_expr(entity)
}

/// Convert an EndpointEntity to a LivelinessKE using the default format
pub fn endpoint_to_liveliness_ke(entity: &EndpointEntity) -> Result<LivelinessKE> {
    let format = ros_z_protocol::KeyExprFormat::default();
    format.liveliness_key_expr(entity, &entity.node.z_id)
}

<<<<<<< HEAD
        // Mangle all slashes in topic name for liveliness tokens
        // Unlike TopicKE which uses slashes as part of the key expression,
        // LivelinessKE uses slashes as field delimiters, so ALL slashes
        // within the topic name field must be escaped to %
        let topic_name = {
            let s = topic_name.strip_suffix('/').unwrap_or(topic_name);
            mangle_name(s)
        };
        let type_info = type_info
            .as_ref()
            .map_or(format!("{EMPTY_TOPIC_TYPE}/{EMPTY_TOPIC_HASH}"), |x| {
                format!("{}/{}", mangle_name(&x.name), x.hash.to_rihs_string())
            });
        let qos = qos.encode();

        let ke = format!(
            "{ADMIN_SPACE}/{domain_id}/{z_id}/{node_id}/{id}/{kind}/{node_enclave_str}/{node_namespace}/{node_name}/{topic_name}/{type_info}/{qos}",
        );

        debug!(
            "[ENT] Liveliness KE: topic={}, kind={:?}, ke={}",
            value.topic, value.kind, ke
        );

        Ok(LivelinessKE(ke.try_into()?))
=======
/// Convert an Entity to a LivelinessKE using the default format
pub fn entity_to_liveliness_ke(entity: &Entity) -> Result<LivelinessKE> {
    match entity {
        Entity::Node(n) => node_to_liveliness_ke(n),
        Entity::Endpoint(e) => endpoint_to_liveliness_ke(e),
>>>>>>> 9acf7d3 (refactor: rename to ros-z-protocol and remove backend trait)
    }
}

/// Get the kind of entity
pub fn entity_kind(entity: &Entity) -> EntityKind {
    match entity {
        Entity::Node(_) => EntityKind::Node,
        Entity::Endpoint(e) => e.kind,
    }
}

<<<<<<< HEAD
impl TryFrom<&Entity> for LivelinessKE {
    type Error = zenoh::Error;
    fn try_from(value: &Entity) -> std::result::Result<Self, Self::Error> {
        match value {
            Entity::Node(node) => LivelinessKE::try_from(node),
            Entity::Endpoint(endpoint) => LivelinessKE::try_from(endpoint),
        }
    }
}

impl Display for EndpointEntity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let ke: LivelinessKE = self.try_into().expect("Failed to parse EndpointEntity");
        write!(f, "{}", ke.0)
    }
}

impl TryFrom<&EndpointEntity> for TopicKE {
    type Error = zenoh::Error;

    // <domain_id>/<topic_name>/<topic_type>/<topic_type_hash>
    fn try_from(value: &EndpointEntity) -> std::result::Result<Self, Self::Error> {
        let NodeEntity { domain_id, .. } = value.node;
        let topic = {
            let s = &value.topic;
            let s = s.strip_prefix('/').unwrap_or(s);
            let s = s.strip_suffix('/').unwrap_or(s);

            // For services, keep slashes as-is to match rmw_zenoh_cpp format
            // Services include regular services and action services
            // rmw_zenoh_cpp uses strip_slashes() which only removes leading/trailing slashes
            if value.kind == EntityKind::Service || value.kind == EntityKind::Client {
                s.to_string()
            } else if let Some(pos) = s.find("/_action/") {
                // Special handling for action pub/sub: keep /_action/ as / in key expression
                // Action services use /_action/ as infrastructure naming, which should be
                // literal slashes in the Zenoh key expression (like type names)
                let (base, action_suffix) = s.split_at(pos);
                // Mangle the base action name, keep /_action/ as /, mangle the service type
                let action_suffix = &action_suffix[1..]; // Remove leading /
                format!("{}/{}", mangle_name(base), action_suffix)
            } else {
                // For topics (publishers/subscribers), mangle the name
                mangle_name(s)
            }
        };
        let type_info = value.type_info.as_ref().map_or(
            format!("{EMPTY_TOPIC_TYPE}/{EMPTY_TOPIC_HASH}"),
            |x| {
                let type_name = demangle_name(&x.name);
                let type_hash = demangle_name(&x.hash.to_string());
                format!("{type_name}/{type_hash}")
            },
        );
        Ok(TopicKE(
            format!("{domain_id}/{topic}/{type_info}").try_into()?,
        ))
    }
}

impl TryFrom<EndpointEntity> for TopicKE {
    type Error = zenoh::Error;
    fn try_from(value: EndpointEntity) -> std::result::Result<Self, Self::Error> {
        TopicKE::try_from(&value)
    }
}

impl EndpointEntity {
    /// Generate topic key expression using the default (rmw_zenoh) format.
    pub fn topic_key_expr(&self) -> Result<KeyExpr<'static>> {
        let ke: TopicKE = self.try_into()?;
        Ok(ke.0)
    }

    /// Generate topic key expression using a specific backend.
    pub fn topic_key_expr_with<B: crate::backend::KeyExprBackend>(
        &self,
    ) -> Result<KeyExpr<'static>> {
        let ke = B::topic_key_expr(self)?;
        Ok(ke.0)
    }

    /// Generate liveliness token using the default (rmw_zenoh) format.
    pub fn lv_token_key_expr(&self) -> Result<KeyExpr<'static>> {
        let ke: LivelinessKE = self.try_into()?;
        Ok(ke.0)
    }

    /// Generate liveliness token using a specific backend.
    pub fn lv_token_key_expr_with<B: crate::backend::KeyExprBackend>(
        &self,
        zid: &zenoh::session::ZenohId,
    ) -> Result<KeyExpr<'static>> {
        let ke = B::liveliness_key_expr(self, zid)?;
        Ok(ke.0)
    }

    pub fn gid(&self) -> GidArray {
        let hash = sha2::Sha256::digest(self.to_string().as_bytes());
        let mut gid = GidArray::default();
        let len = gid.len();
        gid.copy_from_slice(&hash[..len]);
        debug!(
            "[ENT] Generated GID for topic={}: {:02x?}",
            self.topic,
            &gid[..4]
        );
        gid
    }
}

#[derive(Debug, Hash, PartialEq, Eq, Clone)]
pub enum Entity {
    Node(NodeEntity),
    Endpoint(EndpointEntity),
}

impl Entity {
    pub fn get_endpoint(&self) -> Option<&EndpointEntity> {
        match self {
            Self::Endpoint(x) => Some(x),
            _ => None,
        }
    }

    pub fn kind(&self) -> EntityKind {
        match self {
            Self::Node(_) => EntityKind::Node,
            Self::Endpoint(x) => x.kind,
        }
    }

    pub fn id(&self) -> u64 {
        match self {
            Self::Node(x) => x.id as u64,
            Self::Endpoint(x) => x.id as u64,
        }
    }
}

#[derive(Debug)]
pub enum EntityConversionError {
    MissingAdminSpace,
    MissingDomainId,
    MissingZId,
    MissingNodeId,
    MissingEntityId,
    MissingEnclave,
    MissingNamespace,
    MissingNodeName,
    MissingEntityKind,
    MissingTopicName,
    MissingTopicType,
    MissingTopicHash,
    MissingTopicQoS,
    QosDecodeError(crate::qos::QosDecodeError),
    ParsingError,
}

impl std::fmt::Display for EntityConversionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MissingAdminSpace => write!(f, "Missing admin space in key expression"),
            Self::MissingDomainId => write!(f, "Missing domain ID in key expression"),
            Self::MissingZId => write!(f, "Missing Zenoh ID in key expression"),
            Self::MissingNodeId => write!(f, "Missing node ID in key expression"),
            Self::MissingEntityId => write!(f, "Missing entity ID in key expression"),
            Self::MissingEnclave => write!(f, "Missing enclave in key expression"),
            Self::MissingNamespace => write!(f, "Missing namespace in key expression"),
            Self::MissingNodeName => write!(f, "Missing node name in key expression"),
            Self::MissingEntityKind => write!(f, "Missing entity kind in key expression"),
            Self::MissingTopicName => write!(f, "Missing topic name in key expression"),
            Self::MissingTopicType => write!(f, "Missing topic type in key expression"),
            Self::MissingTopicHash => write!(f, "Missing topic hash in key expression"),
            Self::MissingTopicQoS => write!(f, "Missing topic QoS in key expression"),
            Self::QosDecodeError(e) => write!(f, "QoS decode error: {}", e),
            Self::ParsingError => write!(f, "Parsing error in key expression"),
        }
    }
}

impl std::error::Error for EntityConversionError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::QosDecodeError(e) => Some(e),
            _ => None,
        }
    }
}

impl TryFrom<&LivelinessKE> for Entity {
    type Error = EntityConversionError;
    fn try_from(value: &LivelinessKE) -> std::result::Result<Entity, Self::Error> {
        // Possible formats
        // - <ADMIN_SPACE>/<domain_id>/<zid>/<nid>/<eid>/<entity_kind>/<enclave>/<namespace>/<node_name>/<topic_name>/<topic_type>/<topic_type_hash>/<topic_qos>
        // - <ADMIN_SPACE>/<domain_id>/<zid>/<nid>/<eid>/<entity_kind>/<enclave>/<namespace>/<node_name>
        use EntityConversionError::*;
        let mut iter = value.split("/");
        assert_eq!(ADMIN_SPACE, iter.next().ok_or(MissingAdminSpace)?);
        let domain_id = iter
            .next()
            .ok_or(MissingDomainId)?
            .parse()
            .map_err(|_| ParsingError)?;
        let z_id = iter
            .next()
            .ok_or(MissingZId)?
            .parse()
            .map_err(|_| ParsingError)?;
        let node_id = iter
            .next()
            .ok_or(MissingNodeId)?
            .parse()
            .map_err(|_| ParsingError)?;
        let entity_id = iter
            .next()
            .ok_or(MissingEntityId)?
            .parse()
            .map_err(|_| ParsingError)?;
        let entity_kind: EntityKind = iter
            .next()
            .ok_or(MissingEntityKind)?
            .parse()
            .map_err(|_| ParsingError)?;
        let enclave = match iter.next().ok_or(MissingEnclave)? {
            EMPTY_ENCLAVE => "",
            x => &demangle_name(x),
        };
        let namespace = match iter.next().ok_or(MissingNamespace)? {
            EMPTY_NAMESPACE => "",
            x => &demangle_name(x),
        };
        let node_name = demangle_name(iter.next().ok_or(MissingNodeName)?);

        let node = NodeEntity {
            id: node_id,
            domain_id,
            z_id,
            name: node_name,
            namespace: namespace.to_string(),
            enclave: enclave.to_string(),
        };
        Ok(match entity_kind {
            EntityKind::Node => Entity::Node(node),
            _ => {
                let topic_name = demangle_name(iter.next().ok_or(MissingTopicName)?);
                let topic_type = iter.next().ok_or(MissingTopicType)?;
                let topic_hash = iter.next().ok_or(MissingTopicType)?;
                let type_info = match (topic_type, topic_hash) {
                    (EMPTY_TOPIC_TYPE, EMPTY_TOPIC_HASH) => None,
                    (EMPTY_TOPIC_TYPE, _) | (_, EMPTY_TOPIC_HASH) => unreachable!(),
                    (topic_type, topic_hash) => {
                        let type_hash = TypeHash::from_rihs_string(topic_hash)
                            .unwrap_or(TypeHash::new(0, [0u8; 32]));
                        Some(TypeInfo {
                            name: demangle_name(topic_type),
                            hash: type_hash,
                        })
                    }
                };
                let qos = QosProfile::decode(iter.next().ok_or(MissingTopicQoS)?)
                    .map_err(QosDecodeError)?;
                Entity::Endpoint(EndpointEntity {
                    id: entity_id,
                    node,
                    kind: entity_kind,
                    topic: topic_name,
                    type_info,
                    qos,
                })
            }
        })
=======
/// Get the endpoint entity if this is an endpoint
pub fn entity_get_endpoint(entity: &Entity) -> Option<&EndpointEntity> {
    match entity {
        Entity::Node(_) => None,
        Entity::Endpoint(e) => Some(e),
>>>>>>> 9acf7d3 (refactor: rename to ros-z-protocol and remove backend trait)
    }
}
