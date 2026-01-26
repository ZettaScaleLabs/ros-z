use std::fmt::Display;
use std::ops::Deref;

use zenoh::{Result, key_expr::KeyExpr, session::ZenohId};
use tracing::debug;

use crate::{attachment::GidArray, qos::QosProfile};
use sha2::Digest;

const EMPTY_NAMESPACE: &str = "%";
const EMPTY_ENCLAVE: &str = "%";
const EMPTY_TOPIC_TYPE: &str = "EMPTY_TOPIC_TYPE";
const EMPTY_TOPIC_HASH: &str = "EMPTY_TOPIC_HASH";
#[cfg(feature = "no-type-hash")]
const HUMBLE_TYPE_HASH_PLACEHOLDER: &str = "TypeHashNotSupported";
pub const ADMIN_SPACE: &str = "@ros2_lv";

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct LivelinessKE(pub KeyExpr<'static>);

impl Deref for LivelinessKE {
    type Target = KeyExpr<'static>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

pub struct TopicKE(KeyExpr<'static>);

impl Deref for TopicKE {
    type Target = KeyExpr<'static>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

type NodeName = String;
type NodeNamespace = String;
pub type NodeKey = (NodeNamespace, NodeName);

#[derive(Default, Debug, Hash, Clone, PartialEq, Eq)]
pub struct NodeEntity {
    pub domain_id: usize,
    pub z_id: ZenohId,
    pub id: usize,
    pub name: String,
    pub namespace: String,
}

impl NodeEntity {
    pub fn new(
        domain_id: usize,
        z_id: ZenohId,
        id: usize,
        name: String,
        namespace: String,
    ) -> Self {
        Self {
            domain_id,
            z_id,
            id,
            name,
            namespace,
        }
    }

    pub fn key(&self) -> NodeKey {
        let denormalized_ns = if self.namespace.is_empty() {
            "/".to_string()
        } else if !self.namespace.starts_with('/') {
            format!("/{}", self.namespace)
        } else {
            self.namespace.clone()
        };
        (denormalized_ns, self.name.clone())
    }

    pub fn lv_token_key_expr(&self) -> Result<KeyExpr<'static>> {
        let ke: LivelinessKE = self.try_into()?;
        Ok(ke.0)
    }
}

impl TryFrom<&NodeEntity> for LivelinessKE {
    type Error = zenoh::Error;

    // <ADMIN_SPACE>/<domain_id>/<zid>/<nid>/<eid>/<entity_kind>/<enclave>/<namespace>/<node_name>
    // NOTE: enclave is not supported yet
    fn try_from(value: &NodeEntity) -> std::result::Result<Self, Self::Error> {
        let NodeEntity {
            domain_id,
            z_id,
            id,
            name,
            namespace,
        } = value;
        let namespace = if namespace.is_empty() {
            EMPTY_NAMESPACE
        } else {
            &mangle_name(namespace)
        };
        let entity_kind = EntityKind::Node;
        let name = mangle_name(name);
        Ok(LivelinessKE(
            format!("{ADMIN_SPACE}/{domain_id}/{z_id}/{id}/{id}/{entity_kind}/{EMPTY_ENCLAVE}/{namespace}/{name}")
                .try_into()?,
        ))
    }
}

#[derive(Default, Debug, Hash, strum::EnumString, strum::Display, Eq, PartialEq, Clone, Copy)]
pub enum EntityKind {
    #[default]
    #[strum(serialize = "NN")]
    Node,
    #[strum(serialize = "MP")]
    Publisher,
    #[strum(serialize = "MS")]
    Subscription,
    #[strum(serialize = "SS")]
    Service,
    #[strum(serialize = "SC")]
    Client,
}

#[derive(Debug, Hash, PartialEq, Eq, Clone)]
pub struct TypeHash {
    pub version: u8,
    pub value: [u8; 32],
}

impl TypeHash {
    pub const fn new(version: u8, value: [u8; 32]) -> Self {
        Self { version, value }
    }

    /// Creates a zero/placeholder TypeHash (RIHS01 version with all zeros)
    /// Useful for generic wrapper types where actual hash is instance-specific
    pub const fn zero() -> Self {
        Self {
            version: 1,
            value: [0u8; 32],
        }
    }

    /// Creates a Humble-compatible placeholder TypeHash
    /// In Humble (rmw_zenoh v0.1.8), type hashes are not supported,
    /// so a constant placeholder "TypeHashNotSupported" is used instead
    #[cfg(feature = "humble-compat")]
    pub const fn humble_placeholder() -> Self {
        Self::zero() // Use zero hash as placeholder for Humble
    }

    pub fn from_rihs_string(rihs_str: &str) -> Option<Self> {
        // Handle Humble's placeholder
        #[cfg(feature = "humble-compat")]
        if rihs_str == HUMBLE_TYPE_HASH_PLACEHOLDER {
            return Some(Self::zero());
        }
        if let Some(hex_part) = rihs_str.strip_prefix("RIHS01_")
            && hex_part.len() == 64
        {
            let mut hash_bytes = [0u8; 32];
            for (i, chunk) in hex_part.as_bytes().chunks(2).enumerate() {
                if i < 32 {
                    if let Ok(byte_val) =
                        u8::from_str_radix(std::str::from_utf8(chunk).unwrap_or("00"), 16)
                    {
                        hash_bytes[i] = byte_val;
                    } else {
                        return None;
                    }
                }
            }
            return Some(TypeHash {
                version: 1,
                value: hash_bytes,
            });
        }
        None
    }

    pub fn to_rihs_string(&self) -> String {
        #[cfg(feature = "no-type-hash")]
        {
            // In Humble, always use the placeholder regardless of actual hash value
            HUMBLE_TYPE_HASH_PLACEHOLDER.to_string()
        }

        #[cfg(not(feature = "no-type-hash"))]
        {
            match self.version {
                1 => {
                    let hex_str: String = self.value.iter().map(|b| format!("{:02x}", b)).collect();
                    format!("RIHS01_{}", hex_str)
                }
                _ => format!(
                    "RIHS{:02x}_{}",
                    self.version,
                    self.value
                        .iter()
                        .map(|b| format!("{:02x}", b))
                        .collect::<String>()
                ),
            }
        }
    }
}

impl Display for TypeHash {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_rihs_string())
    }
}

#[derive(Debug, Hash, PartialEq, Eq, Clone)]
pub struct TypeInfo {
    pub name: String,
    pub hash: TypeHash,
}

impl TypeInfo {
    pub fn new(name: &str, hash: TypeHash) -> Self {
        TypeInfo {
            name: name.to_string(),
            hash,
        }
    }
}

impl Display for TypeInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let Self { name, hash } = self;
        write!(f, "{name}/{}", hash.to_rihs_string())
    }
}

pub type Topic = String;

#[derive(Default, Debug, Hash, PartialEq, Eq, Clone)]
pub struct EndpointEntity {
    pub id: usize,
    pub node: NodeEntity,
    pub kind: EntityKind,
    pub topic: Topic,
    pub type_info: Option<TypeInfo>,
    pub qos: QosProfile,
}

fn mangle_name(name: &str) -> String {
    name.replace("/", "%")
}

fn demangle_name(name: &str) -> String {
    name.replace("%", "/")
}

impl TryFrom<&EndpointEntity> for LivelinessKE {
    type Error = zenoh::Error;

    // <ADMIN_SPACE>/<domain_id>/<zid>/<nid>/<eid>/<entity_kind>/<enclave>/<namespace>/<node_name>/<topic_name>/<topic_type>/<topic_type_hash>/<topic_qos>
    // NOTE: enclave is not supported yet
    fn try_from(value: &EndpointEntity) -> std::result::Result<Self, Self::Error> {
        let EndpointEntity {
            id,
            node:
                NodeEntity {
                    domain_id,
                    z_id,
                    id: node_id,
                    name: node_name,
                    namespace: node_namespace,
                },
            kind,
            topic: topic_name,
            type_info,
            qos,
        } = value;

        let node_namespace = if node_namespace.is_empty() {
            EMPTY_NAMESPACE
        } else {
            &mangle_name(node_namespace)
        };
        let node_name = mangle_name(node_name);

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
            "{ADMIN_SPACE}/{domain_id}/{z_id}/{node_id}/{id}/{kind}/{EMPTY_ENCLAVE}/{node_namespace}/{node_name}/{topic_name}/{type_info}/{qos}",
        );

        debug!("[ENT] Liveliness KE: topic={}, kind={:?}, ke={}",
            value.topic, value.kind, ke);

        Ok(LivelinessKE(ke.try_into()?))
    }
}

impl TryFrom<EndpointEntity> for LivelinessKE {
    type Error = zenoh::Error;
    fn try_from(value: EndpointEntity) -> std::result::Result<Self, Self::Error> {
        LivelinessKE::try_from(&value)
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

            // Special handling for action services: keep /_action/ as / in key expression
            // Action services use /_action/ as infrastructure naming, which should be
            // literal slashes in the Zenoh key expression (like type names)
            if let Some(pos) = s.find("/_action/") {
                let (base, action_suffix) = s.split_at(pos);
                // Mangle the base action name, keep /_action/ as /, mangle the service type
                let action_suffix = &action_suffix[1..]; // Remove leading /
                format!("{}/{}", mangle_name(base), action_suffix)
            } else {
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
    pub fn topic_key_expr(&self) -> Result<KeyExpr<'static>> {
        let ke: TopicKE = self.try_into()?;
        Ok(ke.0)
    }

    pub fn lv_token_key_expr(&self) -> Result<KeyExpr<'static>> {
        let ke: LivelinessKE = self.try_into()?;
        Ok(ke.0)
    }

    pub fn gid(&self) -> GidArray {
        let hash = sha2::Sha256::digest(self.to_string().as_bytes());
        let mut gid = GidArray::default();
        let len = gid.len();
        gid.copy_from_slice(&hash[..len]);
        debug!("[ENT] Generated GID for topic={}: {:02x?}", self.topic, &gid[..4]);
        gid
    }
}

#[derive(Debug, Hash, PartialEq, Eq)]
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
        // NOTE: enclave is not supported yet
        let _enclave = match iter.next().ok_or(MissingEnclave)? {
            EMPTY_NAMESPACE => "",
            _ => unreachable!(),
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
    }
}
