//! ROS 2 entity types for key expression generation.

#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use alloc::string::String;
use core::{fmt::Display, ops::Deref};
use zenoh::{key_expr::KeyExpr, session::ZenohId};

use crate::qos::QosProfile;

/// Placeholder for empty namespace/enclave.
pub const EMPTY_PLACEHOLDER: &str = "%";

/// Liveliness key expression wrapper.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct LivelinessKE(pub KeyExpr<'static>);

impl LivelinessKE {
    pub fn new(ke: KeyExpr<'static>) -> Self {
        Self(ke)
    }
}

impl Deref for LivelinessKE {
    type Target = KeyExpr<'static>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

/// Topic key expression wrapper.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct TopicKE(KeyExpr<'static>);

impl TopicKE {
    pub fn new(ke: KeyExpr<'static>) -> Self {
        Self(ke)
    }
}

impl Deref for TopicKE {
    type Target = KeyExpr<'static>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl Display for TopicKE {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// ROS 2 node entity.
#[derive(Default, Debug, Hash, Clone, PartialEq, Eq)]
pub struct NodeEntity {
    pub domain_id: usize,
    pub z_id: ZenohId,
    pub id: usize,
    pub name: String,
    pub namespace: String,
    pub enclave: String,
}

impl NodeEntity {
    pub fn new(
        domain_id: usize,
        z_id: ZenohId,
        id: usize,
        name: String,
        namespace: String,
        enclave: String,
    ) -> Self {
        Self {
            domain_id,
            z_id,
            id,
            name,
            namespace,
            enclave,
        }
    }
}

/// ROS 2 entity kind (node, publisher, subscription, service, client).
#[derive(Default, Debug, Hash, Clone, Copy, PartialEq, Eq)]
pub enum EntityKind {
    #[default]
    Node,
    Publisher,
    Subscription,
    Service,
    Client,
}

impl Display for EntityKind {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            EntityKind::Node => write!(f, "NN"),
            EntityKind::Publisher => write!(f, "MP"),
            EntityKind::Subscription => write!(f, "MS"),
            EntityKind::Service => write!(f, "SS"),
            EntityKind::Client => write!(f, "SC"),
        }
    }
}

impl core::str::FromStr for EntityKind {
    type Err = &'static str;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "NN" => Ok(EntityKind::Node),
            "MP" => Ok(EntityKind::Publisher),
            "MS" => Ok(EntityKind::Subscription),
            "SS" => Ok(EntityKind::Service),
            "SC" => Ok(EntityKind::Client),
            _ => Err("Invalid entity kind"),
        }
    }
}

/// Type hash (RIHS format).
#[derive(Debug, Hash, PartialEq, Eq, Clone)]
pub struct TypeHash {
    pub version: u8,
    pub value: [u8; 32],
}

impl TypeHash {
    pub const fn new(version: u8, value: [u8; 32]) -> Self {
        Self { version, value }
    }

    pub const fn zero() -> Self {
        Self {
            version: 1,
            value: [0u8; 32],
        }
    }

    pub fn from_rihs_string(rihs_str: &str) -> Option<Self> {
        if let Some(hex_part) = rihs_str.strip_prefix("RIHS01_") {
            if hex_part.len() == 64 {
                let mut hash_bytes = [0u8; 32];
                for (i, chunk) in hex_part.as_bytes().chunks(2).enumerate() {
                    if i < 32 {
                        if let Ok(byte_val) =
                            u8::from_str_radix(core::str::from_utf8(chunk).unwrap_or("00"), 16)
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
        }
        None
    }

    pub fn to_rihs_string(&self) -> String {
        use alloc::format;
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

impl Display for TypeHash {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.to_rihs_string())
    }
}

/// Type information (name + hash).
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

/// ROS 2 endpoint entity (publisher, subscription, service, client).
#[derive(Default, Debug, Hash, PartialEq, Eq, Clone)]
pub struct EndpointEntity {
    pub id: usize,
    pub node: NodeEntity,
    pub kind: EntityKind,
    pub topic: String,
    pub type_info: Option<TypeInfo>,
    pub qos: QosProfile,
}

/// Generic ROS 2 entity (node or endpoint).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Entity {
    Node(NodeEntity),
    Endpoint(EndpointEntity),
}

/// Errors during entity conversion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityConversionError {
    MissingAdminSpace,
    MissingDomainId,
    MissingZId,
    MissingNodeId,
    MissingEntityId,
    MissingEntityKind,
    MissingEnclave,
    MissingNamespace,
    MissingNodeName,
    MissingTopicName,
    MissingTopicType,
    MissingTopicHash,
    MissingTopicQoS,
    ParsingError,
    QosDecodeError(crate::qos::QosDecodeError),
}

impl Display for EntityConversionError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for EntityConversionError {}
