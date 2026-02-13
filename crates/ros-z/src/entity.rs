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
    hasher.update(entity.id.to_le_bytes());
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

/// Convert an Entity to a LivelinessKE using the default format
pub fn entity_to_liveliness_ke(entity: &Entity) -> Result<LivelinessKE> {
    match entity {
        Entity::Node(n) => node_to_liveliness_ke(n),
        Entity::Endpoint(e) => endpoint_to_liveliness_ke(e),
    }
}

/// Get the kind of entity
pub fn entity_kind(entity: &Entity) -> EntityKind {
    match entity {
        Entity::Node(_) => EntityKind::Node,
        Entity::Endpoint(e) => e.kind,
    }
}

/// Get the endpoint entity if this is an endpoint
pub fn entity_get_endpoint(entity: &Entity) -> Option<&EndpointEntity> {
    match entity {
        Entity::Node(_) => None,
        Entity::Endpoint(e) => Some(e),
    }
}
