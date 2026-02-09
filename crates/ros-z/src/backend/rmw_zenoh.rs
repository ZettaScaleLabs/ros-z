//! rmw_zenoh compatible backend.
//!
//! This backend generates key expressions compatible with rmw_zenoh.
//!
//! Key expression formats:
//! - Topic: `<domain_id>/<topic>/<type>/<hash>`
//! - Liveliness: `@ros2_lv/<domain_id>/<zid>/<nid>/<eid>/<kind>/<enclave>/<ns>/<name>[/<topic>/<type>/<hash>/<qos>]`

use zenoh::{Result, key_expr::KeyExpr, session::ZenohId};

use crate::{
    entity::{
        EndpointEntity, Entity, EntityConversionError, EntityKind, LivelinessKE, NodeEntity,
        TopicKE, TypeHash, TypeInfo,
    },
    qos::QosProfile,
};

use super::KeyExprBackend;

/// Placeholder for empty namespace/enclave.
pub const EMPTY_PLACEHOLDER: &str = "%";
/// Placeholder for empty topic type.
pub const EMPTY_TOPIC_TYPE: &str = "EMPTY_TOPIC_TYPE";
/// Placeholder for empty topic hash.
pub const EMPTY_TOPIC_HASH: &str = "EMPTY_TOPIC_HASH";

/// rmw_zenoh compatible backend.
pub struct RmwZenohBackend;

impl KeyExprBackend for RmwZenohBackend {
    const ESCAPE_CHAR: char = '%';
    const ADMIN_SPACE: &'static str = "@ros2_lv";

    fn topic_key_expr(entity: &EndpointEntity) -> Result<TopicKE> {
        let domain_id = entity.node.domain_id;
        let topic = {
            let s = &entity.topic;
            let s = s.strip_prefix('/').unwrap_or(s);
            let s = s.strip_suffix('/').unwrap_or(s);

            // Special handling for action services: keep /_action/ as / in key expression
            if let Some(pos) = s.find("/_action/") {
                let (base, action_suffix) = s.split_at(pos);
                let action_suffix = &action_suffix[1..]; // Remove leading /
                format!("{}/{}", Self::mangle_name(base), action_suffix)
            } else {
                Self::mangle_name(s)
            }
        };

        let type_info = entity.type_info.as_ref().map_or(
            format!("{EMPTY_TOPIC_TYPE}/{EMPTY_TOPIC_HASH}"),
            |x| {
                let type_name = Self::demangle_name(&x.name);
                let type_hash = Self::demangle_name(&x.hash.to_string());
                format!("{type_name}/{type_hash}")
            },
        );

        Ok(TopicKE::new(
            format!("{domain_id}/{topic}/{type_info}").try_into()?,
        ))
    }

    fn liveliness_key_expr(entity: &EndpointEntity, _zid: &ZenohId) -> Result<LivelinessKE> {
        let EndpointEntity {
            id,
            node:
                NodeEntity {
                    domain_id,
                    z_id,
                    id: node_id,
                    name: node_name,
                    namespace: node_namespace,
                    enclave: _,
                },
            kind,
            topic: topic_name,
            type_info,
            qos,
        } = entity;

        let node_namespace = if node_namespace.is_empty() {
            EMPTY_PLACEHOLDER.to_string()
        } else {
            Self::mangle_name(node_namespace)
        };
        let node_name = Self::mangle_name(node_name);

        // Mangle all slashes in topic name for liveliness tokens
        let topic_name = {
            let s = topic_name.strip_suffix('/').unwrap_or(topic_name);
            Self::mangle_name(s)
        };

        let type_info_str = type_info
            .as_ref()
            .map_or(format!("{EMPTY_TOPIC_TYPE}/{EMPTY_TOPIC_HASH}"), |x| {
                format!("{}/{}", Self::mangle_name(&x.name), x.hash.to_rihs_string())
            });

        let qos_str = qos.encode();

        let ke = format!(
            "{}/{domain_id}/{z_id}/{node_id}/{id}/{kind}/{EMPTY_PLACEHOLDER}/{node_namespace}/{node_name}/{topic_name}/{type_info_str}/{qos_str}",
            Self::ADMIN_SPACE
        );

        Ok(LivelinessKE::new(ke.try_into()?))
    }

    fn node_liveliness_key_expr(entity: &NodeEntity) -> Result<LivelinessKE> {
        let NodeEntity {
            domain_id,
            z_id,
            id,
            name,
            namespace,
            enclave: _,
        } = entity;

        let namespace = if namespace.is_empty() {
            EMPTY_PLACEHOLDER
        } else {
            &Self::mangle_name(namespace)
        };
        let name = Self::mangle_name(name);

        Ok(LivelinessKE::new(
            format!(
                "{}/{domain_id}/{z_id}/{id}/{id}/NN/{EMPTY_PLACEHOLDER}/{namespace}/{name}",
                Self::ADMIN_SPACE
            )
            .try_into()?,
        ))
    }

    fn parse_liveliness(ke: &KeyExpr) -> Result<Entity> {
        use EntityConversionError::*;

        let mut iter = ke.split('/');

        // Check admin space prefix
        let admin = iter.next().ok_or(MissingAdminSpace)?;
        if admin != Self::ADMIN_SPACE {
            return Err(zenoh::Error::from(MissingAdminSpace));
        }

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

        // Enclave (not supported yet)
        let enclave = match iter.next().ok_or(MissingEnclave)? {
            EMPTY_PLACEHOLDER => String::new(),
            x => Self::demangle_name(x),
        };

        let namespace = match iter.next().ok_or(MissingNamespace)? {
            EMPTY_PLACEHOLDER => String::new(),
            x => Self::demangle_name(x),
        };
        let node_name = Self::demangle_name(iter.next().ok_or(MissingNodeName)?);

        let node = NodeEntity {
            id: node_id,
            domain_id,
            z_id,
            name: node_name,
            namespace,
            enclave,
        };

        Ok(match entity_kind {
            EntityKind::Node => Entity::Node(node),
            _ => {
                let topic_name = Self::demangle_name(iter.next().ok_or(MissingTopicName)?);
                let topic_type = iter.next().ok_or(MissingTopicType)?;
                let topic_hash = iter.next().ok_or(MissingTopicHash)?;

                let type_info = match (topic_type, topic_hash) {
                    (EMPTY_TOPIC_TYPE, EMPTY_TOPIC_HASH) => None,
                    (EMPTY_TOPIC_TYPE, _) | (_, EMPTY_TOPIC_HASH) => None,
                    (topic_type, topic_hash) => {
                        let type_hash = TypeHash::from_rihs_string(topic_hash)
                            .unwrap_or(TypeHash::new(0, [0u8; 32]));
                        Some(TypeInfo {
                            name: Self::demangle_name(topic_type),
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

    fn encode_qos(qos: &QosProfile, _keyless: bool) -> String {
        qos.encode()
    }

    fn decode_qos(s: &str) -> Result<(bool, QosProfile)> {
        let qos = QosProfile::decode(s)
            .map_err(|e| zenoh::Error::from(format!("QoS decode error: {:?}", e)))?;
        Ok((false, qos))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entity::{EndpointEntity, EntityKind, NodeEntity, TypeInfo};
    use crate::qos::{QosDurability, QosHistory, QosProfile, QosReliability};

    #[test]
    fn test_mangle_demangle() {
        assert_eq!(RmwZenohBackend::mangle_name("/chatter"), "%chatter");
        assert_eq!(
            RmwZenohBackend::mangle_name("std_msgs/msg/String"),
            "std_msgs%msg%String"
        );
        assert_eq!(
            RmwZenohBackend::demangle_name("std_msgs%msg%String"),
            "std_msgs/msg/String"
        );
    }

    #[test]
    fn test_qos_encode_decode() {
        let qos = QosProfile::default();
        let encoded = RmwZenohBackend::encode_qos(&qos, false);

        let (keyless, decoded) = RmwZenohBackend::decode_qos(&encoded).unwrap();
        assert!(!keyless);
        assert_eq!(decoded.reliability, qos.reliability);
        assert_eq!(decoded.durability, qos.durability);
    }

    #[test]
    fn test_qos_reliable_transient() {
        let qos = QosProfile {
            reliability: QosReliability::Reliable,
            durability: QosDurability::TransientLocal,
            history: QosHistory::from_depth(10),
            ..Default::default()
        };
        let encoded = RmwZenohBackend::encode_qos(&qos, false);

        let (keyless, decoded) = RmwZenohBackend::decode_qos(&encoded).unwrap();
        assert!(!keyless);
        assert_eq!(decoded.reliability, QosReliability::Reliable);
        assert_eq!(decoded.durability, QosDurability::TransientLocal);
    }

    /// Test topic key expression format matches rmw_zenoh.
    ///
    /// rmw_zenoh format: `<domain_id>/<topic>/<type>/<hash>`
    #[test]
    fn test_topic_key_expr_format() {
        let zid: zenoh::session::ZenohId = "1234567890abcdef1234567890abcdef".parse().unwrap();
        let node = NodeEntity::new(
            0,
            zid,
            0,
            "test_node".to_string(),
            "/".to_string(),
            String::new(),
        );

        let entity = EndpointEntity {
            id: 1,
            node,
            kind: EntityKind::Publisher,
            topic: "chatter".to_string(),
            type_info: Some(TypeInfo::new("std_msgs/msg/String", TypeHash::zero())),
            qos: QosProfile::default(),
        };

        let topic_ke = RmwZenohBackend::topic_key_expr(&entity).unwrap();
        let ke_str = topic_ke.as_str();

        // rmw_zenoh format: <domain_id>/<topic>/<type>/<hash>
        assert!(
            ke_str.starts_with("0/"),
            "Should start with domain ID '0/', got: {}",
            ke_str
        );
        assert!(
            ke_str.contains("/chatter/"),
            "Should contain '/chatter/', got: {}",
            ke_str
        );
        // Note: rmw_zenoh does NOT escape slashes in topic key expression
        assert!(
            ke_str.contains("std_msgs/msg/String"),
            "Should contain type name, got: {}",
            ke_str
        );
    }

    /// Test liveliness key expression format matches rmw_zenoh.
    ///
    /// Format: `@ros2_lv/<domain>/<zid>/<nid>/<eid>/<kind>/<enclave>/<namespace>/<node_name>/<topic>/<type>/<hash>/<qos>`
    #[test]
    fn test_liveliness_key_expr_format() {
        let zid: zenoh::session::ZenohId = "1234567890abcdef1234567890abcdef".parse().unwrap();
        let node = NodeEntity::new(
            0,
            zid,
            0,
            "test_node".to_string(),
            "/".to_string(),
            String::new(),
        );

        let entity = EndpointEntity {
            id: 1,
            node,
            kind: EntityKind::Publisher,
            topic: "chatter".to_string(),
            type_info: Some(TypeInfo::new("std_msgs/msg/String", TypeHash::zero())),
            qos: QosProfile::default(),
        };

        let liveliness_ke = RmwZenohBackend::liveliness_key_expr(&entity, &zid).unwrap();
        let ke_str = liveliness_ke.as_str();

        // Should start with @ros2_lv
        assert!(
            ke_str.starts_with("@ros2_lv/"),
            "Should start with '@ros2_lv/', got: {}",
            ke_str
        );

        // Should contain domain ID
        assert!(
            ke_str.contains("/0/"),
            "Should contain domain '/0/', got: {}",
            ke_str
        );

        // Should contain MP for Publisher
        assert!(
            ke_str.contains("/MP/"),
            "Should contain '/MP/' for Publisher, got: {}",
            ke_str
        );

        // Should contain node name
        assert!(
            ke_str.contains("/test_node/"),
            "Should contain '/test_node/', got: {}",
            ke_str
        );

        // Should contain escaped topic name
        assert!(
            ke_str.contains("/chatter/"),
            "Should contain '/chatter/', got: {}",
            ke_str
        );

        // Should contain escaped type name
        assert!(
            ke_str.contains("std_msgs%msg%String"),
            "Should contain 'std_msgs%msg%String', got: {}",
            ke_str
        );
    }

    /// Test subscriber liveliness key expression
    #[test]
    fn test_subscriber_liveliness_key_expr() {
        let zid: zenoh::session::ZenohId = "1234567890abcdef1234567890abcdef".parse().unwrap();
        let node = NodeEntity::new(
            0,
            zid,
            0,
            "test_node".to_string(),
            "/".to_string(),
            String::new(),
        );

        let entity = EndpointEntity {
            id: 1,
            node,
            kind: EntityKind::Subscription,
            topic: "chatter".to_string(),
            type_info: Some(TypeInfo::new("std_msgs/msg/String", TypeHash::zero())),
            qos: QosProfile::default(),
        };

        let liveliness_ke = RmwZenohBackend::liveliness_key_expr(&entity, &zid).unwrap();
        let ke_str = liveliness_ke.as_str();

        // Should contain MS for Subscription
        assert!(
            ke_str.contains("/MS/"),
            "Should contain '/MS/' for Subscription, got: {}",
            ke_str
        );
    }

    /// Test service server liveliness key expression
    #[test]
    fn test_service_liveliness_key_expr() {
        let zid: zenoh::session::ZenohId = "1234567890abcdef1234567890abcdef".parse().unwrap();
        let node = NodeEntity::new(
            0,
            zid,
            0,
            "test_node".to_string(),
            "/".to_string(),
            String::new(),
        );

        let entity = EndpointEntity {
            id: 1,
            node,
            kind: EntityKind::Service,
            topic: "add_two_ints".to_string(),
            type_info: Some(TypeInfo::new(
                "example_interfaces/srv/AddTwoInts",
                TypeHash::zero(),
            )),
            qos: QosProfile::default(),
        };

        let liveliness_ke = RmwZenohBackend::liveliness_key_expr(&entity, &zid).unwrap();
        let ke_str = liveliness_ke.as_str();

        // Should contain SS for Service
        assert!(
            ke_str.contains("/SS/"),
            "Should contain '/SS/' for Service, got: {}",
            ke_str
        );
    }

    /// Test client liveliness key expression
    #[test]
    fn test_client_liveliness_key_expr() {
        let zid: zenoh::session::ZenohId = "1234567890abcdef1234567890abcdef".parse().unwrap();
        let node = NodeEntity::new(
            0,
            zid,
            0,
            "test_node".to_string(),
            "/".to_string(),
            String::new(),
        );

        let entity = EndpointEntity {
            id: 1,
            node,
            kind: EntityKind::Client,
            topic: "add_two_ints".to_string(),
            type_info: Some(TypeInfo::new(
                "example_interfaces/srv/AddTwoInts",
                TypeHash::zero(),
            )),
            qos: QosProfile::default(),
        };

        let liveliness_ke = RmwZenohBackend::liveliness_key_expr(&entity, &zid).unwrap();
        let ke_str = liveliness_ke.as_str();

        // Should contain SC for Client
        assert!(
            ke_str.contains("/SC/"),
            "Should contain '/SC/' for Client, got: {}",
            ke_str
        );
    }

    /// Test node liveliness key expression
    #[test]
    fn test_node_liveliness_key_expr() {
        let zid: zenoh::session::ZenohId = "1234567890abcdef1234567890abcdef".parse().unwrap();
        let node = NodeEntity::new(
            0,
            zid,
            0,
            "test_node".to_string(),
            "/my_namespace".to_string(),
            String::new(),
        );

        let liveliness_ke = RmwZenohBackend::node_liveliness_key_expr(&node).unwrap();
        let ke_str = liveliness_ke.as_str();

        // Should start with @ros2_lv
        assert!(
            ke_str.starts_with("@ros2_lv/"),
            "Should start with '@ros2_lv/', got: {}",
            ke_str
        );

        // Should contain NN for Node
        assert!(
            ke_str.contains("/NN/"),
            "Should contain '/NN/' for Node, got: {}",
            ke_str
        );

        // Should contain node name
        assert!(
            ke_str.contains("/test_node"),
            "Should contain '/test_node', got: {}",
            ke_str
        );
    }
}
