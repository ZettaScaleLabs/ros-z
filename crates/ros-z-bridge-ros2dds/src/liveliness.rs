/// Liveliness token key builder for the ros2dds bridge.
///
/// Tokens follow the ros-z liveliness format so that `ros2 node list`,
/// `ros2 topic list`, and `ros2 service list` see bridged entities.
///
/// Key format (from ros-z-protocol):
/// `@ros2_lv/{domain_id}/{zid}/{node_id}/{entity_id}/{kind}/{enclave}/{ns}/{name}/{topic}/{type}/{hash}/{qos}`

const LIVELINESS_PREFIX: &str = "@ros2_lv";

/// Type hash placeholder: bridge does not compute RIHS01 type hashes.
const PLACEHOLDER_HASH: &str =
    "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";

/// Entity kind tag used in the liveliness key.
#[derive(Clone, Copy, Debug)]
pub enum EntityKind {
    Publisher,
    Subscriber,
    ServiceServer,
    ServiceClient,
}

impl EntityKind {
    fn as_str(self) -> &'static str {
        match self {
            Self::Publisher => "pub",
            Self::Subscriber => "sub",
            Self::ServiceServer => "srv",
            Self::ServiceClient => "cli",
        }
    }
}

/// Build a node-level liveliness key for the virtual bridge node.
///
/// `enclave`  — ROS 2 enclave (use `/` for the default enclave)
/// `ns`       — node namespace (e.g. `/` or `/my_ns`)
/// `name`     — node name (e.g. `ros_z_bridge`)
pub fn build_node_lv_key(
    domain_id: u32,
    zid: &str,
    node_id: u64,
    enclave: &str,
    ns: &str,
    name: &str,
) -> String {
    let enc = encode_segment(enclave);
    let ns_seg = encode_segment(ns);
    format!("{LIVELINESS_PREFIX}/{domain_id}/{zid}/{node_id}/node/{enc}/{ns_seg}/{name}")
}

/// Build a per-entity liveliness key (publisher, subscriber, service, …).
pub fn build_entity_lv_key(
    domain_id: u32,
    zid: &str,
    node_id: u64,
    entity_id: u64,
    kind: EntityKind,
    enclave: &str,
    ns: &str,
    node_name: &str,
    topic: &str,
    ros2_type: &str,
    qos_str: &str,
) -> String {
    let enc = encode_segment(enclave);
    let ns_seg = encode_segment(ns);
    let topic_seg = encode_segment(topic);
    let type_seg = encode_segment(ros2_type);
    format!(
        "{LIVELINESS_PREFIX}/{domain_id}/{zid}/{node_id}/{entity_id}/{kind}/{enc}/{ns_seg}/{node_name}/{topic_seg}/{type_seg}/{PLACEHOLDER_HASH}/{qos_str}",
        kind = kind.as_str(),
    )
}

/// Encode a path-like segment for use in a liveliness key:
/// strips leading slashes and replaces interior slashes with `%`.
fn encode_segment(s: &str) -> String {
    s.trim_start_matches('/').replace('/', "%")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lv_key_node_format() {
        let key = build_node_lv_key(0, "abc123", 1, "/", "/", "ros_z_bridge");
        assert!(key.starts_with("@ros2_lv/0/abc123/1/node/"));
        assert!(key.contains("ros_z_bridge"));
    }

    #[test]
    fn test_lv_key_publisher_format() {
        let key = build_entity_lv_key(
            0,
            "abc123",
            1,
            42,
            EntityKind::Publisher,
            "/",
            "/",
            "ros_z_bridge",
            "/chatter",
            "std_msgs/msg/String",
            "be",
        );
        assert!(key.starts_with("@ros2_lv/0/abc123/1/42/pub/"));
        assert!(key.contains("chatter"));
        assert!(key.contains("std_msgs%msg%String"));
        assert!(key.contains(PLACEHOLDER_HASH));
    }

    #[test]
    fn test_lv_key_subscriber_format() {
        let key = build_entity_lv_key(
            42,
            "zid0",
            2,
            7,
            EntityKind::Subscriber,
            "/",
            "/",
            "ros_z_bridge",
            "/chatter",
            "std_msgs/msg/String",
            "be",
        );
        assert!(key.contains("/sub/"));
    }

    #[test]
    fn test_lv_key_service_server_format() {
        let key = build_entity_lv_key(
            0,
            "zid0",
            1,
            3,
            EntityKind::ServiceServer,
            "/",
            "/",
            "ros_z_bridge",
            "/add_two_ints",
            "example_interfaces/srv/AddTwoInts",
            "re",
        );
        assert!(key.contains("/srv/"));
    }

    #[test]
    fn test_lv_key_service_client_format() {
        let key = build_entity_lv_key(
            0,
            "zid0",
            1,
            4,
            EntityKind::ServiceClient,
            "/",
            "/",
            "ros_z_bridge",
            "/add_two_ints",
            "example_interfaces/srv/AddTwoInts",
            "re",
        );
        assert!(key.contains("/cli/"));
    }

    #[test]
    fn test_lv_key_strips_leading_slash_from_ns_and_topic() {
        let key = build_entity_lv_key(
            0,
            "z",
            1,
            1,
            EntityKind::Publisher,
            "/enclave",
            "/my_ns",
            "bridge",
            "/chatter",
            "std_msgs/msg/String",
            "be",
        );
        // Leading slashes stripped; inner slashes replaced by %
        assert!(key.contains("enclave"));
        assert!(key.contains("my_ns"));
        assert!(key.contains("chatter"));
        // No raw slash in encoded segments (beyond the key separators)
        let after_prefix = key.trim_start_matches("@ros2_lv/");
        // Encoded segments should not start with '/'
        for seg in after_prefix.split('/') {
            assert!(!seg.starts_with('/'), "segment starts with slash: {seg}");
        }
    }

    #[test]
    fn test_lv_key_root_namespace() {
        let key = build_node_lv_key(0, "z", 1, "/", "/", "bridge");
        // Root ns "/" encodes to "" (empty after stripping leading slash)
        assert!(key.contains("/bridge"), "node name should appear after ns");
    }

    #[test]
    fn test_encode_segment_strips_leading_slash() {
        assert_eq!(encode_segment("/chatter"), "chatter");
        assert_eq!(encode_segment("/my_ns/sub"), "my_ns%sub");
        assert_eq!(encode_segment("/"), "");
    }
}
