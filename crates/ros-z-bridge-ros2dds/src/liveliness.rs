/// Liveliness token key builders matching the zenoh-plugin-ros2dds wire format.
///
/// The plugin uses the following key expression templates:
/// ```text
/// pub:  @/{zenoh_id}/@ros2_lv/MP/{ke}/{typ}/{qos_ke}
/// sub:  @/{zenoh_id}/@ros2_lv/MS/{ke}/{typ}/{qos_ke}
/// srv:  @/{zenoh_id}/@ros2_lv/SS/{ke}/{typ}
/// cli:  @/{zenoh_id}/@ros2_lv/SC/{ke}/{typ}
/// ```
///
/// Where:
/// - `zenoh_id` — Zenoh session ID
/// - `ke` — Zenoh key expression with `/` replaced by `§`
/// - `typ` — ROS 2 type string with `/` replaced by `§`
/// - `qos_ke` — QoS encoded as `{keyless}:{reliability}:{durability}:{kind},{depth}[:{user_data}]`
///
/// References: zenoh-plugin-ros2dds `liveliness_mgt.rs`
use std::fmt::Write as _;

use cyclors::qos::Qos;

/// Slash replacement used inside liveliness key segments.
///
/// The plugin uses `§` (U+00A7) to embed path-like strings (type names, key
/// expressions) inside a single Zenoh key-expression segment without breaking
/// the `/`-separated structure of the key.
const SLASH_REPLACEMENT: &str = "§";

/// Replace `/` with `§` so that a path-like string can be embedded as one
/// Zenoh key-expression segment.
fn escape_slashes(s: &str) -> String {
    s.replace('/', SLASH_REPLACEMENT)
}

/// Encode DDS QoS as the key-expression suffix used in pub/sub liveliness tokens.
///
/// Format (matches zenoh-plugin-ros2dds `qos_to_key_expr`):
/// `{keyless}:{reliability}:{durability}:{history_kind},{history_depth}[:{user_data}]`
///
/// - `keyless` — empty when the topic is keyless; `"K"` when it has key fields
/// - Each QoS field is omitted (empty string) when absent; present fields are
///   encoded as their integer enum discriminant
/// - `user_data` — appended when non-empty (carries the type hash on Jazzy+)
pub fn qos_to_lv_str(keyless: bool, qos: &Qos) -> String {
    let mut w = String::new();
    if !keyless {
        w.push('K');
    }
    w.push(':');
    if let Some(r) = &qos.reliability {
        write!(w, "{}", r.kind as i32).unwrap();
    }
    w.push(':');
    if let Some(d) = &qos.durability {
        write!(w, "{}", d.kind as i32).unwrap();
    }
    w.push(':');
    if let Some(h) = &qos.history {
        write!(w, "{},{}", h.kind as i32, h.depth).unwrap();
    }
    if let Some(ud) = &qos.user_data {
        if !ud.is_empty() {
            write!(w, ":{}", String::from_utf8_lossy(ud)).unwrap();
        }
    }
    w
}

/// Build the liveliness key for a DDS→Zenoh publisher route.
///
/// `zenoh_ke` — the Zenoh key expression (e.g. `chatter` or `robot/chatter`)
/// `ros2_type` — the ROS 2 message type (e.g. `std_msgs/msg/String`)
pub fn build_pub_lv_key(
    zid: &str,
    zenoh_ke: &str,
    ros2_type: &str,
    keyless: bool,
    qos: &Qos,
) -> String {
    format!(
        "@/{zid}/@ros2_lv/MP/{ke}/{typ}/{qos_ke}",
        ke = escape_slashes(zenoh_ke),
        typ = escape_slashes(ros2_type),
        qos_ke = qos_to_lv_str(keyless, qos),
    )
}

/// Build the liveliness key for a Zenoh→DDS subscriber route.
pub fn build_sub_lv_key(
    zid: &str,
    zenoh_ke: &str,
    ros2_type: &str,
    keyless: bool,
    qos: &Qos,
) -> String {
    format!(
        "@/{zid}/@ros2_lv/MS/{ke}/{typ}/{qos_ke}",
        ke = escape_slashes(zenoh_ke),
        typ = escape_slashes(ros2_type),
        qos_ke = qos_to_lv_str(keyless, qos),
    )
}

/// Build the liveliness key for a DDS service server → Zenoh queryable route.
pub fn build_service_srv_lv_key(zid: &str, zenoh_ke: &str, ros2_type: &str) -> String {
    format!(
        "@/{zid}/@ros2_lv/SS/{ke}/{typ}",
        ke = escape_slashes(zenoh_ke),
        typ = escape_slashes(ros2_type),
    )
}

/// Build the liveliness key for a DDS service client → Zenoh querier route.
pub fn build_service_cli_lv_key(zid: &str, zenoh_ke: &str, ros2_type: &str) -> String {
    format!(
        "@/{zid}/@ros2_lv/SC/{ke}/{typ}",
        ke = escape_slashes(zenoh_ke),
        typ = escape_slashes(ros2_type),
    )
}

/// Build the liveliness key for an action server (DDS server → Zenoh queryable).
///
/// Uses the `AS` kind prefix, which the plugin emits for action server entities.
/// `action_ke` must be the base action key expression (without `/_action/...`).
pub fn build_action_srv_lv_key(zid: &str, action_ke: &str, ros2_type: &str) -> String {
    format!(
        "@/{zid}/@ros2_lv/AS/{ke}/{typ}",
        ke = escape_slashes(action_ke),
        typ = escape_slashes(ros2_type),
    )
}

/// Build the liveliness key for an action client (DDS client → Zenoh querier).
///
/// Uses the `AC` kind prefix, mirroring the plugin's action client liveliness format.
/// `action_ke` must be the base action key expression (without `/_action/...`).
pub fn build_action_cli_lv_key(zid: &str, action_ke: &str, ros2_type: &str) -> String {
    format!(
        "@/{zid}/@ros2_lv/AC/{ke}/{typ}",
        ke = escape_slashes(action_ke),
        typ = escape_slashes(ros2_type),
    )
}

/// Extract the base action name from a ROS 2 action component name.
///
/// Action component names have the form `/<action_name>/_action/<component>`.
/// Returns the base action name (e.g. `/fibonacci`) or the input unchanged if
/// `/_action/` is not found.
pub fn action_base_name(ros2_name: &str) -> &str {
    if let Some(pos) = ros2_name.find("/_action/") {
        &ros2_name[..pos]
    } else {
        ros2_name
    }
}

/// Build the bridge self-announcement liveliness key.
///
/// Mirrors the zenoh-plugin-ros2dds plugin token declared at startup.
/// Other bridge instances use this to detect peer bridges and coordinate
/// discovery (e.g. avoiding bridging the same DDS endpoint twice).
pub fn build_bridge_lv_key(zid: &str) -> String {
    format!("@/{zid}/@ros2_lv")
}

#[cfg(test)]
mod tests {
    use cyclors::qos::{
        DDS_INFINITE_TIME, Durability, DurabilityKind, History, HistoryKind, Qos, Reliability,
        ReliabilityKind,
    };

    use super::*;

    fn qos_reliable_transient_local(depth: i32) -> Qos {
        Qos {
            reliability: Some(Reliability {
                kind: ReliabilityKind::RELIABLE,
                max_blocking_time: DDS_INFINITE_TIME,
            }),
            durability: Some(Durability {
                kind: DurabilityKind::TRANSIENT_LOCAL,
            }),
            history: Some(History {
                kind: HistoryKind::KEEP_LAST,
                depth,
            }),
            ..Default::default()
        }
    }

    // ── escape_slashes ────────────────────────────────────────────────────────

    #[test]
    fn test_escape_no_slashes() {
        assert_eq!(escape_slashes("chatter"), "chatter");
    }

    #[test]
    fn test_escape_single_slash() {
        assert_eq!(escape_slashes("my_robot/chatter"), "my_robot§chatter");
    }

    #[test]
    fn test_escape_type_slashes() {
        assert_eq!(escape_slashes("std_msgs/msg/String"), "std_msgs§msg§String");
    }

    // ── qos_to_lv_str ─────────────────────────────────────────────────────────

    #[test]
    fn test_qos_lv_str_empty_qos() {
        // No fields set, keyless → ":::"
        assert_eq!(qos_to_lv_str(true, &Qos::default()), ":::");
    }

    #[test]
    fn test_qos_lv_str_keyed() {
        // !keyless → "K" prefix
        assert_eq!(qos_to_lv_str(false, &Qos::default()), "K:::");
    }

    #[test]
    fn test_qos_lv_str_reliable() {
        let qos = Qos {
            reliability: Some(Reliability {
                kind: ReliabilityKind::RELIABLE,
                max_blocking_time: DDS_INFINITE_TIME,
            }),
            ..Default::default()
        };
        // format: ":reliable_int::"
        let s = qos_to_lv_str(true, &qos);
        assert_eq!(s, format!(":{}::", ReliabilityKind::RELIABLE as i32));
    }

    #[test]
    fn test_qos_lv_str_transient_local() {
        let qos = Qos {
            durability: Some(Durability {
                kind: DurabilityKind::TRANSIENT_LOCAL,
            }),
            ..Default::default()
        };
        let s = qos_to_lv_str(true, &qos);
        assert_eq!(s, format!("::{}:", DurabilityKind::TRANSIENT_LOCAL as i32));
    }

    #[test]
    fn test_qos_lv_str_keep_last() {
        let qos = Qos {
            history: Some(History {
                kind: HistoryKind::KEEP_LAST,
                depth: 3,
            }),
            ..Default::default()
        };
        let s = qos_to_lv_str(true, &qos);
        assert_eq!(s, format!(":::{},3", HistoryKind::KEEP_LAST as i32));
    }

    #[test]
    fn test_qos_lv_str_full_reliable_transient() {
        let qos = qos_reliable_transient_local(10);
        let s = qos_to_lv_str(true, &qos);
        assert!(s.contains(&format!("{}", ReliabilityKind::RELIABLE as i32)));
        assert!(s.contains(&format!("{}", DurabilityKind::TRANSIENT_LOCAL as i32)));
        assert!(s.contains(&format!("{},10", HistoryKind::KEEP_LAST as i32)));
    }

    #[test]
    fn test_qos_lv_str_user_data_appended() {
        let qos = Qos {
            user_data: Some(b"typehash=RIHS01_abc;".to_vec()),
            ..Default::default()
        };
        let s = qos_to_lv_str(true, &qos);
        assert!(s.ends_with(":typehash=RIHS01_abc;"), "got: {s}");
    }

    #[test]
    fn test_qos_lv_str_empty_user_data_omitted() {
        let qos = Qos {
            user_data: Some(vec![]),
            ..Default::default()
        };
        assert_eq!(qos_to_lv_str(true, &qos), ":::");
    }

    // ── key builders ──────────────────────────────────────────────────────────

    #[test]
    fn test_pub_lv_key_format() {
        let key = build_pub_lv_key(
            "myzid",
            "chatter",
            "std_msgs/msg/String",
            true,
            &Qos::default(),
        );
        assert!(key.starts_with("@/myzid/@ros2_lv/MP/"));
        assert!(key.contains("chatter"));
        assert!(key.contains("std_msgs§msg§String"));
    }

    #[test]
    fn test_sub_lv_key_format() {
        let key = build_sub_lv_key(
            "myzid",
            "chatter",
            "std_msgs/msg/String",
            true,
            &Qos::default(),
        );
        assert!(key.starts_with("@/myzid/@ros2_lv/MS/"));
        assert!(key.contains("chatter"));
    }

    #[test]
    fn test_service_srv_lv_key_no_qos_suffix() {
        let key =
            build_service_srv_lv_key("myzid", "add_two_ints", "example_interfaces/srv/AddTwoInts");
        assert!(key.starts_with("@/myzid/@ros2_lv/SS/"));
        assert!(key.contains("example_interfaces§srv§AddTwoInts"));
        // Services have no QoS segment
        assert_eq!(
            key,
            "@/myzid/@ros2_lv/SS/add_two_ints/example_interfaces§srv§AddTwoInts"
        );
    }

    #[test]
    fn test_service_cli_lv_key() {
        let key =
            build_service_cli_lv_key("myzid", "add_two_ints", "example_interfaces/srv/AddTwoInts");
        assert!(key.starts_with("@/myzid/@ros2_lv/SC/"));
    }

    #[test]
    fn test_action_srv_lv_key_format() {
        let key = build_action_srv_lv_key("z", "fibonacci", "example_interfaces/action/Fibonacci");
        assert_eq!(
            key,
            "@/z/@ros2_lv/AS/fibonacci/example_interfaces§action§Fibonacci"
        );
    }

    #[test]
    fn test_action_cli_lv_key_format() {
        let key = build_action_cli_lv_key("z", "fibonacci", "example_interfaces/action/Fibonacci");
        assert_eq!(
            key,
            "@/z/@ros2_lv/AC/fibonacci/example_interfaces§action§Fibonacci"
        );
    }

    #[test]
    fn test_action_base_name_strips_action_suffix() {
        assert_eq!(
            action_base_name("/fibonacci/_action/send_goal"),
            "/fibonacci"
        );
        assert_eq!(
            action_base_name("/my_ns/my_action/_action/get_result"),
            "/my_ns/my_action"
        );
    }

    #[test]
    fn test_action_base_name_no_action_unchanged() {
        assert_eq!(action_base_name("/chatter"), "/chatter");
    }

    #[test]
    fn test_bridge_lv_key_format() {
        let key = build_bridge_lv_key("abc123");
        assert_eq!(key, "@/abc123/@ros2_lv");
    }

    #[test]
    fn test_ke_slashes_escaped_in_pub_key() {
        // Namespaced topic: zenoh_ke has a slash → must be escaped
        let key = build_pub_lv_key(
            "z",
            "robot/chatter",
            "std_msgs/msg/String",
            true,
            &Qos::default(),
        );
        assert!(key.contains("robot§chatter"), "got: {key}");
    }
}
