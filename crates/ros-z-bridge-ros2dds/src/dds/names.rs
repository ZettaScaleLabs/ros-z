/// Convert a DDS topic name to a ROS 2 topic/service name.
///
/// DDS uses prefixes: `rt/` (topic), `rq/` (request), `rr/` (reply).
/// Returns `None` for unrecognised prefixes (e.g. built-in topics).
pub fn dds_topic_to_ros2_name(dds_topic: &str) -> Option<String> {
    // Topics: "rt/<ros_name>" → "/<ros_name>"
    if let Some(rest) = dds_topic.strip_prefix("rt/") {
        return Some(format!("/{rest}"));
    }
    // Service requests: "rq/<name>Request" → "/<name>"
    if let Some(rest) = dds_topic.strip_prefix("rq/") {
        if let Some(name) = rest.strip_suffix("Request") {
            return Some(format!("/{name}"));
        }
    }
    // Service replies: "rr/<name>Reply" → "/<name>"
    if let Some(rest) = dds_topic.strip_prefix("rr/") {
        if let Some(name) = rest.strip_suffix("Reply") {
            return Some(format!("/{name}"));
        }
    }
    None
}

/// True if the DDS topic belongs to a ROS 2 service request channel.
pub fn is_request_topic(dds_topic: &str) -> bool {
    dds_topic.starts_with("rq/")
}

/// True if the DDS topic belongs to a ROS 2 service reply channel.
pub fn is_reply_topic(dds_topic: &str) -> bool {
    dds_topic.starts_with("rr/")
}

/// True if the DDS topic is a regular pub/sub topic.
pub fn is_pubsub_topic(dds_topic: &str) -> bool {
    dds_topic.starts_with("rt/")
}

/// Convert a DDS type string (e.g. `std_msgs::msg::dds_::String_`) to a ROS 2 type
/// (e.g. `std_msgs/msg/String`).
pub fn dds_type_to_ros2_type(dds_type: &str) -> String {
    let result = dds_type.replace("::dds_::", "::").replace("::", "/");
    if result.ends_with('_') {
        result[..result.len() - 1].into()
    } else {
        result
    }
}

/// Convert a ROS 2 type string to a DDS type string.
pub fn ros2_type_to_dds_type(ros2_type: &str) -> String {
    let mut result = ros2_type.replace('/', "::");
    if let Some(pos) = result.rfind(':') {
        result.insert_str(pos + 1, "dds_::");
    }
    result.push('_');
    result
}

/// Strip the service suffix (`_Request_` / `_Response_`) and convert to ROS 2 type.
pub fn dds_type_to_ros2_service_type(dds_type: &str) -> String {
    dds_type_to_ros2_type(
        dds_type
            .strip_suffix("_Request_")
            .or(dds_type.strip_suffix("_Response_"))
            .unwrap_or(dds_type),
    )
}

/// Build the Zenoh key expression for a ROS 2 topic/service name.
///
/// Strips the leading `/` since Zenoh key expressions must not start with one.
/// If `namespace` is set (e.g. `"my_robot"`), it is prepended: `my_robot/chatter`.
pub fn ros2_name_to_zenoh_key(ros2_name: &str, namespace: Option<&str>) -> String {
    let stripped = ros2_name.strip_prefix('/').unwrap_or(ros2_name);
    match namespace {
        Some(ns) if !ns.is_empty() && ns != "/" => {
            let ns = ns.strip_prefix('/').unwrap_or(ns);
            format!("{ns}/{stripped}")
        }
        _ => stripped.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dds_topic_conversions() {
        assert_eq!(
            dds_topic_to_ros2_name("rt/chatter"),
            Some("/chatter".into())
        );
        assert_eq!(
            dds_topic_to_ros2_name("rq/add_two_intsRequest"),
            Some("/add_two_ints".into())
        );
        assert_eq!(
            dds_topic_to_ros2_name("rr/add_two_intsReply"),
            Some("/add_two_ints".into())
        );
        assert_eq!(dds_topic_to_ros2_name("DCPS_something"), None);
    }

    #[test]
    fn test_type_conversions() {
        assert_eq!(
            dds_type_to_ros2_type("std_msgs::msg::dds_::String_"),
            "std_msgs/msg/String"
        );
        assert_eq!(
            ros2_type_to_dds_type("std_msgs/msg/String"),
            "std_msgs::msg::dds_::String_"
        );
        assert_eq!(
            dds_type_to_ros2_service_type("example_interfaces::srv::dds_::AddTwoInts_Request_"),
            "example_interfaces/srv/AddTwoInts"
        );
    }

    #[test]
    fn test_zenoh_key() {
        assert_eq!(ros2_name_to_zenoh_key("/chatter", None), "chatter");
        assert_eq!(
            ros2_name_to_zenoh_key("/chatter", Some("robot")),
            "robot/chatter"
        );
        assert_eq!(ros2_name_to_zenoh_key("/chatter", Some("/")), "chatter");
    }
}
