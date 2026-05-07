/// Action routing.
///
/// ROS 2 actions are implemented on top of services and topics:
/// - `/<name>/_action/send_goal`   (service)
/// - `/<name>/_action/cancel_goal` (service)
/// - `/<name>/_action/get_result`  (service)
/// - `/<name>/_action/feedback`    (topic)
/// - `/<name>/_action/status`      (topic)
///
/// The bridge delegates action components to `ServiceRoute` and pub/sub routes.
/// No separate action-specific route type is needed; the standard service and pubsub
/// routing handles each component individually.

pub fn is_action_component(ros2_name: &str) -> bool {
    ros2_name.contains("/_action/")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_action_component_send_goal() {
        assert!(is_action_component("/fibonacci/_action/send_goal"));
    }

    #[test]
    fn test_is_action_component_get_result() {
        assert!(is_action_component("/fibonacci/_action/get_result"));
    }

    #[test]
    fn test_is_action_component_cancel_goal() {
        assert!(is_action_component("/fibonacci/_action/cancel_goal"));
    }

    #[test]
    fn test_is_action_component_feedback() {
        assert!(is_action_component("/fibonacci/_action/feedback"));
    }

    #[test]
    fn test_is_action_component_status() {
        assert!(is_action_component("/fibonacci/_action/status"));
    }

    #[test]
    fn test_is_action_component_plain_topic() {
        assert!(!is_action_component("/chatter"));
    }

    #[test]
    fn test_is_action_component_plain_service() {
        assert!(!is_action_component("/add_two_ints"));
    }

    #[test]
    fn test_is_action_component_namespaced_action() {
        assert!(is_action_component("/my_ns/my_action/_action/send_goal"));
    }

    #[test]
    fn test_is_action_component_dds_topic_prefix_with_action() {
        // DDS topics start with rt/rq/rr; the bridge checks the ros2 name, not dds name.
        assert!(is_action_component("rq/fibonacci/_action/send_goalRequest"));
    }
}
