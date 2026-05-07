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
