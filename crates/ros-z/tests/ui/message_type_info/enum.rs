use ros_z::MessageTypeInfo;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, MessageTypeInfo)]
#[ros_msg(type_name = "custom_msgs/msg/RobotState")]
enum RobotState {
    Idle,
    Error(String),
}

fn main() {}
