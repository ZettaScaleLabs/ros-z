use ros_z::MessageTypeInfo;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, MessageTypeInfo)]
#[ros_msg(type_name = "custom_msgs/msg/OptionField")]
struct OptionField {
    value: Option<u32>,
}

fn main() {}
