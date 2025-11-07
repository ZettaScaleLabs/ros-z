// This module re-exports auto-generated ROS message types
// The actual code is generated at build time and included from OUT_DIR

// Include the generated code
include!(concat!(env!("OUT_DIR"), "/generated.rs"));

// Re-export commonly used items from ros-z for convenience
pub use ros_z::MessageTypeInfo;
pub use ros_z::entity::{TypeInfo, TypeHash};
