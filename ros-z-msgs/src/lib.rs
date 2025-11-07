// This module re-exports auto-generated ROS message types
// The actual code is generated at build time and included from OUT_DIR

// Include the CDR-compatible generated code (serde-based)
include!(concat!(env!("OUT_DIR"), "/generated.rs"));

// Include the protobuf-compatible generated code (prost-based) if feature is enabled
#[cfg(feature = "protobuf")]
pub mod proto {
    //! Protobuf-serializable message types
    //!
    //! These types are compatible with `ros_z::msg::ProtobufSerdes` and can be used
    //! for protobuf serialization instead of CDR.
    //!
    //! # Example
    //! ```no_run
    //! use ros_z_msgs::proto::geometry_msgs::Vector3;
    //! use ros_z::msg::ProtobufSerdes;
    //!
    //! let vec = Vector3 { x: 1.0, y: 2.0, z: 3.0 };
    //! // Use with .with_serdes::<ProtobufSerdes<Vector3>>()
    //! ```

    include!(concat!(env!("OUT_DIR"), "/generated_proto.rs"));
}

// Re-export commonly used items from ros-z for convenience
pub use ros_z::MessageTypeInfo;
pub use ros_z::entity::{TypeInfo, TypeHash};
