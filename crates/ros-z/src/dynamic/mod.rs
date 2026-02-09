//! Dynamic message support for ROS-Z.
//!
//! This module provides runtime message handling where message types are
//! determined at runtime rather than compile time. This is useful for:
//!
//! - Generic tools that work with any message type (rosbag, echo, etc.)
//! - Bridging between different ROS 2 systems
//! - Dynamic message inspection and modification
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────┐     ┌─────────────────┐
//! │  MessageSchema  │────▶│   FieldSchema   │
//! │  (type info)    │     │   (field info)  │
//! └────────┬────────┘     └────────┬────────┘
//!          │                       │
//!          ▼                       ▼
//! ┌─────────────────┐     ┌─────────────────┐
//! │ DynamicMessage  │────▶│  DynamicValue   │
//! │   (container)   │     │    (values)     │
//! └────────┬────────┘     └─────────────────┘
//!          │
//!          ▼
//! ┌─────────────────┐
//! │  CDR Serialize  │
//! │  /Deserialize   │
//! └─────────────────┘
//! ```
//!
//! # Example
//!
//! ```rust,ignore
//! use ros_z::dynamic::{MessageSchema, DynamicMessage, FieldType};
//!
//! // Create a schema for geometry_msgs/msg/Point
//! let schema = MessageSchema::builder("geometry_msgs/msg/Point")
//!     .field("x", FieldType::Float64)
//!     .field("y", FieldType::Float64)
//!     .field("z", FieldType::Float64)
//!     .build()?;
//!
//! // Create and populate a message
//! let mut msg = DynamicMessage::new(&schema);
//! msg.set("x", 1.0f64)?;
//! msg.set("y", 2.0f64)?;
//! msg.set("z", 3.0f64)?;
//!
//! // Serialize to CDR
//! let bytes = msg.to_cdr()?;
//!
//! // Deserialize
//! let decoded = DynamicMessage::from_cdr(&bytes, &schema)?;
//! assert_eq!(decoded.get::<f64>("x")?, 1.0);
//! ```

pub mod error;
pub mod message;
pub mod registry;
pub mod schema;
pub mod serdes;
pub mod serialization;
pub mod value;

#[cfg(test)]
mod tests;

// Re-export main types
pub use error::DynamicError;
pub use message::{DynamicMessage, DynamicMessageBuilder};
pub use registry::{SchemaRegistry, get_schema, has_schema, register_schema};
pub use schema::{FieldSchema, FieldType, MessageSchema, MessageSchemaBuilder};
pub use serdes::DynamicCdrSerdes;
pub use serialization::SerializationFormat;
pub use value::{DynamicValue, FromDynamic, IntoDynamic};

use zenoh::sample::Sample;

use crate::msg::ZMessage;
use crate::pubsub::{ZPub, ZPubBuilder, ZSub, ZSubBuilder};

// Implement ZMessage for DynamicMessage
impl ZMessage for DynamicMessage {
    type Serdes = DynamicCdrSerdes;
}

// Type aliases for convenience
/// Type alias for a dynamic message publisher.
pub type DynPub = ZPub<DynamicMessage, DynamicCdrSerdes>;

/// Type alias for a dynamic message subscriber.
pub type DynSub = ZSub<DynamicMessage, Sample, DynamicCdrSerdes>;

/// Type alias for a dynamic message publisher builder.
pub type DynPubBuilder = ZPubBuilder<DynamicMessage, DynamicCdrSerdes>;

/// Type alias for a dynamic message subscriber builder.
pub type DynSubBuilder = ZSubBuilder<DynamicMessage, DynamicCdrSerdes>;
