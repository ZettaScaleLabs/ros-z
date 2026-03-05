//! Convenience re-exports for common ros-z types.
//!
//! Import everything with `use ros_z::prelude::*;` to get the types and traits
//! needed for most use cases without hunting through submodules.
//!
//! # Example
//!
//! ```rust,ignore
//! use ros_z::prelude::*;
//!
//! #[tokio::main]
//! async fn main() -> Result<()> {
//!     let ctx = ZContextBuilder::default().build()?;
//!     let node = ctx.create_node("my_node").build()?;
//!     // ...
//!     Ok(())
//! }
//! ```

/// The builder trait — required to call `.build()` on any builder type.
pub use crate::Builder;

/// The core entry point for creating nodes.
pub use crate::context::ZContextBuilder;

/// QoS configuration types.
pub use crate::qos::{
    QosDurability, QosDuration, QosHistory, QosLiveliness, QosProfile, QosReliability,
};

/// Action type marker trait.
pub use crate::action::ZAction;

/// Trait bounds for custom messages and services.
pub use crate::ros_msg::{ActionTypeInfo, MessageTypeInfo, ServiceTypeInfo, WithTypeInfo};

/// Type identity helpers for custom message definitions.
pub use crate::entity::{TypeHash, TypeInfo};

/// Parameter types for ROS 2-compatible node parameters.
pub use crate::parameter::{
    FloatingPointRange, IntegerRange, Parameter, ParameterDescriptor, ParameterType,
    ParameterValue, SetParametersResult,
};

/// The `Result` alias used throughout ros-z (equivalent to `zenoh::Result`).
pub use zenoh::Result;
