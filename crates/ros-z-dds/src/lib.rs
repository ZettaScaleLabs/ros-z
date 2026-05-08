//! ros-z-dds — DDS bridging as a first-class ros-z component.
//!
//! Provides `ZDdsPubBridge`, `ZDdsSubBridge`, `ZDdsServiceBridge`, `ZDdsClientBridge`,
//! and `ZDdsBridge` (auto-discovery). The `ros-z-bridge-dds` binary is a thin CLI
//! shell on top of `ZDdsBridge`.

pub mod bridge;
pub(crate) mod cyclors;
pub(crate) mod discovery;
pub mod ext;
pub(crate) mod gid;
pub(crate) mod names;
pub mod participant;
pub mod pubsub;
pub(crate) mod qos;
pub(crate) mod ros_discovery;
pub mod service;

pub use bridge::ZDdsBridge;
pub use cyclors::CyclorsParticipant;
pub use ext::DdsBridgeExt;
pub use participant::{BridgeQos, DdsParticipant};
pub use pubsub::{ZDdsPubBridge, ZDdsSubBridge};
pub use service::{ZDdsClientBridge, ZDdsServiceBridge};
