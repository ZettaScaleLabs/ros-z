//! ros-z-dds — DDS bridging as a first-class ros-z component.
//!
//! Provides `ZDdsPubBridge`, `ZDdsSubBridge`, `ZDdsServiceBridge`, `ZDdsClientBridge`,
//! and `ZDdsBridge` (auto-discovery). The `zenoh-bridge-dds` binary is a thin CLI
//! shell on top of `ZDdsBridge`.

pub mod bridge;
pub mod cyclors;
pub mod discovery;
pub mod ext;
pub mod gid;
pub mod names;
pub mod participant;
pub mod pubsub;
pub mod qos;
pub mod ros_discovery;
pub mod service;
pub mod types;

pub use bridge::ZDdsBridge;
pub use cyclors::CyclorsParticipant;
pub use ext::DdsBridgeExt;
pub use participant::{BridgeQos, DdsParticipant, DdsReader, DdsWriter};
pub use pubsub::{ZDdsPubBridge, ZDdsSubBridge};
pub use ros_discovery::RosDiscoveryPublisher;
pub use service::{ZDdsClientBridge, ZDdsServiceBridge};
