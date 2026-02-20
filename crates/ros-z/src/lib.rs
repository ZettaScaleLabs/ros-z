//! # ros-z — Zenoh-native ROS 2 in pure Rust
//!
//! `ros-z` provides ROS 2-style pub/sub, services, and actions built directly
//! on [Zenoh](https://zenoh.io), with no C/C++ dependencies.
//!
//! ## Getting started
//!
//! ```rust,ignore
//! use ros_z::prelude::*;
//! use ros_z_msgs::std_msgs::String as RosString;
//!
//! let ctx = ZContextBuilder::default().build()?;
//! let node = ctx.create_node("talker").build()?;
//! let publisher = node.create_pub::<RosString>("/chatter").build()?;
//! publisher.async_publish(&RosString { data: "hello".into() }).await?;
//! ```
//!
//! ## Sync and async APIs
//!
//! Most ros-z types expose both a blocking and an async variant of each
//! operation. The naming convention is:
//!
//! | Suffix | Behaviour |
//! |--------|-----------|
//! | *(none)* | Blocking — safe to call from any context |
//! | `_async` | Async — must be `.await`ed inside a Tokio (or compatible) runtime |
//!
//! For example, [`ZPub::publish`](pubsub::ZPub::publish) blocks until
//! the put completes, while [`ZPub::async_publish`](pubsub::ZPub::async_publish)
//! yields to the async executor.
//!
//! ## Imports
//!
//! The easiest way to import all common types is via the prelude:
//!
//! ```rust,ignore
//! use ros_z::prelude::*;
//! ```
//!
//! Or import types individually from their modules.

pub mod action;
pub mod attachment;
mod common;
pub mod config;
pub mod context;
pub mod dynamic;
pub mod encoding;
pub mod entity;
pub mod event;
pub mod graph;
pub mod msg;
pub mod node;
pub mod prelude;
pub mod pubsub;
pub mod python_bridge;
pub mod qos;
pub mod queue;
pub mod ros_msg;
pub mod service;
pub mod shm;
pub mod topic_name;
pub mod zbuf;

#[cfg(feature = "python")]
pub mod zbuf_view;

#[macro_use]
pub mod utils;

pub use attachment::GidArray;
pub use entity::{TypeHash, TypeInfo};
pub use ros_msg::{ActionTypeInfo, MessageTypeInfo, ServiceTypeInfo, WithTypeInfo};
pub use zbuf::ZBuf;
pub use zenoh::Result;

/// Builds a configured object, consuming the builder.
///
/// All ros-z builders implement this trait. Bring it into scope to call `.build()`:
///
/// ```rust,ignore
/// use ros_z::Builder;
/// let ctx = ZContextBuilder::default().build()?;
/// ```
///
/// Alternatively, use `use ros_z::prelude::*;` which includes `Builder`.
pub trait Builder {
    /// The type produced by this builder.
    type Output;
    /// Consume the builder and construct the configured object.
    ///
    /// # Errors
    ///
    /// Returns an error if the configuration is invalid or if network/resource
    /// initialization fails (e.g. Zenoh session could not be opened).
    fn build(self) -> Result<Self::Output>;
}
