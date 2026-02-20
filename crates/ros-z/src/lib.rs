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
