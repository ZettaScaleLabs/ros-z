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

pub trait Builder {
    type Output;
    fn build(self) -> Result<Self::Output>;
}
