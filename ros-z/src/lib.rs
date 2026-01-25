pub mod action;
pub mod attachment;
mod common;
pub mod config;
pub mod context;
pub mod dynamic;
pub mod entity;
pub mod event;
pub mod graph;
pub mod msg;
pub mod node;
pub mod pubsub;
pub mod python_bridge;
pub mod qos;
pub mod queue;
pub mod ros_msg;
pub mod service;
pub mod topic_name;

#[macro_use]
pub mod utils;

pub use attachment::GidArray;
pub use entity::{TypeHash, TypeInfo};
pub use ros_msg::{ActionTypeInfo, MessageTypeInfo, ServiceTypeInfo, WithTypeInfo};
pub use zenoh::Result;

pub trait Builder {
    type Output;
    fn build(self) -> Result<Self::Output>;
}
