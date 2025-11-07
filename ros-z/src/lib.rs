pub mod context;
pub mod msg;
pub mod node;
pub mod pubsub;
pub mod service;
pub mod ros_msg;
pub mod entity;
pub mod attachment;
pub mod qos;
pub mod graph;

#[macro_use]
pub mod utils;

pub use zenoh::Result;
pub use ros_msg::MessageTypeInfo;


pub trait Builder {
    type Output;
    fn build(self) -> Result<Self::Output>;
}
