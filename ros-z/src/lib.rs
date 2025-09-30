pub mod attachment;
pub mod context;
pub mod entity;
pub mod graph;
pub mod msg;
pub mod node;
pub mod pubsub;
pub mod qos;
pub mod ros_msg;
pub mod service;

mod utils;

pub use zenoh::Result;

pub trait Builder {
    type Output;
    fn build(self) -> Result<Self::Output>;
}
