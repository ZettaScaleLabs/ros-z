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

mod utils;

pub use zenoh::Result;


pub trait Builder {
    type Output;
    fn build(self) -> Result<Self::Output>;
}
