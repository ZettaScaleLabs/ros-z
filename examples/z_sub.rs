use ros_z::{Result, context::ZContext, pubsub::Builder, ros_msg::Vector3D};

fn main() -> Result<()> {
    let ctx = ZContext::new()?;
    let node = ctx.create_node();
    let zsub = node
        .create_sub::<Vector3D>("topic")
        .post_deserialization()
        .build()?;

    loop {
        let msg = zsub.recv()?;
        println!("Recived {msg:?}");
    }
}
