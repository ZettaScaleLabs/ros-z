use ros_z::{Result, context::ZContext, pubsub::Builder, ros_msg::Vector3D};
use std::time::Duration;

fn main() -> Result<()> {
    let ctx = ZContext::new()?;
    let node = ctx.create_node();
    let zpub = node.create_pub::<Vector3D>("topic").build()?;

    for idx in 0.. {
        let msg = Vector3D {
            x: idx,
            y: idx,
            z: idx,
        };
        zpub.publish(&msg)?;
        println!("Publish {msg:?}");
        std::thread::sleep(Duration::from_secs(1));
    }
    Ok(())
}
