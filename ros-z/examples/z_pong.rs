use ros_z::{Result, context::ZContext, Builder, ros_msg::ByteMultiArray};

fn main() -> Result<()> {
    let ctx = ZContext::new()?;
    let node = ctx.create_node("MyNode").build()?;
    let zsub = node.create_sub::<ByteMultiArray>("ping").build()?;
    let zpub = node.create_pub::<ByteMultiArray>("pong").build()?;

    loop {
        let msg = zsub.recv()?;
        zpub.publish(&msg)?;
    }
}
