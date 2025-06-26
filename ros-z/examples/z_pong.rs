use ros_z::{Builder, Result, context::ZContextBuilder, ros_msg::ByteMultiArray};

fn main() -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("MyNode").build()?;
    let zsub = node.create_sub::<ByteMultiArray>("ping").build()?;
    let zpub = node.create_pub::<ByteMultiArray>("pong").build()?;

    loop {
        let msg = zsub.recv()?;
        zpub.publish(&msg)?;
    }
}
