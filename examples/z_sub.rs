use ros_z::{context::ZContext, Result, ros_msg::Vector3D};
use std::time::Duration;


// fn main() -> Result<()> {
//     let ctx = ZContext::new()?;
//     let node = ctx.create_node();
//     let zsub = node.create_sub::<Vector3D>("topic")?;
//
//     loop {
//         let msg = zsub.recv()?;
//         println!("Recived {msg:?}");
//     }
//     Ok(())
// }

fn main() -> Result<()> {
    let ctx = ZContext::new()?;
    let node = ctx.create_node();
    let zsub = node.create_sub_predes::<Vector3D>("topic")?;

    loop {
        let msg = zsub.recv()?;
        println!("Recived {msg:?}");
    }
    Ok(())
}
