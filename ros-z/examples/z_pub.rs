use std::time::Duration;

use ros_z::{Builder, Result, context::ZContextBuilder, entity::TypeInfo, ros_msg::Vector3D};

// 0/vector/geometry_msgs::msg::dds_::Vector3_/RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d

fn main() -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("my_node_name").build()?;
    let zpub = node
        .create_pub::<Vector3D>("vector")
        .with_type_info(TypeInfo::new(
            "geometry_msgs::msg::dds_::Vector3_",
            "RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d",
        ))
        .build()?;

    for idx in 0.. {
        let v = idx as _;
        let msg = Vector3D { x: v, y: v, z: v };
        zpub.publish(&msg)?;
        println!("Publish {msg:?}");
        std::thread::sleep(Duration::from_secs(1));
    }
    Ok(())
}
