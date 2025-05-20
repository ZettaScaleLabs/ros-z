use ros_z::{Builder, Result, context::ZContext, ros_msg::Vector3D, entity::TypeInfo};

fn main() -> Result<()> {
    let ctx = ZContext::new()?;
    let node = ctx.create_node("my_node_name").build()?;
    let zsub = node
        .create_sub::<Vector3D>("vector")
        .with_type_info(TypeInfo::new(
            "geometry_msgs::msg::dds_::Vector3_",
            "RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d",
        ))
        // .post_deserialization()
        .build()?;

    loop {
        let msg = zsub.recv()?;
        println!("Recived {msg:?}");
    }
}
