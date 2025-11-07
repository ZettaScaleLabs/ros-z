use ros_z::{Result, Builder, context::ZContextBuilder, entity::{TypeInfo, TypeHash}, ros_msg::Vector3D};

fn main() -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("my_node_name").build()?;
    let zsub = node
        .create_sub::<Vector3D>("vector")
        .with_type_info(TypeInfo::new(
            "geometry_msgs::msg::dds_::Vector3_",
            TypeHash::from_rihs_string("RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d").unwrap(),
        ))
        .build()?;

    loop {
        let msg = zsub.recv()?;
        println!("Recived {msg:?}");
    }
}
