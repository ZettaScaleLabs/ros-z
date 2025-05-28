use ros_z::{
    Builder, Result,
    context::ZContext,
    entity::TypeInfo,
    ros_msg::{Vector3D, srv::AddTwoInts},
};

fn main() -> Result<()> {
    let ctx = ZContext::new()?;
    let node = ctx.create_node("my_node_name").build()?;
    let zcli = node
        .create_client::<AddTwoInts::Service>("add_two_ints")
        // .with_type_info(
        //     TypeInfo::new(
        //     "geometry_msgs::msg::dds_::Vector3_",
        //     "RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d",
        // ))
        .build()?;

    let req = AddTwoInts::Request { a: 1, b: 2 };
    zcli.send_request(&req)?;
    let resp = zcli.take_response()?;
    println!("{resp:?}");
    Ok(())
}
