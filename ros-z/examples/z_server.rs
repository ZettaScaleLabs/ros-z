use ros_z::{Builder, Result, context::ZContextBuilder, ros_msg::srv::AddTwoInts};

fn main() -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("my_node_name").build()?;
    let mut zsrv = node
        .create_service::<AddTwoInts::Service>("add_two_ints")
        // .with_type_info(
        //     TypeInfo::new(
        //     "geometry_msgs::msg::dds_::Vector3_",
        //     "RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d",
        // ))
        .build()?;

    let (key, req) = zsrv.take_request()?;
    let mut resp = AddTwoInts::Response::default();
    resp.sum = req.a + req.b;
    zsrv.send_response(&resp, &key)?;
    Ok(())
}
