use std::time::Duration;

use ros_z::{
    Builder, Result,
    context::ZContextBuilder,
    parameter::{Parameter, ParameterDescriptor, ParameterTarget, ParameterType, ParameterValue},
};

#[tokio::main]
async fn main() -> Result<()> {
    zenoh::init_log_from_env_or("error");
    let ctx = ZContextBuilder::default().build()?;

    println!("\n=== Remote Parameter Client Demo ===\n");

    let server = ctx.create_node("client_demo_server").build()?;
    let desc = ParameterDescriptor::new("max_speed", ParameterType::Integer);
    server
        .declare_parameter("max_speed", ParameterValue::Integer(50), desc)
        .expect("declare max_speed");

    let client_node = ctx.create_node("client_demo").build()?;
    let client = client_node
        .create_parameter_client(
            ParameterTarget::from_fqn("/client_demo_server").expect("valid node name"),
        )
        .build()?;

    tokio::time::sleep(Duration::from_millis(500)).await;

    let values = client.get(&["max_speed"]).await?;
    let types = client.get_types(&["max_speed"]).await?;
    let result = client
        .set_atomically(&[Parameter::new("max_speed", ParameterValue::Integer(80))])
        .await?;

    println!("remote values = {:?}", values);
    println!("remote types = {:?}", types);
    println!("remote atomic set = {:?}", result);
    println!(
        "max_speed after remote set = {:?}",
        server.get_parameter("max_speed")
    );

    Ok(())
}
