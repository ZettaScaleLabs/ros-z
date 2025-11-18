use std::time::Duration;

use ros_z::{Builder, Result, context::ZContextBuilder};
use ros_z_msgs::std_msgs::ByteMultiArray;

async fn run_subscriber(topic: String) -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("Sub").build()?;
    let zsub = node.create_sub::<ByteMultiArray>(&topic).build()?;
    let mut counter = 1;
    while let Ok(msg) = zsub.async_recv().await {
        println!("sub:>> #{counter}: {}", String::from_utf8_lossy(&msg.data));
        counter += 1;
    }
    Ok(())
}

async fn run_publisher(topic: String, period: Duration, payload: String) -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("Pub").build()?;
    let zpub = node.create_pub::<ByteMultiArray>(&topic).build()?;

    let mut count = 0;
    loop {
        let bs = format!("{payload} - #{count}");
        println!("pub:>> {bs}");
        let msg = ByteMultiArray {
            data: bs.into(),
            ..Default::default()
        };
        zpub.async_publish(&msg).await?;
        let _ = tokio::time::sleep(period).await;
        count += 1;
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let period = std::time::Duration::from_secs_f64(args.period);
    zenoh::init_log_from_env_or("error");
    if args.mode == "sub" {
        run_subscriber(args.topic).await?;
    } else if args.mode == "pub" {
        run_publisher(args.topic, period, args.data).await?;
    } else {
        println!(
            "Please use \"pub\" or \"sub\" as mode,  {} is not supported.",
            args.mode
        );
    }
    Ok(())
}

use clap::Parser;
#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "Hello ROS-Z")]
    data: String,
    #[arg(short, long, default_value = "rosz/hello")]
    topic: String,
    #[arg(short, long, default_value = "1.0")]
    period: f64,
    #[arg(short, long, default_value = "sub")]
    mode: String,
}
