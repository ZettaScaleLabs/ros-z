use std::time::Duration;

use ros_z::{
    Builder, Result,
    context::{ZContext, ZContextBuilder},
};
use ros_z_msgs::std_msgs::String as RosString;

async fn run_subscriber(ctx: ZContext, topic: String) -> Result<()> {
    let node = ctx.create_node("Sub").build()?;
    let zsub = node.create_sub::<RosString>(&topic).build()?;
    while let Ok(msg) = zsub.async_recv().await {
        println!("Hearing:>> {}", msg.data);
    }
    Ok(())
}

async fn run_publisher(
    ctx: ZContext,
    topic: String,
    period: Duration,
    payload: String,
) -> Result<()> {
    let node = ctx.create_node("Pub").build()?;
    let zpub = node.create_pub::<RosString>(&topic).build()?;

    let mut count = 0;
    loop {
        let str = RosString {
            data: format!("{payload} - #{count}"),
        };
        println!("Telling:>> {}", str.data);

        zpub.async_publish(&str).await?;
        let _ = tokio::time::sleep(period).await;
        count += 1;
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let ctx = if let Some(e) = args.endpoint {
        ZContextBuilder::default()
            .with_mode(args.mode)
            .with_connect_endpoints([e])
            .build()?
    } else {
        ZContextBuilder::default().with_mode(args.mode).build()?
    };

    let period = std::time::Duration::from_secs_f64(args.period);
    zenoh::init_log_from_env_or("error");
    if args.role == "listener" {
        run_subscriber(ctx, args.topic).await?;
    } else if args.role == "talker" {
        run_publisher(ctx, args.topic, period, args.data).await?;
    } else {
        println!(
            "Please use \"talker\" or \"listener\" as role,  {} is not supported.",
            args.role
        );
    }
    Ok(())
}

use clap::Parser;
#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "Hello ROS-Z")]
    data: String,
    #[arg(short, long, default_value = "/chatter")]
    topic: String,
    #[arg(short, long, default_value = "1.0")]
    period: f64,
    #[arg(short, long, default_value = "listener")]
    role: String,
    #[arg(short, long, default_value = "peer")]
    mode: String,
    #[arg(short, long)]
    endpoint: Option<String>,
}
