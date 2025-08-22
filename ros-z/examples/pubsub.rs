use std::time::Duration;

use ros_z::{Builder, Result, context::ZContextBuilder, ros_msg::ByteMultiArray};

use clap::Parser;
#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "20")]
    payload: usize,
    #[arg(short, long, default_value = "1")]
    rate: usize,
    // #[arg(short, long, default_value = "")]
    // config: String,
    #[arg(short, long, default_value = "default_topic")]
    topic: String,
    #[arg(short, long, default_value = "3")]
    duration: f64,
    #[arg(short, long, default_value = "sub")]
    mode: String,
}

fn run_subscriber(topic: String, duration: Duration) -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("MyNode").build()?;
    let zsub = node.create_sub::<ByteMultiArray>(&topic).build()?;

    if duration.is_zero() {
        loop {
            let msg = zsub.recv()?;
            tracing::info!("Recv {msg:?}");
        }
    } else {
        let fut = async {
            loop {
                match zsub.async_recv().await {
                    Ok(msg) => {
                        tracing::info!("Recv {msg:?}");
                    }
                    Err(err) => {
                        tracing::error!(err);
                        break;
                    }
                }
            }
        };
        tokio::runtime::Runtime::new()?.block_on(async {
            let _ = tokio::time::timeout(duration, fut).await?;
            Result::Ok(())
        })?;
    }

    Ok(())
}

fn run_publisher(
    topic: String,
    duration: Duration,
    period: Duration,
    payload_size: usize,
) -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("MyNode").build()?;
    let zpub = node.create_pub::<ByteMultiArray>(&topic).build()?;
    let now = std::time::Instant::now();
    let mut msg = ros_z::ros_msg::ByteMultiArray::default();
    msg.data = vec![0u8; payload_size as _];
    loop {
        zpub.publish(&msg)?;
        std::thread::sleep(period);
        if !duration.is_zero() && now.elapsed() >= duration {
            break;
        }
    }
    Ok(())
}

fn main() -> Result<()> {
    let args = Args::parse();
    let duration = std::time::Duration::from_secs_f64(args.duration);
    let period = std::time::Duration::from_secs_f64(1.0 / args.rate as f64);
    zenoh::init_log_from_env_or("error");
    tracing::info!(
        "Begin testing with topic: {}, duration: {duration:?}, payload: {}, rate: {} Hz",
        args.topic,
        args.payload,
        args.rate
    );
    if args.mode == "sub" {
        run_subscriber(args.topic, duration)?;
    } else if args.mode == "pub" {
        run_publisher(args.topic, duration, period, args.payload)?;
    } else {
        tracing::error!("The mode {} is not supported.", args.mode);
    }
    Ok(())
}
