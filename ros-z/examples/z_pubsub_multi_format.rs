use std::time::Duration;

use ros_z::{Builder, Result, context::ZContextBuilder};
use ros_z_msgs::geometry_msgs::Vector3;
use ros_z::MessageTypeInfo;

#[cfg(feature = "protobuf")]
use ros_z::msg::ProtobufSerdes;

use clap::Parser;

#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "1")]
    rate: usize,
    #[arg(short, long, default_value = "default_topic")]
    topic: String,
    #[arg(short, long, default_value = "0")]
    duration: f64,
    #[arg(short, long, default_value = "sub")]
    mode: String,
    #[arg(short, long, default_value = "cdr")]
    format: String,
}

fn run_subscriber_cdr(topic: String, duration: Duration) -> Result<()> {
    println!("Starting subscriber with CDR serialization on topic: {}", topic);
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("cdr_subscriber").build()?;

    // Create subscriber with automatic type info from MessageTypeInfo trait
    let zsub = node.create_sub::<Vector3>(&topic)
        .with_type_info(Vector3::type_info())
        .build()?;

    let mut counter = 0;
    if duration.is_zero() {
        loop {
            let msg = zsub.recv()?;
            tracing::info!("CDR Recv {counter}-th msg: x={}, y={}, z={}", msg.r#x, msg.r#y, msg.r#z);
            counter += 1;
        }
    } else {
        let fut = async {
            loop {
                match zsub.async_recv().await {
                    Ok(msg) => {
                        tracing::info!("CDR Recv {counter}-th msg: x={}, y={}, z={}", msg.r#x, msg.r#y, msg.r#z);
                        counter += 1;
                    }
                    Err(err) => {
                        tracing::error!("Error: {err}");
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

fn run_publisher_cdr(
    topic: String,
    duration: Duration,
    period: Duration,
) -> Result<()> {
    println!("Starting publisher with CDR serialization on topic: {}", topic);
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("cdr_publisher").build()?;

    // Create publisher with automatic type info from MessageTypeInfo trait
    let zpub = node.create_pub::<Vector3>(&topic)
        .with_type_info(Vector3::type_info())
        .build()?;

    let now = std::time::Instant::now();
    let mut counter = 0;

    loop {
        let msg = Vector3 {
            r#x: counter as f64,
            r#y: (counter as f64) * 2.0,
            r#z: (counter as f64) * 3.0,
        };

        zpub.publish(&msg)?;
        tracing::info!("CDR Sent msg #{counter}: {:?}", msg);

        counter += 1;
        std::thread::sleep(period);

        if !duration.is_zero() && now.elapsed() >= duration {
            break;
        }
    }
    Ok(())
}

#[cfg(feature = "protobuf")]
fn run_subscriber_protobuf(_topic: String, _duration: Duration) -> Result<()> {
    println!("Protobuf feature is enabled, but the example message types don't have protobuf support yet.");
    println!("To add protobuf support, you would:");
    println!("1. Define .proto files for your messages");
    println!("2. Use prost to generate Rust types");
    println!("3. Implement MessageTypeInfo for those types");
    println!("4. Use .with_serdes::<ProtobufSerdes<T>>() when creating publishers/subscribers");
    Ok(())
}

#[cfg(feature = "protobuf")]
fn run_publisher_protobuf(_topic: String, _duration: Duration, _period: Duration) -> Result<()> {
    println!("Protobuf feature is enabled, but the example message types don't have protobuf support yet.");
    println!("See run_subscriber_protobuf for more details.");
    Ok(())
}

fn main() -> Result<()> {
    let args = Args::parse();
    let duration = std::time::Duration::from_secs_f64(args.duration);
    let period = std::time::Duration::from_secs_f64(1.0 / args.rate as f64);
    zenoh::init_log_from_env_or("info");

    println!("\n=== ROS-Z Multi-Format Pub/Sub Example ===");
    println!("Topic: {}", args.topic);
    println!("Duration: {:?}", duration);
    println!("Rate: {} Hz", args.rate);
    println!("Format: {}", args.format);
    println!("==========================================\n");

    match (args.mode.as_str(), args.format.as_str()) {
        ("sub", "cdr") => run_subscriber_cdr(args.topic, duration)?,
        ("pub", "cdr") => run_publisher_cdr(args.topic, duration, period)?,
        #[cfg(feature = "protobuf")]
        ("sub", "protobuf") => run_subscriber_protobuf(args.topic, duration)?,
        #[cfg(feature = "protobuf")]
        ("pub", "protobuf") => run_publisher_protobuf(args.topic, duration, period)?,
        #[cfg(not(feature = "protobuf"))]
        (_, "protobuf") => {
            println!("Protobuf format requested but protobuf feature is not enabled.");
            println!("Rebuild with: cargo build --features protobuf");
        }
        _ => {
            tracing::error!("Unsupported mode/format combination: {}/{}", args.mode, args.format);
            println!("\nSupported combinations:");
            println!("  --mode pub --format cdr");
            println!("  --mode sub --format cdr");
            #[cfg(feature = "protobuf")]
            {
                println!("  --mode pub --format protobuf");
                println!("  --mode sub --format protobuf");
            }
        }
    }

    Ok(())
}
