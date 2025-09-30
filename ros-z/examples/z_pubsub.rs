use std::time::Duration;

use ros_z::{
    Builder, Result,
    context::ZContextBuilder,
    entity::{TypeHash, TypeInfo},
    msg::{ProtobufSerdes, ZMessage},
    ros_msg::ByteMultiArray,
};

mod example {
    include!(concat!(env!("OUT_DIR"), "/example.rs"));
}

// Users can choose the serdes at the pub/sub creation time using:
//   - node.create_pub::<MyType>(&topic) uses MyType::Serdes (default)
//   - node.create_pub_with_serdes::<MyType, CdrSerdes<_>>(&topic) for CDR
//   - node.create_pub_with_serdes::<MyType, ProtobufSerdes<_>>(&topic) for Protobuf
impl ZMessage for example::Entity {
    type Serdes = ProtobufSerdes<Self>; // Default
}

use clap::{Parser, ValueEnum};

#[derive(Debug, Clone, ValueEnum)]
enum SerdesType {
    Cdr,
    Protobuf,
}

#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "64")]
    payload: usize,
    #[arg(short, long, default_value = "1")]
    rate: usize,
    // #[arg(short, long, default_value = "")]
    // config: String,
    #[arg(short, long, default_value = "default_topic")]
    topic: String,
    #[arg(short, long, default_value = "0")]
    duration: f64,
    #[arg(short, long, default_value = "sub")]
    mode: String,
    #[arg(long, default_value = "cdr")]
    serdes: SerdesType,
}

fn run_cdr_subscriber(topic: String, duration: Duration) -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("MyNode").build()?;
    let zsub = node
        .create_sub::<ByteMultiArray>(&topic)
        .with_type_info(TypeInfo::new(
            "std_msgs::msg::dds_::UInt8MultiArray_",
            TypeHash::from_rihs_string(
                "RIHS01_5687e861b8d307a5e48b7515467ae7a5fc2daf805bd0ce6d8e9e604bade9f385",
            )
            .unwrap(),
        ))
        .build()?;

    let mut counter = 0;
    if duration.is_zero() {
        loop {
            let msg = zsub.recv()?;
            tracing::info!("Recv {counter}-th CDR msg of {} bytes", msg.data.len());
            counter += 1;
        }
    } else {
        let fut = async {
            loop {
                match zsub.async_recv().await {
                    Ok(msg) => {
                        tracing::info!("Recv {counter}-th CDR msg of {} bytes", msg.data.len());
                    }
                    Err(err) => {
                        tracing::error!(err);
                        break;
                    }
                }
                counter += 1;
            }
        };
        tokio::runtime::Runtime::new()?.block_on(async {
            let _ = tokio::time::timeout(duration, fut).await?;
            Result::Ok(())
        })?;
    }

    Ok(())
}

fn run_cdr_publisher(
    topic: String,
    duration: Duration,
    period: Duration,
    payload_size: usize,
) -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("MyNode").build()?;
    let zpub = node
        .create_pub::<ByteMultiArray>(&topic)
        .with_type_info(TypeInfo::new(
            "std_msgs::msg::dds_::UInt8MultiArray_",
            TypeHash::from_rihs_string(
                "RIHS01_5687e861b8d307a5e48b7515467ae7a5fc2daf805bd0ce6d8e9e604bade9f385",
            )
            .unwrap(),
        ))
        .build()?;
    let now = std::time::Instant::now();
    let mut msg = ros_z::ros_msg::ByteMultiArray::default();
    msg.data = vec![0u8; payload_size as _];
    loop {
        zpub.publish(&msg)?;
        tracing::info!("Published CDR msg of {} bytes", msg.data.len());
        std::thread::sleep(period);
        if !duration.is_zero() && now.elapsed() >= duration {
            break;
        }
    }
    Ok(())
}

fn run_protobuf_subscriber(topic: String, duration: Duration) -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("MyNode").build()?;

    let zsub = node
        .create_sub_with_serdes::<example::Entity, ProtobufSerdes<_>>(&topic)
        .with_type_info(TypeInfo::new(
            "example::Entity",
            TypeHash::from_rihs_string(
                "RIHS01_0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
            )
            .unwrap(),
        ))
        .build()?;

    let mut counter = 0;
    if duration.is_zero() {
        loop {
            let msg = zsub.recv()?;
            tracing::info!(
                "Recv {counter}-th protobuf msg: id={}, name={}",
                msg.id,
                msg.name
            );
            counter += 1;
        }
    } else {
        let fut = async {
            loop {
                match zsub.async_recv().await {
                    Ok(msg) => {
                        tracing::info!(
                            "Recv {counter}-th protobuf msg: id={}, name={}",
                            msg.id,
                            msg.name
                        );
                    }
                    Err(err) => {
                        tracing::error!("Error: {err}");
                        break;
                    }
                }
                counter += 1;
            }
        };
        tokio::runtime::Runtime::new()?.block_on(async {
            let _ = tokio::time::timeout(duration, fut).await?;
            Result::Ok(())
        })?;
    }

    Ok(())
}

fn run_protobuf_publisher(
    topic: String,
    duration: Duration,
    period: Duration,
    _payload_size: usize,
) -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("MyNode").build()?;

    // Explicit serdes selection - using Protobuf serialization for this Entity
    let zpub = node
        .create_pub_with_serdes::<example::Entity, ProtobufSerdes<_>>(&topic)
        .with_type_info(TypeInfo::new(
            "example::Entity",
            TypeHash::from_rihs_string(
                "RIHS01_0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
            )
            .unwrap(),
        ))
        .build()?;
    let now = std::time::Instant::now();
    let mut counter = 0u32;
    loop {
        let msg = example::Entity {
            id: counter,
            name: format!("Entity_{}", counter),
        };
        zpub.publish(&msg)?;
        tracing::info!("Published protobuf msg: id={}, name={}", msg.id, msg.name);
        counter += 1;
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
        "Begin testing with topic: {}, duration: {duration:?}, payload: {}, rate: {} Hz, serdes: {:?}",
        args.topic,
        args.payload,
        args.rate,
        args.serdes
    );

    match args.serdes {
        SerdesType::Protobuf => {
            if args.mode == "sub" {
                run_protobuf_subscriber(args.topic, duration)?;
            } else if args.mode == "pub" {
                run_protobuf_publisher(args.topic, duration, period, args.payload)?;
            } else {
                tracing::error!("The mode {} is not supported.", args.mode);
            }
        }
        SerdesType::Cdr => {
            if args.mode == "sub" {
                run_cdr_subscriber(args.topic, duration)?;
            } else if args.mode == "pub" {
                run_cdr_publisher(args.topic, duration, period, args.payload)?;
            } else {
                tracing::error!("The mode {} is not supported.", args.mode);
            }
        }
    }
    Ok(())
}
