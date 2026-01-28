use std::time::Duration;

use clap::{Parser, ValueEnum};
#[cfg(feature = "ros2dds")]
use ros_z::backend::Ros2DdsBackend;
use ros_z::{
    Builder, Result,
    backend::{KeyExprBackend, RmwZenohBackend},
    context::{ZContext, ZContextBuilder},
};
use ros_z_msgs::std_msgs::String as RosString;

/// Subscriber function that continuously receives messages from a topic
async fn run_subscriber<B: KeyExprBackend>(ctx: ZContext, topic: String) -> Result<()> {
    // Create a ROS 2 node - the fundamental unit of computation
    // Nodes are logical groupings of publishers, subscribers, services, etc.
    let node = ctx.create_node("Sub").build()?;

    // Create a subscriber for the specified topic with backend selection
    // The type parameter RosString determines what message type we'll receive
    // The backend parameter determines the key expression format
    let zsub = node
        .create_sub::<RosString>(&topic)
        .with_backend::<B>()
        .build()?;

    // Continuously receive messages asynchronously
    // This loop will block waiting for messages on the topic
    while let Ok(msg) = zsub.async_recv().await {
        println!("Hearing:>> {}", msg.data);
    }
    Ok(())
}

/// Publisher function that continuously publishes messages to a topic
async fn run_publisher<B: KeyExprBackend>(
    ctx: ZContext,
    topic: String,
    period: Duration,
    payload: String,
) -> Result<()> {
    // Create a ROS 2 node for publishing
    let node = ctx.create_node("Pub").build()?;

    // Create a publisher for the specified topic with backend selection
    // The type parameter RosString determines what message type we'll send
    // The backend parameter determines the key expression format
    let zpub = node
        .create_pub::<RosString>(&topic)
        .with_backend::<B>()
        .build()?;

    let mut count = 0;
    loop {
        // Create a new message with incrementing counter
        let str = RosString {
            data: format!("{payload} - #{count}"),
        };
        println!("Telling:>> {}", str.data);

        // Publish the message asynchronously to all subscribers on this topic
        zpub.async_publish(&str).await?;

        // Wait for the specified period before publishing again
        let _ = tokio::time::sleep(period).await;
        count += 1;
    }
}

// The #[tokio::main] attribute sets up the async runtime
// ros-z requires an async runtime (Tokio is the most common choice)
#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();

    // Create a ZContext - the entry point for ros-z applications
    // ZContext manages the connection to the Zenoh network and coordinates
    // communication between nodes. It can be configured with different modes:
    // - "peer" mode: nodes discover each other via multicast scouting
    // - "client" mode: nodes connect to a Zenoh router
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

    // Run as either a publisher (talker) or subscriber (listener)
    // Both share the same ZContext but perform different roles
    // Backend selection determines key expression format (domain prefix or not)
    match (args.role.as_str(), args.backend) {
        ("listener", Backend::RmwZenoh) => {
            run_subscriber::<RmwZenohBackend>(ctx, args.topic).await?
        }
        #[cfg(feature = "ros2dds")]
        ("listener", Backend::Ros2Dds) => run_subscriber::<Ros2DdsBackend>(ctx, args.topic).await?,
        ("talker", Backend::RmwZenoh) => {
            run_publisher::<RmwZenohBackend>(ctx, args.topic, period, args.data).await?
        }
        #[cfg(feature = "ros2dds")]
        ("talker", Backend::Ros2Dds) => {
            run_publisher::<Ros2DdsBackend>(ctx, args.topic, period, args.data).await?
        }
        (role, _) => println!(
            "Please use \"talker\" or \"listener\" as role, {} is not supported.",
            role
        ),
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum Backend {
    /// RmwZenoh backend (default) - compatible with rmw_zenoh nodes
    /// Uses key expressions with domain prefix: <domain_id>/<topic>/**
    RmwZenoh,
    /// Ros2Dds backend - compatible with zenoh-bridge-ros2dds
    /// Uses key expressions without domain prefix: <topic>/**
    #[cfg(feature = "ros2dds")]
    Ros2Dds,
}

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
    /// Backend selection: rmw-zenoh (default) or ros2-dds
    #[arg(short, long, value_enum, default_value = "rmw-zenoh")]
    backend: Backend,
}
