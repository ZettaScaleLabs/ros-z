use std::time::Duration;

use ros_z::{
    Builder, Result,
    context::ZContext,
    qos::{QosHistory, QosProfile},
};
use ros_z_msgs::std_msgs::String as RosString;

/// Talker node that publishes "Hello World" messages to a topic
///
/// # Arguments
/// * `ctx` - The ROS-Z context
/// * `topic` - The topic name to publish to
/// * `period` - Duration between messages
/// * `max_count` - Optional maximum number of messages to publish. If None, publishes indefinitely.
pub async fn run_talker(
    ctx: ZContext,
    topic: &str,
    period: Duration,
    max_count: Option<usize>,
) -> Result<()> {
    // Create a node named "talker"
    let node = ctx.create_node("talker").build()?;

    // Create a publisher with a custom Quality of Service profile
    let qos = QosProfile {
        history: QosHistory::KeepLast(7),
        ..Default::default()
    };
    let publisher = node.create_pub::<RosString>(topic).with_qos(qos).build()?;

    let mut count = 1;

    loop {
        // Create the message
        let msg = RosString {
            data: format!("Hello World: {}", count),
        };

        // Log the message being published
        println!("Publishing: '{}'", msg.data);

        // Publish the message (non-blocking)
        publisher.async_publish(&msg).await?;

        // Check if we've reached the max count
        if let Some(max) = max_count
            && count >= max
        {
            break;
        }

        // Wait for the next publish cycle
        tokio::time::sleep(period).await;

        count += 1;
    }

    Ok(())
}

// Only compile main when building as a binary (not when included as a module)
#[cfg(not(any(test, doctest)))]
#[tokio::main]
async fn main() -> Result<()> {
    use clap::Parser;
    use ros_z::context::ZContextBuilder;

    let args = Args::parse();

    // Initialize logging
    zenoh::init_log_from_env_or("error");

    // Create the ROS-Z context with optional configuration
    let ctx = if let Some(e) = args.endpoint {
        ZContextBuilder::default()
            .with_mode(args.mode)
            .with_connect_endpoints([e])
            .build()?
    } else {
        ZContextBuilder::default().with_mode(args.mode).build()?
    };

    let period = Duration::from_secs_f64(args.period);

    // Run the talker node
    run_talker(ctx, &args.topic, period, None).await
}

#[cfg(not(any(test, doctest)))]
#[derive(Debug, clap::Parser)]
#[command(
    name = "demo_nodes_talker",
    about = "ROS 2 demo talker node - publishes 'Hello World' messages"
)]
struct Args {
    /// Topic name to publish to
    #[arg(short, long, default_value = "chatter")]
    topic: String,

    /// Publishing period in seconds
    #[arg(short, long, default_value = "1.0")]
    period: f64,

    /// Zenoh session mode (peer, client, router)
    #[arg(short, long, default_value = "peer")]
    mode: String,

    /// Zenoh router endpoint to connect to (e.g., tcp/localhost:7447)
    #[arg(short, long)]
    endpoint: Option<String>,
}
