//! Typed DDS↔Zenoh bridge using `DdsBridgeExt`.
//!
//! Demonstrates bridging specific topics with compile-time type information.  The
//! type hash is embedded in the Zenoh liveliness token, which enables remote bridges
//! and ros-z nodes to reconstruct the exact DDS type without any runtime negotiation.
//!
//! This approach is suitable when you know the message types upfront and want:
//! - Compile-time type checking
//! - Full type-hash embedding in liveliness tokens
//! - No unintended topics being bridged
//!
//! For a bridge that automatically discovers and routes everything, see `auto_bridge.rs`.
//!
//! Run with:
//! ```bash
//! cargo run --example custom_bridge -p ros-z-dds
//! ```

use anyhow::{Result, anyhow};
use ros_z::{Builder, context::ZContextBuilder};
use ros_z_dds::{CyclorsParticipant, DdsBridgeExt, DdsParticipant};
use ros_z_msgs::std_msgs::String as RosString;

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .init();

    let domain_id: u32 = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);

    let ctx = ZContextBuilder::default()
        .with_connect_endpoints(["tcp/127.0.0.1:7447"])
        .build()
        .map_err(|e| anyhow!("{e}"))?;

    let node = ctx
        .create_node("custom_bridge")
        .build()
        .map_err(|e| anyhow!("{e}"))?;

    let participant = CyclorsParticipant::create(domain_id)?;

    // Bridge /chatter: DDS publishers → Zenoh and Zenoh → DDS subscribers.
    // The type hash for std_msgs/String is embedded in the liveliness token so
    // remote bridges can reconstruct the DDS topic with the exact same type.
    let _pub_bridge = node
        .bridge_dds_pub::<RosString>("/chatter", &participant)
        .await?;
    let _sub_bridge = node
        .bridge_dds_sub::<RosString>("/chatter", &participant)
        .await?;

    tracing::info!("custom bridge active on DDS domain {domain_id}");
    tracing::info!("  pub/sub: /chatter (std_msgs/String)");

    // Routes are RAII handles — they stay active as long as the variables are in scope.
    tokio::signal::ctrl_c().await?;
    tracing::info!("shutting down");
    Ok(())
}
