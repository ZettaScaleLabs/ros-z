//! Auto-discovery DDS↔Zenoh bridge.
//!
//! Connects to a Zenoh router and a CycloneDDS domain, then watches for DDS endpoints
//! and automatically creates routes for every publisher, subscriber, service server, and
//! service client that appears. Bridge-to-bridge federation is also handled automatically:
//! a second bridge on a different DDS domain routes traffic across domains without any
//! manual configuration.
//!
//! Run with:
//! ```bash
//! cargo run --example auto_bridge -p ros-z-dds
//! ```

use anyhow::{Result, anyhow};
use ros_z::{Builder, context::ZContextBuilder};
use ros_z_dds::{CyclorsParticipant, DdsParticipant, ZDdsBridge};

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
        .create_node("zenoh_bridge_dds")
        .build()
        .map_err(|e| anyhow!("{e}"))?;

    let participant = CyclorsParticipant::create(domain_id)?;

    tracing::info!("bridge started on DDS domain {domain_id}");

    ZDdsBridge::new(node, participant).run().await
}
