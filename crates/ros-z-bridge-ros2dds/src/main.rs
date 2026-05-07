mod bridge;
mod config;
mod dds;
mod liveliness;
mod routes;

use anyhow::{Result, anyhow};
use clap::Parser;
use zenoh::Config as ZConfig;

use crate::{bridge::Bridge, config::Config};

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .init();

    let config = Config::parse();

    // G7: load Zenoh config file if provided, then overlay the endpoint.
    let mut z_config = match &config.zenoh_config_file {
        Some(path) => {
            ZConfig::from_file(path).map_err(|e| anyhow!("Zenoh config load failed: {e}"))?
        }
        None => ZConfig::default(),
    };
    z_config
        .insert_json5(
            "connect/endpoints",
            &format!(r#"["{}"]"#, config.zenoh_endpoint),
        )
        .map_err(|e| anyhow!("Zenoh config error: {e}"))?;

    tracing::info!("Connecting to Zenoh at {}", config.zenoh_endpoint);
    let session = zenoh::open(z_config)
        .await
        .map_err(|e| anyhow!("Zenoh open failed: {e}"))?;

    Bridge::new(config, session).await?.run().await
}
