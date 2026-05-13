mod config;

use anyhow::{Result, anyhow};
use clap::Parser;
use ros_z::{Builder, context::ZContextBuilder};
use ros_z_dds::{CyclorsParticipant, DdsParticipant, ZDdsBridge};

use crate::config::Config;

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .init();

    let cfg = Config::parse();

    let ctx = ZContextBuilder::default()
        .with_connect_endpoints([cfg.zenoh_endpoint.as_str()])
        .keyexpr_format(cfg.wire_format.into())
        .build()
        .map_err(|e| anyhow!("{e}"))?;

    let node = ctx
        .create_node(&cfg.node_name)
        .with_namespace(cfg.namespace.as_deref().unwrap_or("/"))
        .build()
        .map_err(|e| anyhow!("{e}"))?;

    let participant = CyclorsParticipant::create(cfg.domain_id)?;

    ZDdsBridge::new(node, participant)
        .allow_topics_regex(cfg.allow.as_deref())
        .deny_topics_regex(cfg.deny.as_deref())
        .service_timeout(std::time::Duration::from_secs(cfg.service_timeout_secs))
        .action_get_result_timeout(std::time::Duration::from_secs(
            cfg.action_get_result_timeout_secs,
        ))
        .transient_local_cache_multiplier(cfg.transient_local_cache_multiplier)
        .run()
        .await
}
