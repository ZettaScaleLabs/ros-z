use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use color_eyre::eyre::{Result, WrapErr, eyre};
use ros_z::{
    Builder,
    context::{ZContext, ZContextBuilder},
    dynamic::DynSubBuilder,
    graph::GraphSnapshot,
    node::ZNode,
    parameter::{ParameterClient, ParameterTarget},
};

use crate::{
    cli::Backend,
    support::{display_error, graph::SnapshotFingerprint},
};

const GRAPH_POLL_INTERVAL: Duration = Duration::from_millis(200);
const GRAPH_SETTLE_TIMEOUT: Duration = Duration::from_secs(2);

pub struct AppContext {
    context: ZContext,
    node: Arc<ZNode>,
    domain_id: usize,
}

impl AppContext {
    pub fn new(router: &str, domain_id: usize, backend: Backend) -> Result<Self> {
        let keyexpr_format = match backend {
            Backend::RmwZenoh => ros_z_protocol::KeyExprFormat::RmwZenoh,
            Backend::Ros2Dds => ros_z_protocol::KeyExprFormat::Ros2Dds,
        };

        let context = ZContextBuilder::default()
            .with_mode("peer")
            .with_connect_endpoints([router])
            .with_domain_id(domain_id)
            .keyexpr_format(keyexpr_format)
            .build()
            .map_err(|error| eyre!(error))
            .wrap_err("failed to build ros-z context")?;
        let node = Arc::new(
            context
                .create_node("rosz")
                .with_type_description_service()
                .build()
                .map_err(|error| eyre!(error))
                .wrap_err("failed to build rosz node")?,
        );

        Ok(Self {
            context,
            node,
            domain_id,
        })
    }

    pub fn graph(&self) -> &ros_z::graph::Graph {
        self.context.graph().as_ref()
    }

    pub fn snapshot(&self) -> GraphSnapshot {
        self.graph().snapshot(self.domain_id)
    }

    pub async fn wait_for_graph_settle(&self) {
        let deadline = Instant::now() + GRAPH_SETTLE_TIMEOUT;
        let mut previous = SnapshotFingerprint::from(&self.snapshot());

        while Instant::now() < deadline {
            tokio::time::sleep(GRAPH_POLL_INTERVAL).await;

            let current_snapshot = self.snapshot();
            let current = SnapshotFingerprint::from(&current_snapshot);
            if current == previous {
                return;
            }
            previous = current;
        }
    }

    pub async fn wait_for_graph_condition<F>(&self, predicate: F)
    where
        F: Fn(&ros_z::graph::Graph) -> bool,
    {
        let deadline = Instant::now() + GRAPH_SETTLE_TIMEOUT;

        while Instant::now() < deadline {
            if predicate(self.graph()) {
                return;
            }
            tokio::time::sleep(GRAPH_POLL_INTERVAL).await;
        }
    }

    pub async fn create_dynamic_subscriber_builder(
        &self,
        topic: &str,
        discovery_timeout: Duration,
    ) -> Result<DynSubBuilder> {
        self.node
            .create_dyn_sub_auto(topic, discovery_timeout)
            .await
            .map_err(|error| eyre!(error))
    }

    pub fn parameter_client(&self, target: ParameterTarget) -> Result<ParameterClient> {
        ParameterClient::new(Arc::clone(&self.node), target).map_err(display_error)
    }

    pub fn shutdown(&self) -> Result<()> {
        self.context
            .shutdown()
            .map_err(|error| eyre!(error))
            .wrap_err("failed to close ros-z context")
    }
}
