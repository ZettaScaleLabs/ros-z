use std::collections::HashMap;

use anyhow::Result;
use regex::Regex;
use zenoh::Session;

use crate::{
    config::Config,
    dds::{
        discovery::{DiscoveredEndpoint, DiscoveryEvent, run_discovery},
        entity::DdsEntity,
        gid::Gid,
        names::{is_pubsub_topic, is_reply_topic, is_request_topic},
        participant::{create_participant, get_instance_handle},
    },
    routes::{
        pubsub::{DdsToZenohRoute, ZenohToDdsRoute},
        service::ServiceRoute,
        service_cli::ServiceCliRoute,
    },
};

/// Top-level bridge state.
///
/// Holds all active routes keyed by the DDS endpoint GID.
/// Dropping a route tears down the underlying DDS entities via RAII.
pub struct Bridge {
    config: Config,
    session: Session,
    participant: DdsEntity,
    /// Stable u64 derived from the participant instance handle. Used as client_guid
    /// for all service requests originated by this bridge (fixes #647).
    client_guid: u64,

    dds_to_zenoh: HashMap<Gid, DdsToZenohRoute>,
    zenoh_to_dds: HashMap<Gid, ZenohToDdsRoute>,
    /// DDS server → Zenoh queryable
    service_srv: HashMap<Gid, ServiceRoute>,
    /// DDS client → Zenoh querier
    service_cli: HashMap<Gid, ServiceCliRoute>,

    allow: Option<Regex>,
    deny: Option<Regex>,
}

impl Bridge {
    pub async fn new(config: Config, session: Session) -> Result<Self> {
        let participant = create_participant(config.domain_id)?;
        let client_guid = get_instance_handle(participant.raw())?;

        let allow = config
            .allow
            .as_deref()
            .map(Regex::new)
            .transpose()
            .map_err(|e| anyhow::anyhow!("invalid allow regex: {e}"))?;
        let deny = config
            .deny
            .as_deref()
            .map(Regex::new)
            .transpose()
            .map_err(|e| anyhow::anyhow!("invalid deny regex: {e}"))?;

        Ok(Self {
            config,
            session,
            participant,
            client_guid,
            dds_to_zenoh: HashMap::new(),
            zenoh_to_dds: HashMap::new(),
            service_srv: HashMap::new(),
            service_cli: HashMap::new(),
            allow,
            deny,
        })
    }

    /// Start the discovery loop and process events until shutdown.
    pub async fn run(mut self) -> Result<()> {
        let (tx, rx) = flume::bounded::<DiscoveryEvent>(256);
        run_discovery(self.participant.raw(), tx);

        tracing::info!(
            "Bridge running (domain={}, zenoh={})",
            self.config.domain_id,
            self.config.zenoh_endpoint,
        );

        while let Ok(event) = rx.recv_async().await {
            if let Err(e) = self.handle_event(event).await {
                tracing::warn!("Route creation error: {e}");
            }
        }
        Ok(())
    }

    async fn handle_event(&mut self, event: DiscoveryEvent) -> Result<()> {
        match event {
            DiscoveryEvent::DiscoveredPublication(ep) => {
                if self.should_bridge(&ep.topic_name) {
                    self.on_discovered_publication(ep).await?;
                }
            }
            DiscoveryEvent::UndiscoveredPublication(gid) => {
                self.dds_to_zenoh.remove(&gid);
                self.service_cli.remove(&gid);
                tracing::debug!("Removed publication route for {gid}");
            }
            DiscoveryEvent::DiscoveredSubscription(ep) => {
                if self.should_bridge(&ep.topic_name) {
                    self.on_discovered_subscription(ep).await?;
                }
            }
            DiscoveryEvent::UndiscoveredSubscription(gid) => {
                self.zenoh_to_dds.remove(&gid);
                self.service_srv.remove(&gid);
                tracing::debug!("Removed subscription route for {gid}");
            }
        }
        Ok(())
    }

    /// A DDS publication was discovered (someone is publishing on DDS).
    async fn on_discovered_publication(&mut self, ep: DiscoveredEndpoint) -> Result<()> {
        if is_pubsub_topic(&ep.topic_name) {
            // Regular pub/sub: DDS publisher → Zenoh publisher
            tracing::info!("DDS→Zenoh pub route: {}", ep.topic_name);
            let route = DdsToZenohRoute::create(
                self.participant.raw(),
                &ep,
                &self.session,
                self.config.namespace.as_deref(),
            )
            .await?;
            self.dds_to_zenoh.insert(ep.key, route);
        } else if is_request_topic(&ep.topic_name) {
            // A DDS service CLIENT is publishing requests: DDS client → Zenoh querier
            tracing::info!("DDS client→Zenoh querier route: {}", ep.topic_name);
            let route = ServiceCliRoute::create(
                self.participant.raw(),
                &ep,
                &self.session,
                self.config.namespace.as_deref(),
            )
            .await?;
            self.service_cli.insert(ep.key, route);
        }
        // reply publications (rr/) are handled inside ServiceRoute
        Ok(())
    }

    /// A DDS subscription was discovered (someone is subscribing on DDS).
    async fn on_discovered_subscription(&mut self, ep: DiscoveredEndpoint) -> Result<()> {
        if is_pubsub_topic(&ep.topic_name) {
            // Regular pub/sub: Zenoh subscriber → DDS writer
            tracing::info!("Zenoh→DDS sub route: {}", ep.topic_name);
            let route = ZenohToDdsRoute::create(
                self.participant.raw(),
                &ep,
                &self.session,
                self.config.namespace.as_deref(),
            )
            .await?;
            self.zenoh_to_dds.insert(ep.key, route);
        } else if is_request_topic(&ep.topic_name) {
            // A DDS service SERVER is subscribing to requests: expose as Zenoh queryable
            tracing::info!("DDS server→Zenoh queryable route: {}", ep.topic_name);
            let route = ServiceRoute::create(
                self.participant.raw(),
                &ep,
                &self.session,
                self.config.namespace.as_deref(),
                self.client_guid,
            )
            .await?;
            self.service_srv.insert(ep.key, route);
        }
        // reply subscriptions (rr/) are internal; ServiceRoute handles them
        Ok(())
    }

    fn should_bridge(&self, topic_name: &str) -> bool {
        if let Some(deny) = &self.deny {
            if deny.is_match(topic_name) {
                return false;
            }
        }
        if let Some(allow) = &self.allow {
            return allow.is_match(topic_name);
        }
        true
    }
}
