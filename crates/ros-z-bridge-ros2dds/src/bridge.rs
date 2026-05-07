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
        names::{is_pubsub_topic, is_request_topic},
        participant::{create_participant, get_instance_handle},
    },
    routes::{
        pubsub::{DdsToZenohRoute, ZenohToDdsRoute},
        service::ServiceRoute,
        service_cli::ServiceCliRoute,
    },
};

/// Compiled per-type allow/deny regex pair.
struct Filter {
    allow: Option<Regex>,
    deny: Option<Regex>,
}

impl Filter {
    fn compile(allow: Option<&str>, deny: Option<&str>) -> Result<Self> {
        let allow = allow
            .map(Regex::new)
            .transpose()
            .map_err(|e| anyhow::anyhow!("invalid allow regex: {e}"))?;
        let deny = deny
            .map(Regex::new)
            .transpose()
            .map_err(|e| anyhow::anyhow!("invalid deny regex: {e}"))?;
        Ok(Self { allow, deny })
    }

    /// Returns true if `name` passes this filter (allow overrides deny; if neither set, passes).
    fn allows(&self, name: &str) -> bool {
        if let Some(deny) = &self.deny {
            if deny.is_match(name) {
                return false;
            }
        }
        if let Some(allow) = &self.allow {
            return allow.is_match(name);
        }
        true
    }
}

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

    /// Global filter (applied when no per-type filter is set)
    global: Filter,
    /// Per-type filters; fall back to global when None fields are present
    filter_pub: Filter,
    filter_sub: Filter,
    filter_service_srv: Filter,
    filter_service_cli: Filter,
}

impl Bridge {
    pub async fn new(config: Config, session: Session) -> Result<Self> {
        let participant = create_participant(config.domain_id)?;
        let client_guid = get_instance_handle(participant.raw())?;

        let global = Filter::compile(config.allow.as_deref(), config.deny.as_deref())?;

        // Per-type filters: use explicit per-type if set, else fall back to global regex.
        let filter_pub = Filter::compile(
            config.allow_pub.as_deref().or(config.allow.as_deref()),
            config.deny_pub.as_deref().or(config.deny.as_deref()),
        )?;
        let filter_sub = Filter::compile(
            config.allow_sub.as_deref().or(config.allow.as_deref()),
            config.deny_sub.as_deref().or(config.deny.as_deref()),
        )?;
        let filter_service_srv = Filter::compile(
            config
                .allow_service_srv
                .as_deref()
                .or(config.allow.as_deref()),
            config
                .deny_service_srv
                .as_deref()
                .or(config.deny.as_deref()),
        )?;
        let filter_service_cli = Filter::compile(
            config
                .allow_service_cli
                .as_deref()
                .or(config.allow.as_deref()),
            config
                .deny_service_cli
                .as_deref()
                .or(config.deny.as_deref()),
        )?;

        Ok(Self {
            config,
            session,
            participant,
            client_guid,
            dds_to_zenoh: HashMap::new(),
            zenoh_to_dds: HashMap::new(),
            service_srv: HashMap::new(),
            service_cli: HashMap::new(),
            global,
            filter_pub,
            filter_sub,
            filter_service_srv,
            filter_service_cli,
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
                self.on_discovered_publication(ep).await?;
            }
            DiscoveryEvent::UndiscoveredPublication(gid) => {
                self.dds_to_zenoh.remove(&gid);
                self.service_cli.remove(&gid);
                tracing::debug!("Removed publication route for {gid}");
            }
            DiscoveryEvent::DiscoveredSubscription(ep) => {
                self.on_discovered_subscription(ep).await?;
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
            if self.filter_pub.allows(&ep.topic_name) {
                tracing::info!("DDS→Zenoh pub route: {}", ep.topic_name);
                let route = DdsToZenohRoute::create(
                    self.participant.raw(),
                    &ep,
                    &self.session,
                    self.config.namespace.as_deref(),
                    self.config.reliable_routes_blocking,
                )
                .await?;
                self.dds_to_zenoh.insert(ep.key, route);
            }
        } else if is_request_topic(&ep.topic_name) {
            if self.filter_service_cli.allows(&ep.topic_name) {
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
        }
        // reply publications (rr/) are handled inside ServiceRoute
        Ok(())
    }

    /// A DDS subscription was discovered (someone is subscribing on DDS).
    async fn on_discovered_subscription(&mut self, ep: DiscoveredEndpoint) -> Result<()> {
        if is_pubsub_topic(&ep.topic_name) {
            if self.filter_sub.allows(&ep.topic_name) {
                tracing::info!("Zenoh→DDS sub route: {}", ep.topic_name);
                let route = ZenohToDdsRoute::create(
                    self.participant.raw(),
                    &ep,
                    &self.session,
                    self.config.namespace.as_deref(),
                )
                .await?;
                self.zenoh_to_dds.insert(ep.key, route);
            }
        } else if is_request_topic(&ep.topic_name) {
            if self.filter_service_srv.allows(&ep.topic_name) {
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
        }
        // reply subscriptions (rr/) are internal; ServiceRoute handles them
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::Filter;

    #[test]
    fn test_filter_no_rules_allows_all() {
        let f = Filter::compile(None, None).unwrap();
        assert!(f.allows("rt/chatter"));
        assert!(f.allows("rq/add_two_intsRequest"));
    }

    #[test]
    fn test_filter_allow_only_matching() {
        let f = Filter::compile(Some("rt/chatter"), None).unwrap();
        assert!(f.allows("rt/chatter"));
        assert!(!f.allows("rt/other"));
    }

    #[test]
    fn test_filter_deny_blocks() {
        let f = Filter::compile(None, Some("rt/private.*")).unwrap();
        assert!(!f.allows("rt/private_topic"));
        assert!(f.allows("rt/chatter"));
    }

    #[test]
    fn test_filter_deny_overrides_allow_for_matching_deny() {
        // deny takes priority: topic matches deny → blocked even if allow also matches
        let f = Filter::compile(Some(".*"), Some("rt/secret")).unwrap();
        assert!(!f.allows("rt/secret"));
        assert!(f.allows("rt/chatter"));
    }

    #[test]
    fn test_filter_invalid_regex_returns_error() {
        assert!(Filter::compile(Some("[invalid"), None).is_err());
        assert!(Filter::compile(None, Some("[invalid")).is_err());
    }
}
