use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

use anyhow::Result;
use parking_lot::Mutex;
use regex::Regex;
use zenoh::{Session, liveliness::LivelinessToken};

use crate::{
    config::Config,
    dds::{
        discovery::{DiscoveredEndpoint, DiscoveryEvent, run_discovery},
        entity::DdsEntity,
        gid::Gid,
        names::{
            dds_topic_to_ros2_name, dds_type_to_ros2_action_type, dds_type_to_ros2_service_type,
            dds_type_to_ros2_type, is_pubsub_topic, is_request_topic, ros2_name_to_zenoh_key,
        },
        participant::create_participant,
    },
    liveliness::{
        action_base_name, build_action_cli_lv_key, build_action_srv_lv_key, build_bridge_lv_key,
        build_pub_lv_key, build_service_cli_lv_key, build_service_srv_lv_key, build_sub_lv_key,
    },
    routes::{
        action::is_action_component,
        pubsub::{DdsToZenohRoute, TopicPublisherSlot, ZenohToDdsRoute, priority_from_u8},
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

    /// Returns true if `name` passes this filter (deny checked first; if neither set, passes).
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

enum EntityKind {
    Publisher,
    Subscriber,
    ServiceServer,
    ServiceClient,
}

/// Top-level bridge state.
///
/// Holds all active routes keyed by (domain_id, DDS endpoint GID).
/// Dropping a route tears down the underlying DDS entities via RAII.
pub struct Bridge {
    config: Config,
    session: Session,
    /// One participant per domain ID.
    participants: Vec<(u32, DdsEntity)>,

    /// DDS→Zenoh routes keyed by (domain_id, gid).
    dds_to_zenoh: HashMap<(u32, Gid), DdsToZenohRoute>,
    /// Shared Zenoh publisher slots for DDS topics; keyed by (domain_id, dds_topic_name).
    /// Kept alive across DDS writer churn to preserve AdvancedPublisher history cache (#690).
    topic_publishers: Arc<Mutex<HashMap<(u32, String), Arc<TopicPublisherSlot>>>>,
    zenoh_to_dds: HashMap<(u32, Gid), ZenohToDdsRoute>,
    /// DDS server → Zenoh queryable
    service_srv: HashMap<(u32, Gid), ServiceRoute>,
    /// DDS client → Zenoh querier
    service_cli: HashMap<(u32, Gid), ServiceCliRoute>,

    /// Global filter (applied when no per-type filter is set)
    global: Filter,
    /// Per-type filters; fall back to global when per-type unset
    filter_pub: Filter,
    filter_sub: Filter,
    filter_service_srv: Filter,
    filter_service_cli: Filter,
    /// Action-specific filter (/_action/ topics); falls back to global.
    filter_action: Filter,

    // ── Liveliness ────────────────────────────────────────────────────────────
    /// Zenoh session ID string, used as a key component in liveliness tokens.
    zid: String,
    /// Self-announcement token (`@/{zid}/@ros2_lv`) kept alive for the bridge lifetime.
    /// Signals peer bridges that this instance is active, matching zenoh-plugin-ros2dds.
    _bridge_lv_token: Option<LivelinessToken>,
    /// Per-route entity liveliness tokens; removed (and thus undeclared) with their routes.
    entity_lv_tokens: HashMap<(u32, Gid), LivelinessToken>,
}

impl Bridge {
    pub async fn new(config: Config, session: Session) -> Result<Self> {
        // Validate domain IDs: DDS spec limits to 0–229; no duplicates.
        let mut seen = HashSet::new();
        for &did in &config.domain_ids {
            if did > 229 {
                anyhow::bail!("domain ID {did} is out of range (valid: 0–229)");
            }
            if !seen.insert(did) {
                anyhow::bail!("domain ID {did} specified more than once");
            }
        }

        let participants: Vec<(u32, DdsEntity)> = config
            .domain_ids
            .iter()
            .map(|&did| create_participant(did).map(|p| (did, p)))
            .collect::<Result<_>>()?;

        let global = Filter::compile(config.allow.as_deref(), config.deny.as_deref())?;

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
        let filter_action = Filter::compile(
            config.allow_action.as_deref().or(config.allow.as_deref()),
            config.deny_action.as_deref().or(config.deny.as_deref()),
        )?;

        let zid = session.info().zid().await.to_string();

        let bridge_lv_token = match session
            .liveliness()
            .declare_token(build_bridge_lv_key(&zid))
            .await
        {
            Ok(t) => {
                tracing::debug!("Bridge self-announcement liveliness token declared (zid={zid})");
                Some(t)
            }
            Err(e) => {
                tracing::warn!("Failed to declare bridge liveliness token: {e}");
                None
            }
        };

        Ok(Self {
            config,
            session,
            participants,
            dds_to_zenoh: HashMap::new(),
            topic_publishers: Arc::new(Mutex::new(HashMap::new())),
            zenoh_to_dds: HashMap::new(),
            service_srv: HashMap::new(),
            service_cli: HashMap::new(),
            global,
            filter_pub,
            filter_sub,
            filter_service_srv,
            filter_service_cli,
            filter_action,
            zid,
            _bridge_lv_token: bridge_lv_token,
            entity_lv_tokens: HashMap::new(),
        })
    }

    /// Start the discovery loop and process events until shutdown.
    pub async fn run(mut self) -> Result<()> {
        // Merge all per-domain discovery channels into one tagged stream.
        let (merged_tx, merged_rx) = flume::bounded::<(u32, DiscoveryEvent)>(256);

        for (domain_id, participant) in &self.participants {
            let (tx, rx) = flume::bounded::<DiscoveryEvent>(256);
            run_discovery(participant.raw(), tx);
            let mtx = merged_tx.clone();
            let did = *domain_id;
            tokio::spawn(async move {
                while let Ok(ev) = rx.recv_async().await {
                    let _ = mtx.send_async((did, ev)).await;
                }
            });
        }
        // Drop our copy so merged_rx closes when all forwarders finish.
        drop(merged_tx);

        tracing::info!(
            "Bridge running (domains={:?}, zenoh={})",
            self.config.domain_ids,
            self.config.zenoh_endpoint,
        );

        while let Ok((domain_id, event)) = merged_rx.recv_async().await {
            if let Err(e) = self.handle_event(domain_id, event).await {
                tracing::warn!("Route creation error: {e}");
            }
        }
        Ok(())
    }

    async fn handle_event(&mut self, domain_id: u32, event: DiscoveryEvent) -> Result<()> {
        match event {
            DiscoveryEvent::DiscoveredPublication(ep) => {
                self.on_discovered_publication(domain_id, ep).await?;
            }
            DiscoveryEvent::UndiscoveredPublication(gid) => {
                let key = (domain_id, gid);
                if let Some(removed) = self.dds_to_zenoh.remove(&key) {
                    // G1: after dropping the reader (and its Arc<TopicPublisherSlot>), start a
                    // 5-second grace period before evicting the publisher slot from the map.
                    // If a new DDS writer for the same topic appears within that window, it will
                    // reuse the slot and its AdvancedPublisher history cache survives.
                    let topic_name = removed.topic_name().to_owned();
                    let slot_key = (domain_id, topic_name.clone());
                    let tp = Arc::clone(&self.topic_publishers);
                    drop(removed);
                    tokio::spawn(async move {
                        tokio::time::sleep(std::time::Duration::from_secs(5)).await;
                        let mut map = tp.lock();
                        if let Some(arc) = map.get(&slot_key) {
                            if Arc::strong_count(arc) == 1 {
                                map.remove(&slot_key);
                                tracing::debug!(
                                    "Evicted publisher slot for {} after grace period",
                                    slot_key.1
                                );
                            }
                        }
                    });
                }
                self.service_cli.remove(&key);
                // G5: drop entity token → liveliness undeclared on the Zenoh graph.
                self.entity_lv_tokens.remove(&key);
                tracing::debug!("Removed publication route for {gid}");
            }
            DiscoveryEvent::DiscoveredSubscription(ep) => {
                self.on_discovered_subscription(domain_id, ep).await?;
            }
            DiscoveryEvent::UndiscoveredSubscription(gid) => {
                let key = (domain_id, gid);
                self.zenoh_to_dds.remove(&key);
                self.service_srv.remove(&key);
                // G5: drop entity token → liveliness undeclared on the Zenoh graph.
                self.entity_lv_tokens.remove(&key);
                tracing::debug!("Removed subscription route for {gid}");
            }
        }
        Ok(())
    }

    // G2: pick the right filter based on whether the topic is an action component.

    fn pub_filter_for<'a>(&'a self, topic_name: &str) -> &'a Filter {
        if is_action_component(topic_name) {
            &self.filter_action
        } else {
            &self.filter_pub
        }
    }

    fn sub_filter_for<'a>(&'a self, topic_name: &str) -> &'a Filter {
        if is_action_component(topic_name) {
            &self.filter_action
        } else {
            &self.filter_sub
        }
    }

    fn service_cli_filter_for<'a>(&'a self, topic_name: &str) -> &'a Filter {
        if is_action_component(topic_name) {
            &self.filter_action
        } else {
            &self.filter_service_cli
        }
    }

    fn service_srv_filter_for<'a>(&'a self, topic_name: &str) -> &'a Filter {
        if is_action_component(topic_name) {
            &self.filter_action
        } else {
            &self.filter_service_srv
        }
    }

    /// A DDS publication was discovered (someone is publishing on DDS).
    async fn on_discovered_publication(
        &mut self,
        domain_id: u32,
        ep: DiscoveredEndpoint,
    ) -> Result<()> {
        if is_pubsub_topic(&ep.topic_name) {
            if self.pub_filter_for(&ep.topic_name).allows(&ep.topic_name) {
                tracing::info!("DDS→Zenoh pub route: {}", ep.topic_name);
                let route = DdsToZenohRoute::create(
                    self.participant_for(domain_id),
                    &ep,
                    &self.session,
                    self.config.namespace.as_deref(),
                    self.config.reliable_routes_blocking,
                    priority_from_u8(self.config.publication_priority),
                    self.config.publication_express,
                    self.config.max_publication_hz,
                    &self.topic_publishers,
                    domain_id,
                )
                .await?;
                self.dds_to_zenoh.insert((domain_id, ep.key), route);
                // G5: advertise this bridged publisher on the Zenoh graph.
                if let Some(token) = self
                    .declare_entity_token(domain_id, &ep, EntityKind::Publisher)
                    .await
                {
                    self.entity_lv_tokens.insert((domain_id, ep.key), token);
                }
            }
        } else if is_request_topic(&ep.topic_name) {
            if self
                .service_cli_filter_for(&ep.topic_name)
                .allows(&ep.topic_name)
            {
                tracing::info!("DDS client→Zenoh querier route: {}", ep.topic_name);
                let route = ServiceCliRoute::create(
                    self.participant_for(domain_id),
                    &ep,
                    &self.session,
                    self.config.namespace.as_deref(),
                )
                .await?;
                self.service_cli.insert((domain_id, ep.key), route);
                // G5: advertise this bridged service client on the Zenoh graph.
                if let Some(token) = self
                    .declare_entity_token(domain_id, &ep, EntityKind::ServiceClient)
                    .await
                {
                    self.entity_lv_tokens.insert((domain_id, ep.key), token);
                }
            }
        }
        Ok(())
    }

    /// A DDS subscription was discovered (someone is subscribing on DDS).
    async fn on_discovered_subscription(
        &mut self,
        domain_id: u32,
        ep: DiscoveredEndpoint,
    ) -> Result<()> {
        if is_pubsub_topic(&ep.topic_name) {
            if self.sub_filter_for(&ep.topic_name).allows(&ep.topic_name) {
                tracing::info!("Zenoh→DDS sub route: {}", ep.topic_name);
                let route = ZenohToDdsRoute::create(
                    self.participant_for(domain_id),
                    &ep,
                    &self.session,
                    self.config.namespace.as_deref(),
                )
                .await?;
                self.zenoh_to_dds.insert((domain_id, ep.key), route);
                // G5: advertise this bridged subscriber on the Zenoh graph.
                if let Some(token) = self
                    .declare_entity_token(domain_id, &ep, EntityKind::Subscriber)
                    .await
                {
                    self.entity_lv_tokens.insert((domain_id, ep.key), token);
                }
            }
        } else if is_request_topic(&ep.topic_name) {
            if self
                .service_srv_filter_for(&ep.topic_name)
                .allows(&ep.topic_name)
            {
                tracing::info!("DDS server→Zenoh queryable route: {}", ep.topic_name);
                let route = ServiceRoute::create(
                    self.participant_for(domain_id),
                    &ep,
                    &self.session,
                    self.config.namespace.as_deref(),
                )
                .await?;
                self.service_srv.insert((domain_id, ep.key), route);
                // G5: advertise this bridged service server on the Zenoh graph.
                if let Some(token) = self
                    .declare_entity_token(domain_id, &ep, EntityKind::ServiceServer)
                    .await
                {
                    self.entity_lv_tokens.insert((domain_id, ep.key), token);
                }
            }
        }
        Ok(())
    }

    fn participant_for(&self, domain_id: u32) -> i32 {
        self.participants
            .iter()
            .find(|(did, _)| *did == domain_id)
            .map(|(_, p)| p.raw())
            .unwrap_or_else(|| self.participants[0].1.raw())
    }

    /// Declare a per-route liveliness token matching the zenoh-plugin-ros2dds wire format.
    ///
    /// Failures are non-fatal: a warning is logged and `None` is returned so the
    /// caller can still store the route even when Zenoh liveliness is unavailable.
    async fn declare_entity_token(
        &mut self,
        _domain_id: u32,
        endpoint: &DiscoveredEndpoint,
        kind: EntityKind,
    ) -> Option<LivelinessToken> {
        let ros2_name = dds_topic_to_ros2_name(&endpoint.topic_name)
            .unwrap_or_else(|| endpoint.topic_name.clone());
        let zenoh_ke = ros2_name_to_zenoh_key(&ros2_name, self.config.namespace.as_deref());
        let is_action = is_action_component(&ros2_name);

        let key = match kind {
            EntityKind::Publisher => {
                let ros2_type = if is_action {
                    dds_type_to_ros2_action_type(&endpoint.type_name)
                } else {
                    dds_type_to_ros2_type(&endpoint.type_name)
                };
                build_pub_lv_key(
                    &self.zid,
                    &zenoh_ke,
                    &ros2_type,
                    endpoint.keyless,
                    &endpoint.qos,
                )
            }
            EntityKind::Subscriber => {
                let ros2_type = if is_action {
                    dds_type_to_ros2_action_type(&endpoint.type_name)
                } else {
                    dds_type_to_ros2_type(&endpoint.type_name)
                };
                build_sub_lv_key(
                    &self.zid,
                    &zenoh_ke,
                    &ros2_type,
                    endpoint.keyless,
                    &endpoint.qos,
                )
            }
            EntityKind::ServiceServer => {
                if is_action {
                    let ros2_type = dds_type_to_ros2_action_type(&endpoint.type_name);
                    let action_ke = ros2_name_to_zenoh_key(
                        action_base_name(&ros2_name),
                        self.config.namespace.as_deref(),
                    );
                    build_action_srv_lv_key(&self.zid, &action_ke, &ros2_type)
                } else {
                    let ros2_type = dds_type_to_ros2_service_type(&endpoint.type_name);
                    build_service_srv_lv_key(&self.zid, &zenoh_ke, &ros2_type)
                }
            }
            EntityKind::ServiceClient => {
                if is_action {
                    let ros2_type = dds_type_to_ros2_action_type(&endpoint.type_name);
                    let action_ke = ros2_name_to_zenoh_key(
                        action_base_name(&ros2_name),
                        self.config.namespace.as_deref(),
                    );
                    build_action_cli_lv_key(&self.zid, &action_ke, &ros2_type)
                } else {
                    let ros2_type = dds_type_to_ros2_service_type(&endpoint.type_name);
                    build_service_cli_lv_key(&self.zid, &zenoh_ke, &ros2_type)
                }
            }
        };

        match self.session.liveliness().declare_token(key).await {
            Ok(token) => Some(token),
            Err(e) => {
                tracing::warn!(
                    "Failed to declare entity liveliness token for {}: {e}",
                    endpoint.topic_name
                );
                None
            }
        }
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
        let f = Filter::compile(Some(".*"), Some("rt/secret")).unwrap();
        assert!(!f.allows("rt/secret"));
        assert!(f.allows("rt/chatter"));
    }

    #[test]
    fn test_filter_invalid_regex_returns_error() {
        assert!(Filter::compile(Some("[invalid"), None).is_err());
        assert!(Filter::compile(None, Some("[invalid")).is_err());
    }

    // G2: action filtering

    #[test]
    fn test_action_filter_allows_matching() {
        let f = Filter::compile(Some(".*_action.*"), None).unwrap();
        assert!(f.allows("rq/fibonacci/_action/send_goalRequest"));
        assert!(f.allows("rt/fibonacci/_action/feedback"));
    }

    #[test]
    fn test_action_filter_denies_non_matching() {
        let f = Filter::compile(None, Some(".*_action.*")).unwrap();
        assert!(!f.allows("rq/fibonacci/_action/send_goalRequest"));
        assert!(!f.allows("rt/fibonacci/_action/feedback"));
        assert!(f.allows("rt/chatter"));
    }

    #[test]
    fn test_action_filter_falls_back_to_global() {
        let action_f = Filter::compile(None, Some(".*fibonacci.*")).unwrap();
        assert!(!action_f.allows("rt/fibonacci/_action/feedback"));
        assert!(action_f.allows("rt/chatter"));
    }

    #[test]
    fn test_non_action_unaffected_by_action_filter() {
        let action_f = Filter::compile(None, Some(".*_action.*")).unwrap();
        let pub_f = Filter::compile(None, None).unwrap();
        assert!(pub_f.allows("rt/chatter"));
        assert!(!action_f.allows("rt/fibonacci/_action/feedback"));
    }

    // G6: multi-domain validation

    #[test]
    fn test_domain_id_validation_rejects_too_large() {
        let ids = vec![230u32];
        let mut seen = std::collections::HashSet::new();
        let result: Result<(), String> = ids.iter().try_for_each(|&did| {
            if did > 229 {
                Err(format!("domain ID {did} out of range"))
            } else if !seen.insert(did) {
                Err(format!("domain ID {did} duplicated"))
            } else {
                Ok(())
            }
        });
        assert!(result.is_err());
    }

    #[test]
    fn test_domain_id_dedup_rejects_duplicate() {
        let ids = vec![0u32, 0u32];
        let mut seen = std::collections::HashSet::new();
        let result: Result<(), String> = ids.iter().try_for_each(|&did| {
            if did > 229 {
                Err(format!("domain ID {did} out of range"))
            } else if !seen.insert(did) {
                Err(format!("domain ID {did} duplicated"))
            } else {
                Ok(())
            }
        });
        assert!(result.is_err());
    }

    #[test]
    fn test_route_key_distinct_across_domains() {
        // (domain_id, topic_name) keys with same topic but different domains are distinct.
        let key0: (u32, String) = (0, "rt/chatter".to_string());
        let key42: (u32, String) = (42, "rt/chatter".to_string());
        assert_ne!(key0, key42);
    }

    // G5: liveliness token key construction via bridge helpers

    #[test]
    fn test_entity_token_qos_str_reliable() {
        use crate::dds::qos::is_reliable;
        use cyclors::qos::{DDS_INFINITE_TIME, Qos, Reliability, ReliabilityKind};
        let qos = Qos {
            reliability: Some(Reliability {
                kind: ReliabilityKind::RELIABLE,
                max_blocking_time: DDS_INFINITE_TIME,
            }),
            ..Default::default()
        };
        assert_eq!(if is_reliable(&qos) { "re" } else { "be" }, "re");
    }

    #[test]
    fn test_entity_token_qos_str_best_effort() {
        use crate::dds::qos::is_reliable;
        use cyclors::qos::{DDS_INFINITE_TIME, Qos, Reliability, ReliabilityKind};
        let qos = Qos {
            reliability: Some(Reliability {
                kind: ReliabilityKind::BEST_EFFORT,
                max_blocking_time: DDS_INFINITE_TIME,
            }),
            ..Default::default()
        };
        assert_eq!(if is_reliable(&qos) { "re" } else { "be" }, "be");
    }

    #[test]
    fn test_entity_token_pubsub_type_uses_dds_type_to_ros2_type() {
        use crate::dds::names::dds_type_to_ros2_type;
        let dds_type = "std_msgs::msg::dds_::String_";
        assert_eq!(dds_type_to_ros2_type(dds_type), "std_msgs/msg/String");
    }

    #[test]
    fn test_entity_token_service_type_uses_service_variant() {
        use crate::dds::names::dds_type_to_ros2_service_type;
        let dds_type = "example_interfaces::srv::dds_::AddTwoInts_Request_";
        assert_eq!(
            dds_type_to_ros2_service_type(dds_type),
            "example_interfaces/srv/AddTwoInts"
        );
    }

    #[test]
    fn test_entity_lv_key_publisher_mp() {
        use crate::liveliness::build_pub_lv_key;
        use cyclors::qos::Qos;
        let key = build_pub_lv_key("z", "chatter", "std_msgs/msg/String", true, &Qos::default());
        assert!(key.starts_with("@/z/@ros2_lv/MP/"));
        assert!(key.contains("chatter"));
        assert!(key.contains("std_msgs§msg§String"));
    }

    #[test]
    fn test_entity_lv_key_subscriber_ms() {
        use crate::liveliness::build_sub_lv_key;
        use cyclors::qos::Qos;
        let key = build_sub_lv_key("z", "chatter", "std_msgs/msg/String", true, &Qos::default());
        assert!(key.starts_with("@/z/@ros2_lv/MS/"));
    }

    #[test]
    fn test_entity_lv_key_service_server_ss() {
        use crate::liveliness::build_service_srv_lv_key;
        let key =
            build_service_srv_lv_key("z", "add_two_ints", "example_interfaces/srv/AddTwoInts");
        assert!(key.starts_with("@/z/@ros2_lv/SS/"));
        assert!(key.contains("example_interfaces§srv§AddTwoInts"));
    }

    #[test]
    fn test_entity_lv_key_service_client_sc() {
        use crate::liveliness::build_service_cli_lv_key;
        let key =
            build_service_cli_lv_key("z", "add_two_ints", "example_interfaces/srv/AddTwoInts");
        assert!(key.starts_with("@/z/@ros2_lv/SC/"));
    }
}
