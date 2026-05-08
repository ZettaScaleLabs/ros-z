//! ZDdsBridge — automatic DDS↔Zenoh discovery and route management.

use std::{collections::HashMap, time::Duration};

use anyhow::{Result, anyhow};
use regex::Regex;
use ros_z::node::ZNode;

use crate::{
    discovery::{DiscoveredEndpoint, DiscoveryEvent},
    gid::Gid,
    names::{
        dds_topic_to_ros2_name, dds_type_to_ros2_action_type, dds_type_to_ros2_service_type,
        dds_type_to_ros2_type, is_action_get_result_topic, is_pubsub_topic, is_request_topic,
        type_info_from_user_data,
    },
    participant::{BridgeQos, DdsParticipant},
    pubsub::{ZDdsPubBridge, ZDdsSubBridge},
    ros_discovery::RosDiscoveryPublisher,
    service::{ZDdsClientBridge, ZDdsServiceBridge},
};

// ─── Filter ───────────────────────────────────────────────────────────────────

struct Filter {
    allow: Option<Regex>,
    deny: Option<Regex>,
}

impl Filter {
    fn new(allow: Option<&str>, deny: Option<&str>) -> Result<Self> {
        let allow = allow
            .map(Regex::new)
            .transpose()
            .map_err(|e| anyhow!("invalid allow regex: {e}"))?;
        let deny = deny
            .map(Regex::new)
            .transpose()
            .map_err(|e| anyhow!("invalid deny regex: {e}"))?;
        Ok(Self { allow, deny })
    }

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

// ─── Route sets (opaque RAII handles) ────────────────────────────────────────

enum PubRoute<P: DdsParticipant> {
    Pub(ZDdsPubBridge<P>),
}

enum SubRoute<P: DdsParticipant> {
    Sub(ZDdsSubBridge<P>),
}

// GIDs of the DDS reader(s) and writer(s) created for a single named route.
// Stored alongside routes so they can be unregistered from ros_discovery_info on removal.
#[derive(Default)]
struct RouteGids {
    readers: Vec<Gid>,
    writers: Vec<Gid>,
}

// ─── ZDdsBridgeBuilder ────────────────────────────────────────────────────────

/// Builder for `ZDdsBridge` — configure filters before starting discovery.
pub struct ZDdsBridgeBuilder<P: DdsParticipant> {
    node: ZNode,
    participant: P,
    allow_topics: Option<String>,
    deny_topics: Option<String>,
    service_timeout: Duration,
    action_get_result_timeout: Duration,
    transient_local_cache_multiplier: usize,
}

impl<P: DdsParticipant> ZDdsBridgeBuilder<P> {
    /// Allow only topics/services whose DDS topic name matches `regex`.
    pub fn allow_topics_regex(mut self, regex: Option<&str>) -> Self {
        self.allow_topics = regex.map(str::to_owned);
        self
    }

    /// Deny topics/services whose DDS topic name matches `regex`.
    pub fn deny_topics_regex(mut self, regex: Option<&str>) -> Self {
        self.deny_topics = regex.map(str::to_owned);
        self
    }

    /// Timeout for service queryable replies (default 10 s).
    pub fn service_timeout(mut self, d: Duration) -> Self {
        self.service_timeout = d;
        self
    }

    /// Timeout for action get_result queryable replies (default 300 s).
    pub fn action_get_result_timeout(mut self, d: Duration) -> Self {
        self.action_get_result_timeout = d;
        self
    }

    /// TRANSIENT_LOCAL AdvancedPublisher cache depth multiplier (default 10).
    pub fn transient_local_cache_multiplier(mut self, n: usize) -> Self {
        self.transient_local_cache_multiplier = n;
        self
    }

    /// Start the discovery loop and run until all discovery channels close.
    pub async fn run(self) -> Result<()> {
        let filter = Filter::new(self.allow_topics.as_deref(), self.deny_topics.as_deref())?;
        let ros_discovery =
            RosDiscoveryPublisher::new(&self.participant, self.node.namespace(), self.node.name())
                .map_err(|e| tracing::warn!("ros_discovery_info publisher failed to start: {e}"))
                .ok();
        let bridge = ZDdsBridge {
            node: self.node,
            participant: self.participant,
            pub_routes: HashMap::new(),
            sub_routes: HashMap::new(),
            srv_routes: HashMap::new(),
            cli_routes: HashMap::new(),
            gid_to_name: HashMap::new(),
            route_gids: HashMap::new(),
            ros_discovery,
            filter,
            service_timeout: self.service_timeout,
            action_get_result_timeout: self.action_get_result_timeout,
            cache_multiplier: self.transient_local_cache_multiplier,
        };
        bridge.run_loop().await
    }
}

// ─── ZDdsBridge ──────────────────────────────────────────────────────────────

/// Full auto-discovery DDS↔Zenoh bridge.
///
/// Watches one DDS participant for discovered publications and subscriptions and
/// creates the appropriate `ZDdsPubBridge`, `ZDdsSubBridge`, `ZDdsServiceBridge`,
/// or `ZDdsClientBridge` for each.
///
/// Construct via `ZDdsBridge::new(node, participant)` which returns a builder;
/// call `.run().await` to start the discovery loop.
pub struct ZDdsBridge<P: DdsParticipant> {
    node: ZNode,
    participant: P,

    /// DDS publisher → Zenoh publisher bridges, keyed by ros2_name.
    pub_routes: HashMap<String, PubRoute<P>>,
    /// Zenoh subscriber → DDS writer bridges, keyed by ros2_name.
    sub_routes: HashMap<String, SubRoute<P>>,
    /// DDS service server → Zenoh queryable bridges, keyed by ros2_name.
    srv_routes: HashMap<String, ZDdsServiceBridge<P>>,
    /// DDS service client → Zenoh querier bridges, keyed by ros2_name.
    cli_routes: HashMap<String, ZDdsClientBridge<P>>,

    /// Reverse map: gid → ros2_name, for O(1) removal on undiscovery.
    gid_to_name: HashMap<Gid, String>,

    /// GIDs of DDS readers/writers per route name — for ros_discovery_info unregistration.
    route_gids: HashMap<String, RouteGids>,

    /// Publishes `ros_discovery_info` so `ros2 topic/node/service list` can see bridge endpoints.
    ros_discovery: Option<RosDiscoveryPublisher>,

    filter: Filter,
    service_timeout: Duration,
    action_get_result_timeout: Duration,
    cache_multiplier: usize,
}

impl<P: DdsParticipant> ZDdsBridge<P> {
    /// Create a new bridge builder for `node` and `participant`.
    pub fn new(node: ZNode, participant: P) -> ZDdsBridgeBuilder<P> {
        ZDdsBridgeBuilder {
            node,
            participant,
            allow_topics: None,
            deny_topics: None,
            service_timeout: Duration::from_secs(10),
            action_get_result_timeout: Duration::from_secs(300),
            transient_local_cache_multiplier: 10,
        }
    }

    async fn run_loop(mut self) -> Result<()> {
        let rx = self.participant.run_discovery();
        tracing::info!("ZDdsBridge: discovery loop started");
        while let Ok(event) = rx.recv_async().await {
            if let Err(e) = self.handle_event(event).await {
                tracing::warn!("ZDdsBridge: route error: {e}");
            }
        }
        tracing::info!("ZDdsBridge: discovery channel closed, shutting down");
        Ok(())
    }

    async fn handle_event(&mut self, event: DiscoveryEvent) -> Result<()> {
        match event {
            DiscoveryEvent::DiscoveredPublication(ep) => {
                self.on_publication(ep).await?;
            }
            DiscoveryEvent::UndiscoveredPublication(gid) => {
                if let Some(name) = self.gid_to_name.remove(&gid) {
                    self.pub_routes.remove(&name);
                    self.cli_routes.remove(&name);
                    self.unregister_route_gids(&name);
                    tracing::debug!("ZDdsBridge: removed pub/cli route for {name}");
                }
            }
            DiscoveryEvent::DiscoveredSubscription(ep) => {
                self.on_subscription(ep).await?;
            }
            DiscoveryEvent::UndiscoveredSubscription(gid) => {
                if let Some(name) = self.gid_to_name.remove(&gid) {
                    self.sub_routes.remove(&name);
                    self.srv_routes.remove(&name);
                    self.unregister_route_gids(&name);
                    tracing::debug!("ZDdsBridge: removed sub/srv route for {name}");
                }
            }
        }
        Ok(())
    }

    fn register_route_gids(&mut self, name: &str, gids: RouteGids) {
        if let Some(rd) = &self.ros_discovery {
            for g in &gids.readers {
                rd.add_reader(*g);
            }
            for g in &gids.writers {
                rd.add_writer(*g);
            }
        }
        self.route_gids.insert(name.to_string(), gids);
    }

    fn unregister_route_gids(&mut self, name: &str) {
        if let Some(gids) = self.route_gids.remove(name) {
            if let Some(rd) = &self.ros_discovery {
                for g in gids.readers {
                    rd.remove_reader(g);
                }
                for g in gids.writers {
                    rd.remove_writer(g);
                }
            }
        }
    }

    async fn on_publication(&mut self, ep: DiscoveredEndpoint) -> Result<()> {
        if !self.filter.allows(&ep.topic_name) {
            return Ok(());
        }

        if is_pubsub_topic(&ep.topic_name) {
            let ros2_name = match dds_topic_to_ros2_name(&ep.topic_name) {
                Some(n) => n,
                None => return Ok(()),
            };
            if self.pub_routes.contains_key(&ros2_name) {
                return Ok(());
            }
            let ros2_type = dds_type_to_ros2_type(&ep.type_name);
            let ud = ep.qos.user_data.as_deref().unwrap_or(&[]);
            let type_info = type_info_from_user_data(&ros2_type, ud);
            let type_hash = type_info.as_ref().map(|ti| ti.hash.clone());
            let qos = BridgeQos {
                reliability: ep.qos.reliability.clone(),
                durability: ep.qos.durability.clone(),
                history: ep.qos.history.clone(),
                durability_service: ep.qos.durability_service.clone(),
                ..Default::default()
            };
            let bridge = ZDdsPubBridge::new(
                &self.node,
                &ros2_name,
                &ros2_type,
                type_hash,
                &self.participant,
                qos,
                ep.keyless,
                self.cache_multiplier,
            )
            .await?;
            let gids = RouteGids {
                readers: bridge.reader_guid().into_iter().collect(),
                writers: vec![],
            };
            self.pub_routes
                .insert(ros2_name.clone(), PubRoute::Pub(bridge));
            self.register_route_gids(&ros2_name, gids);
            self.gid_to_name.insert(ep.key, ros2_name);
        } else if is_request_topic(&ep.topic_name) {
            // DDS service client: request writer discovered → create ZDdsClientBridge
            let ros2_name = match dds_topic_to_ros2_name(&ep.topic_name) {
                Some(n) => n,
                None => return Ok(()),
            };
            if self.cli_routes.contains_key(&ros2_name) {
                return Ok(());
            }
            let ros2_type = if is_action_get_result_topic(&ep.topic_name) {
                dds_type_to_ros2_action_type(&ep.type_name)
            } else {
                dds_type_to_ros2_service_type(&ep.type_name)
            };
            let timeout = if is_action_get_result_topic(&ep.topic_name) {
                self.action_get_result_timeout
            } else {
                self.service_timeout
            };
            let ud = ep.qos.user_data.as_deref().unwrap_or(&[]);
            let type_info = type_info_from_user_data(&ros2_type, ud);
            let type_hash = type_info.as_ref().map(|ti| ti.hash.clone());
            let bridge = ZDdsClientBridge::new(
                &self.node,
                &ros2_name,
                &ros2_type,
                type_hash,
                &self.participant,
                BridgeQos::default(),
                timeout,
            )
            .await?;
            let gids = RouteGids {
                readers: bridge.reader_guid().into_iter().collect(),
                writers: bridge.writer_guid().into_iter().collect(),
            };
            self.cli_routes.insert(ros2_name.clone(), bridge);
            self.register_route_gids(&ros2_name, gids);
            self.gid_to_name.insert(ep.key, ros2_name);
        }
        Ok(())
    }

    async fn on_subscription(&mut self, ep: DiscoveredEndpoint) -> Result<()> {
        if !self.filter.allows(&ep.topic_name) {
            return Ok(());
        }

        if is_pubsub_topic(&ep.topic_name) {
            let ros2_name = match dds_topic_to_ros2_name(&ep.topic_name) {
                Some(n) => n,
                None => return Ok(()),
            };
            if self.sub_routes.contains_key(&ros2_name) {
                return Ok(());
            }
            let ros2_type = dds_type_to_ros2_type(&ep.type_name);
            let ud = ep.qos.user_data.as_deref().unwrap_or(&[]);
            let type_info = type_info_from_user_data(&ros2_type, ud);
            let type_hash = type_info.as_ref().map(|ti| ti.hash.clone());
            let qos = BridgeQos {
                reliability: ep.qos.reliability.clone(),
                durability: ep.qos.durability.clone(),
                history: ep.qos.history.clone(),
                durability_service: ep.qos.durability_service.clone(),
                ..Default::default()
            };
            let bridge = ZDdsSubBridge::new(
                &self.node,
                &ros2_name,
                &ros2_type,
                type_hash,
                &self.participant,
                qos,
                ep.keyless,
            )
            .await?;
            let gids = RouteGids {
                readers: vec![],
                writers: bridge.writer_guid().into_iter().collect(),
            };
            self.sub_routes
                .insert(ros2_name.clone(), SubRoute::Sub(bridge));
            self.register_route_gids(&ros2_name, gids);
            self.gid_to_name.insert(ep.key, ros2_name);
        } else if is_request_topic(&ep.topic_name) {
            // DDS service server: request reader discovered → create ZDdsServiceBridge
            let ros2_name = match dds_topic_to_ros2_name(&ep.topic_name) {
                Some(n) => n,
                None => return Ok(()),
            };
            if self.srv_routes.contains_key(&ros2_name) {
                return Ok(());
            }
            let ros2_type = if is_action_get_result_topic(&ep.topic_name) {
                dds_type_to_ros2_action_type(&ep.type_name)
            } else {
                dds_type_to_ros2_service_type(&ep.type_name)
            };
            let ud = ep.qos.user_data.as_deref().unwrap_or(&[]);
            let type_info = type_info_from_user_data(&ros2_type, ud);
            let type_hash = type_info.as_ref().map(|ti| ti.hash.clone());
            let bridge = ZDdsServiceBridge::new(
                &self.node,
                &ros2_name,
                &ros2_type,
                type_hash,
                &self.participant,
                BridgeQos::default(),
                self.service_timeout,
            )
            .await?;
            let gids = RouteGids {
                readers: bridge.reader_guid().into_iter().collect(),
                writers: bridge.writer_guid().into_iter().collect(),
            };
            self.srv_routes.insert(ros2_name.clone(), bridge);
            self.register_route_gids(&ros2_name, gids);
            self.gid_to_name.insert(ep.key, ros2_name);
        }
        Ok(())
    }
}

// ─── tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::Filter;

    #[test]
    fn test_filter_no_rules_allows_all() {
        let f = Filter::new(None, None).unwrap();
        assert!(f.allows("rt/chatter"));
        assert!(f.allows("rq/add_two_intsRequest"));
    }

    #[test]
    fn test_filter_allow_only_matching() {
        let f = Filter::new(Some("rt/chatter"), None).unwrap();
        assert!(f.allows("rt/chatter"));
        assert!(!f.allows("rt/other"));
    }

    #[test]
    fn test_filter_deny_blocks() {
        let f = Filter::new(None, Some("rt/private.*")).unwrap();
        assert!(!f.allows("rt/private_topic"));
        assert!(f.allows("rt/chatter"));
    }

    #[test]
    fn test_filter_deny_overrides_allow() {
        let f = Filter::new(Some(".*"), Some("rt/secret")).unwrap();
        assert!(!f.allows("rt/secret"));
        assert!(f.allows("rt/chatter"));
    }

    #[test]
    fn test_filter_invalid_regex() {
        assert!(Filter::new(Some("[invalid"), None).is_err());
        assert!(Filter::new(None, Some("[invalid")).is_err());
    }

    #[test]
    fn test_dedup_by_name_same_route_skipped() {
        use std::collections::HashMap;
        let mut routes: HashMap<String, &str> = HashMap::new();
        let name = "/add_two_ints".to_string();

        if !routes.contains_key(&name) {
            routes.insert(name.clone(), "route_a");
        }
        if !routes.contains_key(&name) {
            routes.insert(name.clone(), "route_b");
        }

        assert_eq!(routes.len(), 1);
        assert_eq!(routes[&name], "route_a");
    }

    #[test]
    fn test_dedup_different_names_both_inserted() {
        use std::collections::HashMap;
        let mut routes: HashMap<String, &str> = HashMap::new();

        for name in ["/service_a", "/service_b"] {
            if !routes.contains_key(name) {
                routes.insert(name.to_string(), "route");
            }
        }
        assert_eq!(routes.len(), 2);
    }

    #[test]
    fn test_gid_reverse_map_removal() {
        use crate::gid::Gid;
        use std::collections::HashMap;

        let gid = Gid::from([1u8; 16]);
        let name = "/chatter".to_string();

        let mut routes: HashMap<String, &str> = HashMap::new();
        let mut gid_to_name: HashMap<Gid, String> = HashMap::new();

        routes.insert(name.clone(), "route");
        gid_to_name.insert(gid, name.clone());

        if let Some(removed_name) = gid_to_name.remove(&gid) {
            routes.remove(&removed_name);
        }

        assert!(routes.is_empty());
        assert!(gid_to_name.is_empty());
    }
}
