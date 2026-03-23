//! Top-level bridge orchestrator.
//!
//! Creates two Zenoh sessions (one for Humble, one for Jazzy), starts
//! discovery on both, and wires up forwarders as topics are discovered.
//!
//! # Forwarding strategy
//!
//! When a Humble publisher is discovered:
//!   - We forward: humble KE (TypeHashNotSupported) → jazzy KE (RIHS01_<hash>)
//!   - We also forward the reverse direction so Jazzy nodes can receive Humble data
//!
//! Services use queryable-based forwarding (see [`crate::forwarder`]).
//! Actions are independent sub-entities (send_goal, cancel_goal, get_result,
//! feedback, status) and are bridged individually via their KEs.

use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

use anyhow::Result;
use ros_z_protocol::{EntityKind, TypeHash};
use zenoh::{Wait, liveliness::LivelinessToken};

use crate::{
    discovery::{self, DiscoveryEvent, Distro},
    forwarder::{
        ForwarderHandle, ServiceForwarderHandle, start_forwarder, start_service_forwarder,
    },
    hash_registry,
    ke::rihs_string,
};

// ---------------------------------------------------------------------------
// BridgedEntry
// ---------------------------------------------------------------------------

/// Key identifying a unique bridged topic/service/action sub-entity.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct BridgeKey {
    /// Topic / service name (without domain prefix).
    topic: String,
    /// Fully-qualified ROS type name.
    type_name: String,
    /// Entity kind.
    kind: EntityKind,
}

/// All forwarder handles for a single bridged entity.
///
/// Dropping this struct tears down the corresponding forwarders and withdraws
/// the synthetic liveliness tokens from both sides.
struct BridgedEntry {
    _humble_to_jazzy: Option<ForwarderHandle>,
    _jazzy_to_humble: Option<ForwarderHandle>,
    _humble_service: Option<ServiceForwarderHandle>,
    _jazzy_service: Option<ServiceForwarderHandle>,
    /// Synthetic Jazzy liveliness token declared on the Jazzy session (re-announces a Humble entity).
    _jazzy_lv: Option<LivelinessToken>,
    /// Synthetic Humble liveliness token declared on the Humble session (re-announces a Jazzy entity).
    _humble_lv: Option<LivelinessToken>,
}

// ---------------------------------------------------------------------------
// Bridge
// ---------------------------------------------------------------------------

/// The bridge holds the two Zenoh sessions and drives the event loop.
pub struct Bridge {
    humble_session: Arc<zenoh::Session>,
    jazzy_session: Arc<zenoh::Session>,
    domain_id: usize,
    /// Active bridged entries, keyed by (topic, type, kind).
    active: Arc<Mutex<HashMap<BridgeKey, BridgedEntry>>>,
}

/// Rewrite the hash segment of a liveliness KE to Jazzy format.
///
/// Replaces the `TypeHashNotSupported` segment with the given `RIHS01_<hex>` string.
fn lv_ke_to_jazzy(raw_ke: &str, jazzy_hash: &TypeHash) -> String {
    let rihs = rihs_string(jazzy_hash);
    raw_ke
        .split('/')
        .map(|seg| {
            if seg == "TypeHashNotSupported" {
                rihs.as_str()
            } else {
                seg
            }
        })
        .collect::<Vec<_>>()
        .join("/")
}

/// Rewrite the hash segment of a liveliness KE to Humble format.
///
/// Replaces the `RIHS01_<hex>` segment with `TypeHashNotSupported`.
fn lv_ke_to_humble(raw_ke: &str) -> String {
    raw_ke
        .split('/')
        .map(|seg| {
            if seg.starts_with("RIHS01_") {
                "TypeHashNotSupported"
            } else {
                seg
            }
        })
        .collect::<Vec<_>>()
        .join("/")
}

/// Build a minimal Zenoh config connecting to a single endpoint.
fn build_session_config(endpoint: &str) -> Result<zenoh::Config> {
    let mut config = zenoh::Config::default();
    config
        .insert_json5("connect/endpoints", &format!("[\"{endpoint}\"]"))
        .map_err(|e| anyhow::anyhow!("config insert connect/endpoints: {e}"))?;
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .map_err(|e| anyhow::anyhow!("config insert scouting: {e}"))?;
    Ok(config)
}

impl Bridge {
    /// Create a new bridge connecting to `humble_endpoint` and `jazzy_endpoint`.
    pub async fn new(
        humble_endpoint: &str,
        jazzy_endpoint: &str,
        domain_id: usize,
    ) -> Result<Self> {
        let humble_cfg = build_session_config(humble_endpoint)?;
        let jazzy_cfg = build_session_config(jazzy_endpoint)?;

        let humble_session = Arc::new(
            zenoh::open(humble_cfg)
                .wait()
                .map_err(|e| anyhow::anyhow!("humble session open: {e}"))?,
        );
        let jazzy_session = Arc::new(
            zenoh::open(jazzy_cfg)
                .wait()
                .map_err(|e| anyhow::anyhow!("jazzy session open: {e}"))?,
        );

        Ok(Self {
            humble_session,
            jazzy_session,
            domain_id,
            active: Arc::new(Mutex::new(HashMap::new())),
        })
    }

    /// Run the bridge event loop until a shutdown signal is received.
    pub async fn run(&mut self) -> Result<()> {
        let mut humble_rx =
            discovery::start_discovery(self.humble_session.clone(), self.domain_id)?;
        let mut jazzy_rx = discovery::start_discovery(self.jazzy_session.clone(), self.domain_id)?;

        tracing::info!("Bridge running — waiting for discovery events");

        loop {
            tokio::select! {
                event = humble_rx.recv() => {
                    match event {
                        Some(e) => self.handle_event(e),
                        None => {
                            tracing::warn!("Humble discovery channel closed");
                            break;
                        }
                    }
                }
                event = jazzy_rx.recv() => {
                    match event {
                        Some(e) => self.handle_event(e),
                        None => {
                            tracing::warn!("Jazzy discovery channel closed");
                            break;
                        }
                    }
                }
                _ = tokio::signal::ctrl_c() => {
                    tracing::info!("Received SIGINT, shutting down");
                    break;
                }
            }
        }

        Ok(())
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    fn handle_event(&self, event: DiscoveryEvent) {
        use ros_z_protocol::Entity;

        let entity = match &*event.entity {
            Entity::Node(_) => return,
            Entity::Endpoint(ep) => ep.clone(),
        };

        let type_info = match &entity.type_info {
            Some(ti) => ti.clone(),
            None => return,
        };

        let key = BridgeKey {
            topic: entity.topic.clone(),
            type_name: type_info.name.clone(),
            kind: entity.kind,
        };

        if event.appeared {
            tracing::info!(
                "Entity appeared: {:?} {:?} topic={} type={}",
                event.distro,
                entity.kind,
                entity.topic,
                type_info.name
            );
            self.bridge_entity(key, &type_info.hash, event.distro, &event.raw_ke);
        } else {
            tracing::info!(
                "Entity disappeared: topic={} type={}",
                entity.topic,
                type_info.name
            );
            self.active.lock().unwrap().remove(&key);
        }
    }

    /// Set up forwarders for a discovered entity and re-announce it on the opposite side.
    fn bridge_entity(
        &self,
        key: BridgeKey,
        discovered_hash: &TypeHash,
        from_distro: Distro,
        raw_lv_ke: &str,
    ) {
        // Only bridge each (topic, type, kind) pair once.
        {
            let active = self.active.lock().unwrap();
            if active.contains_key(&key) {
                return;
            }
        }

        // Resolve the Jazzy hash. If the entity comes from Humble, the hash is
        // all-zero — look it up from the registry. If it comes from Jazzy, use
        // the provided hash directly.
        let jazzy_hash = match from_distro {
            Distro::Jazzy => discovered_hash.clone(),
            Distro::Humble => match hash_registry::lookup(&key.type_name) {
                Some(h) => h.clone(),
                None => {
                    tracing::warn!(
                        "Unknown type {} — cannot bridge (no hash in registry)",
                        key.type_name
                    );
                    return;
                }
            },
        };

        let d = self.domain_id;
        let topic_stripped = key.topic.strip_prefix('/').unwrap_or(&key.topic);

        // Build KEs for both sides.
        let humble_ke = format!(
            "{d}/{topic_stripped}/{}/TypeHashNotSupported",
            key.type_name
        );
        let jazzy_ke = format!(
            "{d}/{topic_stripped}/{}/{}",
            key.type_name,
            rihs_string(&jazzy_hash)
        );

        tracing::debug!("Bridging: humble={humble_ke}  jazzy={jazzy_ke}");

        // Re-announce the entity on the opposite side so `ros2 topic list` / graph
        // introspection sees it.  A Humble entity gets a synthetic Jazzy liveliness
        // token (RIHS01 hash) on the Jazzy session, and vice versa.
        let (jazzy_lv, humble_lv) = match from_distro {
            Distro::Humble => {
                let rewritten = lv_ke_to_jazzy(raw_lv_ke, &jazzy_hash);
                let token = self
                    .jazzy_session
                    .liveliness()
                    .declare_token(&rewritten)
                    .wait()
                    .map_err(|e| anyhow::anyhow!("declare jazzy lv token: {e}"));
                match token {
                    Ok(t) => {
                        tracing::debug!("Declared synthetic Jazzy liveliness token: {rewritten}");
                        (Some(t), None)
                    }
                    Err(e) => {
                        tracing::warn!("Failed to declare synthetic Jazzy lv token: {e}");
                        (None, None)
                    }
                }
            }
            Distro::Jazzy => {
                let rewritten = lv_ke_to_humble(raw_lv_ke);
                let token = self
                    .humble_session
                    .liveliness()
                    .declare_token(&rewritten)
                    .wait()
                    .map_err(|e| anyhow::anyhow!("declare humble lv token: {e}"));
                match token {
                    Ok(t) => {
                        tracing::debug!("Declared synthetic Humble liveliness token: {rewritten}");
                        (None, Some(t))
                    }
                    Err(e) => {
                        tracing::warn!("Failed to declare synthetic Humble lv token: {e}");
                        (None, None)
                    }
                }
            }
        };

        let entry = match key.kind {
            EntityKind::Publisher | EntityKind::Subscription => {
                self.setup_pubsub_bridge(&humble_ke, &jazzy_ke, jazzy_lv, humble_lv)
            }
            EntityKind::Service | EntityKind::Client => {
                self.setup_service_bridge(&humble_ke, &jazzy_ke, jazzy_lv, humble_lv)
            }
            EntityKind::Node => return,
        };

        match entry {
            Ok(e) => {
                self.active.lock().unwrap().insert(key, e);
            }
            Err(err) => {
                tracing::error!("Failed to set up bridge for {}: {err}", key.topic);
            }
        }
    }

    fn setup_pubsub_bridge(
        &self,
        humble_ke: &str,
        jazzy_ke: &str,
        jazzy_lv: Option<LivelinessToken>,
        humble_lv: Option<LivelinessToken>,
    ) -> Result<BridgedEntry> {
        let h2j = start_forwarder(
            self.humble_session.clone(),
            humble_ke.to_string(),
            self.jazzy_session.clone(),
            jazzy_ke.to_string(),
        )
        .map_err(|e| anyhow::anyhow!("humble→jazzy forwarder: {e}"))?;

        let j2h = start_forwarder(
            self.jazzy_session.clone(),
            jazzy_ke.to_string(),
            self.humble_session.clone(),
            humble_ke.to_string(),
        )
        .map_err(|e| anyhow::anyhow!("jazzy→humble forwarder: {e}"))?;

        Ok(BridgedEntry {
            _humble_to_jazzy: Some(h2j),
            _jazzy_to_humble: Some(j2h),
            _humble_service: None,
            _jazzy_service: None,
            _jazzy_lv: jazzy_lv,
            _humble_lv: humble_lv,
        })
    }

    fn setup_service_bridge(
        &self,
        humble_ke: &str,
        jazzy_ke: &str,
        jazzy_lv: Option<LivelinessToken>,
        humble_lv: Option<LivelinessToken>,
    ) -> Result<BridgedEntry> {
        // Proxy queries from Jazzy clients to the Humble server.
        let humble_svc = start_service_forwarder(
            self.jazzy_session.clone(),
            jazzy_ke.to_string(),
            self.humble_session.clone(),
            humble_ke.to_string(),
        )
        .map_err(|e| anyhow::anyhow!("jazzy→humble service: {e}"))?;

        // Proxy queries from Humble clients to the Jazzy server.
        let jazzy_svc = start_service_forwarder(
            self.humble_session.clone(),
            humble_ke.to_string(),
            self.jazzy_session.clone(),
            jazzy_ke.to_string(),
        )
        .map_err(|e| anyhow::anyhow!("humble→jazzy service: {e}"))?;

        Ok(BridgedEntry {
            _humble_to_jazzy: None,
            _jazzy_to_humble: None,
            _humble_service: Some(humble_svc),
            _jazzy_service: Some(jazzy_svc),
            _jazzy_lv: jazzy_lv,
            _humble_lv: humble_lv,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bridge_key_equality() {
        let k1 = BridgeKey {
            topic: "/chatter".to_string(),
            type_name: "std_msgs::msg::dds_::String_".to_string(),
            kind: EntityKind::Publisher,
        };
        let k2 = k1.clone();
        assert_eq!(k1, k2);
    }

    const HUMBLE_LV_KE: &str = "@ros2_lv/0/aabbccdd1122334455667788aabbccdd/1/2/P//chatter/std_msgs::msg::dds_::String_/TypeHashNotSupported/1";
    const JAZZY_LV_KE: &str = "@ros2_lv/0/aabbccdd1122334455667788aabbccdd/1/2/P//chatter/std_msgs::msg::dds_::String_/RIHS01_abababababababababababababababababababababababababababababababababab/1";

    #[test]
    fn lv_ke_humble_to_jazzy() {
        let hash = TypeHash::new(1, [0xabu8; 32]);
        let rewritten = lv_ke_to_jazzy(HUMBLE_LV_KE, &hash);
        assert!(rewritten.contains("RIHS01_"));
        assert!(!rewritten.contains("TypeHashNotSupported"));
        // All other segments preserved
        assert!(rewritten.contains("@ros2_lv/0/"));
        assert!(rewritten.contains("std_msgs::msg::dds_::String_"));
    }

    #[test]
    fn lv_ke_jazzy_to_humble() {
        let rewritten = lv_ke_to_humble(JAZZY_LV_KE);
        assert!(rewritten.contains("TypeHashNotSupported"));
        assert!(!rewritten.contains("RIHS01_"));
        assert!(rewritten.contains("@ros2_lv/0/"));
        assert!(rewritten.contains("std_msgs::msg::dds_::String_"));
    }

    #[test]
    fn lv_ke_roundtrip() {
        let hash = TypeHash::new(1, [0xabu8; 32]);
        // Humble → Jazzy → Humble should restore the original (modulo the hash value).
        let jazzy = lv_ke_to_jazzy(HUMBLE_LV_KE, &hash);
        let back = lv_ke_to_humble(&jazzy);
        assert_eq!(back, HUMBLE_LV_KE);
    }
}
