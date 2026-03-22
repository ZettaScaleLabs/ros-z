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
use zenoh::Wait;

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
/// Dropping this struct tears down the corresponding forwarders.
struct BridgedEntry {
    _humble_to_jazzy: Option<ForwarderHandle>,
    _jazzy_to_humble: Option<ForwarderHandle>,
    _humble_service: Option<ServiceForwarderHandle>,
    _jazzy_service: Option<ServiceForwarderHandle>,
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
            self.bridge_entity(
                key,
                entity.kind,
                &entity.topic,
                &type_info.name,
                &type_info.hash,
                event.distro,
            );
        } else {
            tracing::info!(
                "Entity disappeared: topic={} type={}",
                entity.topic,
                type_info.name
            );
            self.active.lock().unwrap().remove(&key);
        }
    }

    /// Set up forwarders for a discovered entity.
    fn bridge_entity(
        &self,
        key: BridgeKey,
        kind: EntityKind,
        topic: &str,
        type_name: &str,
        discovered_hash: &TypeHash,
        from_distro: Distro,
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
            Distro::Humble => match hash_registry::lookup(type_name) {
                Some(h) => h.clone(),
                None => {
                    tracing::warn!(
                        "Unknown type {type_name} — cannot bridge (no hash in registry)"
                    );
                    return;
                }
            },
        };

        let d = self.domain_id;
        let topic_stripped = topic.strip_prefix('/').unwrap_or(topic);

        // Build KEs for both sides.
        let humble_ke = format!("{d}/{topic_stripped}/{type_name}/TypeHashNotSupported");
        let jazzy_ke = format!(
            "{d}/{topic_stripped}/{type_name}/{}",
            rihs_string(&jazzy_hash)
        );

        tracing::debug!("Bridging: humble={humble_ke}  jazzy={jazzy_ke}");

        let entry = match kind {
            EntityKind::Publisher | EntityKind::Subscription => {
                self.setup_pubsub_bridge(&humble_ke, &jazzy_ke)
            }
            EntityKind::Service | EntityKind::Client => {
                self.setup_service_bridge(&humble_ke, &jazzy_ke)
            }
            EntityKind::Node => return,
        };

        match entry {
            Ok(e) => {
                self.active.lock().unwrap().insert(key, e);
            }
            Err(err) => {
                tracing::error!("Failed to set up bridge for {topic}: {err}");
            }
        }
    }

    fn setup_pubsub_bridge(&self, humble_ke: &str, jazzy_ke: &str) -> Result<BridgedEntry> {
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
        })
    }

    fn setup_service_bridge(&self, humble_ke: &str, jazzy_ke: &str) -> Result<BridgedEntry> {
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
}
