//! ROS 2 Zenoh configuration builders
//! Generates rmw_zenoh_cpp compatible configs programmatically

use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(Serialize, Deserialize, Clone)]
pub struct ConfigOverride {
    pub key: &'static str,
    pub value: Value,
    pub reason: &'static str,
}

// ROUTER CONFIG - Based on rmw_zenoh_cpp DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5
pub fn router_overrides() -> Vec<ConfigOverride> {
    vec![
    ConfigOverride {
        key: "mode",
        value: serde_json::json!("router"),
        reason: "Router mode required for ROS 2 discovery/routing",
    },
    ConfigOverride {
        key: "listen/endpoints",
        value: serde_json::json!(["tcp/[::]:7447"]),
        reason: "Standard ROS 2 port, IPv6 wildcard for all interfaces",
    },
    ConfigOverride {
        key: "connect/endpoints",
        value: serde_json::json!([]),
        reason: "Router does not connect to other endpoints",
    },
    ConfigOverride {
        key: "scouting/multicast/enabled",
        value: serde_json::json!(false),
        reason: "Disable multicast discovery by default - router uses TCP gossip",
    },
    ConfigOverride {
        key: "scouting/gossip/target",
        value: serde_json::json!({"router": ["router", "peer"], "peer": ["router"]}),
        reason: "Peers send gossip messages only to router to avoid unnecessary traffic",
    },
    ConfigOverride {
        key: "timestamping/enabled",
        value: serde_json::json!({"router": true, "peer": true, "client": true}),
        reason: "Required for PublicationCache/transient_local durability",
    },
    ConfigOverride {
        key: "queries_default_timeout",
        value: serde_json::json!(600000),
        reason: "10 minutes timeout for service calls at launch with many nodes",
    },
    ConfigOverride {
        key: "routing/router/peers_failover_brokering",
        value: serde_json::json!(false),
        reason: "Disabled - no purpose when peers connect directly, reduces overhead",
    },
    ConfigOverride {
        key: "transport/unicast/open_timeout",
        value: serde_json::json!(60000),
        reason: "Increased timeout to avoid issues at launch with many nodes",
    },
    ConfigOverride {
        key: "transport/unicast/accept_timeout",
        value: serde_json::json!(60000),
        reason: "Increased timeout to avoid issues at launch with many nodes",
    },
    ConfigOverride {
        key: "transport/unicast/accept_pending",
        value: serde_json::json!(10000),
        reason: "Support large number of nodes starting together",
    },
    ConfigOverride {
        key: "transport/unicast/max_sessions",
        value: serde_json::json!(10000),
        reason: "Support large number of nodes starting together",
    },
    ConfigOverride {
        key: "transport/link/tx/lease",
        value: serde_json::json!(60000),
        reason: "Avoid lease expiration at launch with many nodes",
    },
    ConfigOverride {
        key: "transport/link/tx/keep_alive",
        value: serde_json::json!(2),
        reason: "Decreased for loopback where keep-alive messages are less likely lost",
    },
    ConfigOverride {
        key: "transport/link/tx/queue/congestion_control/block/wait_before_close",
        value: serde_json::json!(5000000),
        reason: "Router routes to outside robot possibly over WiFi, lower value prevents long blocks",
    },
    ConfigOverride {
        key: "transport/shared_memory/enabled",
        value: serde_json::json!(false),
        reason: "Disabled by default until fully tested",
    },
    ]
}

// SESSION CONFIG - Based on rmw_zenoh_cpp DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5
pub fn session_overrides() -> Vec<ConfigOverride> {
    vec![
    ConfigOverride {
        key: "mode",
        value: serde_json::json!("peer"),
        reason: "Peer mode connects to router (not router itself)",
    },
    ConfigOverride {
        key: "connect/endpoints",
        value: serde_json::json!(["tcp/localhost:7447"]),
        reason: "Connect to router at standard ROS 2 location",
    },
    ConfigOverride {
        key: "listen/endpoints",
        value: serde_json::json!(["tcp/localhost:0"]),
        reason: "Localhost only - ROS nodes don't accept external connections",
    },
    ConfigOverride {
        key: "scouting/multicast/enabled",
        value: serde_json::json!(false),
        reason: "Peers use TCP to router, no multicast needed",
    },
    ConfigOverride {
        key: "scouting/multicast/autoconnect_strategy",
        value: serde_json::json!({"peer": {"to_router": "always", "to_peer": "greater-zid"}}),
        reason: "All peers interconnect over loopback, greater-zid avoids double connections",
    },
    ConfigOverride {
        key: "scouting/gossip/target",
        value: serde_json::json!({"router": ["router", "peer"], "peer": ["router"]}),
        reason: "Peers send gossip messages only to router to avoid unnecessary traffic",
    },
    ConfigOverride {
        key: "scouting/gossip/autoconnect_strategy",
        value: serde_json::json!({"peer": {"to_router": "always", "to_peer": "greater-zid"}}),
        reason: "All peers reachable on loopback, greater-zid avoids double connections at startup",
    },
    ConfigOverride {
        key: "timestamping/enabled",
        value: serde_json::json!({"router": true, "peer": true, "client": true}),
        reason: "Required for PublicationCache/transient_local durability",
    },
    ConfigOverride {
        key: "queries_default_timeout",
        value: serde_json::json!(600000),
        reason: "10 minutes timeout for service calls at launch with many nodes",
    },
    ConfigOverride {
        key: "routing/router/peers_failover_brokering",
        value: serde_json::json!(true),
        reason: "Enable failover brokering for peer sessions",
    },
    ConfigOverride {
        key: "transport/unicast/open_timeout",
        value: serde_json::json!(60000),
        reason: "Increased timeout to avoid issues at launch with many nodes",
    },
    ConfigOverride {
        key: "transport/unicast/accept_timeout",
        value: serde_json::json!(60000),
        reason: "Increased timeout to avoid issues at launch with many nodes",
    },
    ConfigOverride {
        key: "transport/unicast/accept_pending",
        value: serde_json::json!(10000),
        reason: "Support large number of nodes starting together",
    },
    ConfigOverride {
        key: "transport/unicast/max_sessions",
        value: serde_json::json!(10000),
        reason: "Support large number of nodes starting together",
    },
    ConfigOverride {
        key: "transport/link/tx/lease",
        value: serde_json::json!(60000),
        reason: "Avoid lease expiration at launch with many nodes",
    },
    ConfigOverride {
        key: "transport/link/tx/keep_alive",
        value: serde_json::json!(2),
        reason: "Decreased for loopback where keep-alive messages are less likely lost",
    },
    ConfigOverride {
        key: "transport/link/tx/queue/congestion_control/block/wait_before_close",
        value: serde_json::json!(60000000),
        reason: "Increased to avoid link closure at launch where congestion likely on loopback",
    },
    ConfigOverride {
        key: "transport/shared_memory/enabled",
        value: serde_json::json!(false),
        reason: "Disabled by default until fully tested",
    },
    ]
}

/// Build-time JSON5 generator with comments
pub fn generate_json5(overrides: &[ConfigOverride], name: &str) -> String {
    let mut output = format!("// GENERATED: {} - DO NOT EDIT\n", name);
    output.push_str("// This file is auto-generated from ros-z/src/config.rs\n");
    output.push_str("// Edit the source file and rebuild to make changes\n");
    output.push_str("{\n");

    for (i, override_) in overrides.iter().enumerate() {
        output.push_str(&format!("  // {}\n", override_.reason));
        output.push_str(&format!("  \"{}\": ", override_.key));
        let value_str = serde_json::to_string_pretty(&override_.value)
            .unwrap()
            .replace('\n', "\n  ");
        output.push_str(&value_str);
        output.push_str(if i < overrides.len() - 1 { ",\n\n" } else { "\n" });
    }

    output.push_str("}\n");
    output
}

/// Build a Zenoh config from a set of overrides
fn build_config(overrides: &[ConfigOverride]) -> zenoh::Result<zenoh::Config> {
    let mut config = zenoh::Config::default();
    for override_ in overrides {
        let value_str = serde_json::to_string(&override_.value)?;
        config.insert_json5(override_.key, &value_str)?;
    }
    Ok(config)
}

/// Create a router configuration matching rmw_zenoh_cpp
pub fn router_config() -> zenoh::Result<zenoh::Config> {
    build_config(&router_overrides())
}

/// Create a session configuration matching rmw_zenoh_cpp
pub fn session_config() -> zenoh::Result<zenoh::Config> {
    build_config(&session_overrides())
}

/// Builder for router configuration with customization options
#[derive(Clone)]
pub struct RouterConfigBuilder {
    overrides: Vec<ConfigOverride>,
}

impl RouterConfigBuilder {
    /// Create a new router config builder with default ROS settings
    pub fn new() -> Self {
        Self {
            overrides: router_overrides(),
        }
    }

    /// Change the listen port (default: 7447)
    pub fn with_listen_port(mut self, port: u16) -> Self {
        if let Some(listen) = self.overrides.iter_mut().find(|o| o.key == "listen/endpoints") {
            listen.value = serde_json::json!([format!("tcp/[::]:{}", port)]);
        }
        self
    }

    /// Set a custom listen endpoint
    pub fn with_listen_endpoint(mut self, endpoint: &str) -> Self {
        if let Some(listen) = self.overrides.iter_mut().find(|o| o.key == "listen/endpoints") {
            listen.value = serde_json::json!([endpoint]);
        }
        self
    }

    /// Build the Zenoh config
    pub fn build(self) -> zenoh::Result<zenoh::Config> {
        build_config(&self.overrides)
    }
}

impl Default for RouterConfigBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Builder for session configuration with customization options
#[derive(Clone)]
pub struct SessionConfigBuilder {
    overrides: Vec<ConfigOverride>,
}

impl SessionConfigBuilder {
    /// Create a new session config builder with default ROS settings
    pub fn new() -> Self {
        Self {
            overrides: session_overrides(),
        }
    }

    /// Change the router endpoint to connect to (default: tcp/localhost:7447)
    pub fn with_router_endpoint(mut self, endpoint: &str) -> Self {
        if let Some(connect) = self.overrides.iter_mut().find(|o| o.key == "connect/endpoints") {
            connect.value = serde_json::json!([endpoint]);
        }
        self
    }

    /// Build the Zenoh config
    pub fn build(self) -> zenoh::Result<zenoh::Config> {
        build_config(&self.overrides)
    }
}

impl Default for SessionConfigBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_router_config() {
        let config = router_config().expect("Failed to build router config");
        // Verify mode is router
        assert_eq!(
            config.mode().unwrap().to_string(),
            "router"
        );
    }

    #[test]
    fn test_session_config() {
        let config = session_config().expect("Failed to build session config");
        // Verify mode is peer
        assert_eq!(
            config.mode().unwrap().to_string(),
            "peer"
        );
    }

    #[test]
    fn test_router_builder_custom_port() {
        let config = RouterConfigBuilder::new()
            .with_listen_port(7448)
            .build()
            .expect("Failed to build router config");
        assert_eq!(
            config.mode().unwrap().to_string(),
            "router"
        );
    }

    #[test]
    fn test_session_builder_custom_endpoint() {
        let config = SessionConfigBuilder::new()
            .with_router_endpoint("tcp/192.168.1.1:7447")
            .build()
            .expect("Failed to build session config");
        assert_eq!(
            config.mode().unwrap().to_string(),
            "peer"
        );
    }

    #[test]
    fn test_generate_json5_router() {
        let json5 = generate_json5(&router_overrides(), "Test Router Config");
        assert!(json5.contains("GENERATED"));
        assert!(json5.contains("mode"));
        assert!(json5.contains("router"));
    }

    #[test]
    fn test_generate_json5_session() {
        let json5 = generate_json5(&session_overrides(), "Test Session Config");
        assert!(json5.contains("GENERATED"));
        assert!(json5.contains("mode"));
        assert!(json5.contains("peer"));
    }
}
