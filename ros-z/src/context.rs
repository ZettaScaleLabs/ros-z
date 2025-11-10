use std::sync::{Arc, atomic::AtomicUsize};

use zenoh::{Result, Session, Wait};

use crate::{Builder, graph::Graph, node::ZNodeBuilder};

#[derive(Debug, Default)]
pub struct GlobalCounter(AtomicUsize);

impl GlobalCounter {
    pub fn increment(&self) -> usize {
        self.0.fetch_add(1, std::sync::atomic::Ordering::AcqRel)
    }
}

use serde_json::json;
use std::path::PathBuf;

pub struct ZContextBuilder {
    domain_id: usize,
    config_file: Option<PathBuf>,
    config_overrides: Vec<(String, serde_json::Value)>,
}

impl Default for ZContextBuilder {
    fn default() -> Self {
        Self {
            domain_id: 0,
            config_file: None,
            config_overrides: Vec::new(),
        }
    }
}

impl ZContextBuilder {
    /// Set the ROS domain ID
    pub fn with_domain_id(mut self, domain_id: usize) -> Self {
        self.domain_id = domain_id;
        self
    }

    /// Load configuration from a JSON file
    pub fn with_config_file<P: Into<PathBuf>>(mut self, path: P) -> Self {
        self.config_file = Some(path.into());
        self
    }

    /// Add a JSON configuration override
    ///
    /// # Example
    /// ```
    /// use serde_json::json;
    ///
    /// let ctx = ZContextBuilder::default()
    ///     .with_json("scouting/multicast/enabled", json!(false))
    ///     .with_json("connect/endpoints", json!(["tcp/127.0.0.1:7447"]))
    ///     .build()?;
    /// ```
    pub fn with_json<K: Into<String>, V: serde::Serialize>(mut self, key: K, value: V) -> Self {
        let key = key.into();
        let value_json = serde_json::to_value(&value)
            .unwrap_or_else(|_| panic!("Failed to serialize value for key: {}", key));
        self.config_overrides.push((key, value_json));
        self
    }

    /// Convenience method: disable multicast scouting
    pub fn disable_multicast_scouting(self) -> Self {
        self.with_json("scouting/multicast/enabled", json!(false))
    }

    /// Convenience method: connect to specific endpoints
    ///
    /// # Example
    /// ```
    /// let ctx = ZContextBuilder::default()
    ///     .with_connect_endpoints(["tcp/127.0.0.1:7447"])
    ///     .build()?;
    /// ```
    pub fn with_connect_endpoints<I, S>(self, endpoints: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        let endpoints: Vec<String> = endpoints.into_iter().map(|s| s.into()).collect();
        self.with_json("connect/endpoints", json!(endpoints))
    }

    /// Convenience method: connect to localhost zenohd
    pub fn connect_to_local_zenohd(self) -> Self {
        self.with_connect_endpoints(["tcp/127.0.0.1:7447"])
    }

    /// Convenience method: set mode (peer, client, router)
    pub fn with_mode<S: Into<String>>(self, mode: S) -> Self {
        self.with_json("mode", json!(mode.into()))
    }

    /// Parse and apply overrides from environment variable
    ///
    /// Expected format: `key1=value1;key2=value2`
    /// Values should be valid JSON5
    ///
    /// # Example
    /// ```
    /// export ROSZ_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.1.1:7447"]'
    /// ```
    fn apply_env_overrides(mut self) -> Result<Self> {
        if let Ok(overrides_str) = std::env::var("ROSZ_CONFIG_OVERRIDE") {
            tracing::debug!(
                "Applying config overrides from ROSZ_CONFIG_OVERRIDE: {}",
                overrides_str
            );

            // Parse semicolon-separated key=value pairs
            for pair in overrides_str.split(';') {
                let pair = pair.trim();
                if pair.is_empty() {
                    continue;
                }

                // Split on first '=' only
                if let Some((key, value)) = pair.split_once('=') {
                    let key = key.trim();
                    let value = value.trim();

                    // Parse JSON5 value
                    match json5::from_str::<serde_json::Value>(value) {
                        Ok(json_value) => {
                            tracing::debug!("Override: {} = {}", key, json_value);
                            self.config_overrides.push((key.to_string(), json_value));
                        }
                        Err(e) => {
                            return Err(format!(
                                "Failed to parse ROSZ_CONFIG_OVERRIDE value for key '{}': {} (value: {})",
                                key, e, value
                            ).into());
                        }
                    }
                } else {
                    return Err(format!(
                        "Invalid ROSZ_CONFIG_OVERRIDE format: '{}'. Expected 'key=value'",
                        pair
                    )
                    .into());
                }
            }
        }

        Ok(self)
    }
}

impl Builder for ZContextBuilder {
    type Output = ZContext;

    fn build(mut self) -> Result<ZContext> {
        // Priority order:
        // 1. Config file passed via with_config_file()
        // 2. ROSZ_CONFIG_FILE environment variable
        // 3. Default config

        let mut config = if let Some(ref config_file) = self.config_file {
            // Use explicit config file
            zenoh::Config::from_file(config_file)?
        } else if let Ok(path) = std::env::var("ROSZ_CONFIG_FILE") {
            // Use environment variable config file
            zenoh::Config::from_file(path)?
        } else {
            // Use default config
            zenoh::Config::default()
        };

        // Apply environment variable overrides first
        self = self.apply_env_overrides()?;

        // Apply all JSON overrides
        for (key, value) in self.config_overrides {
            let value_str = serde_json::to_string(&value)
                .map_err(|e| format!("Failed to serialize value for key '{}': {}", key, e))?;

            config.insert_json5(&key, &value_str).map_err(|e| {
                format!(
                    "Failed to apply config override '{}' = '{}': {}",
                    key, value_str, e
                )
            })?;
        }

        // Open Zenoh session
        let session = zenoh::open(config).wait()?;

        let domain_id = self.domain_id;
        let graph = Arc::new(Graph::new(&session, domain_id)?);

        Ok(ZContext {
            session: Arc::new(session),
            counter: Arc::new(GlobalCounter::default()),
            domain_id,
            graph,
        })
    }
}

pub struct ZContext {
    session: Arc<Session>,
    // Global counter for the participants
    counter: Arc<GlobalCounter>,
    domain_id: usize,
    graph: Arc<Graph>,
}

impl ZContext {
    pub fn create_node<S: AsRef<str>>(&self, name: S) -> ZNodeBuilder {
        ZNodeBuilder {
            domain_id: self.domain_id,
            name: name.as_ref().to_owned(),
            namespace: "".to_string(),
            session: self.session.clone(),
            counter: self.counter.clone(),
            graph: self.graph.clone(),
        }
    }

    pub fn shutdown(&self) -> Result<()> {
        self.session.close().wait()
    }
}
