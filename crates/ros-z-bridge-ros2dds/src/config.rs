use clap::Parser;

/// Read ROS_DOMAIN_ID from the environment, falling back to 0.
fn default_domain_id() -> u32 {
    std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0)
}

fn default_domain_ids() -> Vec<u32> {
    vec![default_domain_id()]
}

/// CLI configuration for the ros2dds bridge.
#[derive(Parser, Debug, Clone)]
#[command(
    name = "ros-z-bridge-ros2dds",
    about = "Bridge between DDS-based ROS 2 nodes and a Zenoh/ros-z network"
)]
pub struct Config {
    /// Zenoh endpoint to connect to (e.g. tcp/127.0.0.1:7447).
    #[arg(short, long, default_value = "tcp/127.0.0.1:7447")]
    pub zenoh_endpoint: String,

    /// ROS 2 domain ID(s). May be specified multiple times for multi-domain bridging.
    /// Defaults to the ROS_DOMAIN_ID environment variable, or 0 if unset.
    #[arg(short, long, default_values_t = default_domain_ids())]
    pub domain_ids: Vec<u32>,

    /// Optional namespace prefix applied to all bridged topics/services on the Zenoh side.
    ///
    /// Example: `--namespace my_robot` → `/chatter` becomes `my_robot/chatter` in Zenoh.
    #[arg(short, long)]
    pub namespace: Option<String>,

    /// Global topic allow-list regex. Only topics matching this pattern are bridged.
    /// Applies to all entity types unless a per-type filter is set.
    #[arg(long)]
    pub allow: Option<String>,

    /// Global topic deny-list regex. Topics matching this pattern are not bridged.
    #[arg(long)]
    pub deny: Option<String>,

    /// Allow-list regex for DDS publisher (DDS→Zenoh) routes only.
    #[arg(long)]
    pub allow_pub: Option<String>,

    /// Deny-list regex for DDS publisher (DDS→Zenoh) routes only.
    #[arg(long)]
    pub deny_pub: Option<String>,

    /// Allow-list regex for DDS subscriber (Zenoh→DDS) routes only.
    #[arg(long)]
    pub allow_sub: Option<String>,

    /// Deny-list regex for DDS subscriber (Zenoh→DDS) routes only.
    #[arg(long)]
    pub deny_sub: Option<String>,

    /// Allow-list regex for DDS service server (DDS server → Zenoh queryable) routes only.
    #[arg(long)]
    pub allow_service_srv: Option<String>,

    /// Deny-list regex for DDS service server routes only.
    #[arg(long)]
    pub deny_service_srv: Option<String>,

    /// Allow-list regex for DDS service client (Zenoh querier → DDS client) routes only.
    #[arg(long)]
    pub allow_service_cli: Option<String>,

    /// Deny-list regex for DDS service client routes only.
    #[arg(long)]
    pub deny_service_cli: Option<String>,

    /// Allow-list regex for action routes (/_action/ topics). Falls back to --allow if unset.
    #[arg(long)]
    pub allow_action: Option<String>,

    /// Deny-list regex for action routes (/_action/ topics). Falls back to --deny if unset.
    #[arg(long)]
    pub deny_action: Option<String>,

    /// Block the Zenoh publisher on RELIABLE DDS topics instead of dropping samples.
    /// Enables CongestionControl::Block for DDS→Zenoh routes where DDS QoS is RELIABLE.
    #[arg(long, default_value_t = true)]
    pub reliable_routes_blocking: bool,

    /// Path to a Zenoh JSON5 config file. Merged before --zenoh-endpoint is applied.
    /// Use this to configure TLS, access control, or other advanced Zenoh options.
    #[arg(long)]
    pub zenoh_config_file: Option<std::path::PathBuf>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_domain_id_without_env() {
        unsafe { std::env::remove_var("ROS_DOMAIN_ID") };
        assert_eq!(default_domain_id(), 0);
    }

    #[test]
    fn test_default_domain_id_with_env() {
        unsafe { std::env::set_var("ROS_DOMAIN_ID", "42") };
        assert_eq!(default_domain_id(), 42);
        unsafe { std::env::remove_var("ROS_DOMAIN_ID") };
    }

    #[test]
    fn test_default_domain_id_invalid_env() {
        unsafe { std::env::set_var("ROS_DOMAIN_ID", "not_a_number") };
        assert_eq!(default_domain_id(), 0);
        unsafe { std::env::remove_var("ROS_DOMAIN_ID") };
    }

    #[test]
    fn test_config_file_field_defaults_to_none() {
        let cfg = Config::parse_from(["ros-z-bridge-ros2dds"]);
        assert!(cfg.zenoh_config_file.is_none());
    }

    #[test]
    fn test_config_file_field_stored() {
        let cfg = Config::parse_from(["ros-z-bridge-ros2dds", "--zenoh-config-file", "foo.json5"]);
        assert_eq!(
            cfg.zenoh_config_file,
            Some(std::path::PathBuf::from("foo.json5"))
        );
    }
}
