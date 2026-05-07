use clap::Parser;

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

    /// ROS 2 domain ID for the DDS participant.
    #[arg(short, long, default_value_t = 0)]
    pub domain_id: u32,

    /// Optional namespace prefix applied to all bridged topics/services on the Zenoh side.
    ///
    /// Example: `--namespace my_robot` → `/chatter` becomes `my_robot/chatter` in Zenoh.
    #[arg(short, long)]
    pub namespace: Option<String>,

    /// Topic allow-list regex. Only topics matching this pattern are bridged.
    /// Defaults to bridging all topics.
    #[arg(long)]
    pub allow: Option<String>,

    /// Topic deny-list regex. Topics matching this pattern are not bridged.
    #[arg(long)]
    pub deny: Option<String>,
}
