use std::process::{Child, Command, Stdio};
use std::thread;
use std::time::Duration;

/// Helper to manage background processes with automatic cleanup
pub struct ProcessGuard {
    child: Option<Child>,
    name: String,
}

impl ProcessGuard {
    pub fn new(child: Child, name: &str) -> Self {
        println!("Started process: {}", name);
        Self {
            child: Some(child),
            name: name.to_string(),
        }
    }
}

impl Drop for ProcessGuard {
    fn drop(&mut self) {
        if let Some(mut child) = self.child.take() {
            println!("Stopping process: {}", self.name);
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

/// Global zenohd daemon shared across ALL tests
pub static ZENOHD: once_cell::sync::Lazy<ProcessGuard> = once_cell::sync::Lazy::new(|| {
    println!("🚀 Starting shared rmw_zenohd daemon...");

    // Check if rmw_zenoh_cpp is available
    if !check_rmw_zenoh_available() {
        panic!(
            "rmw_zenoh_cpp package not found!\n\
             Please install it with: apt install ros-$ROS_DISTRO-rmw-zenoh-cpp\n\
             Or ensure ROS environment is sourced: source /opt/ros/$ROS_DISTRO/setup.bash"
        );
    }

    let child = Command::new("ros2")
        .args(["run", "rmw_zenoh_cpp", "rmw_zenohd"])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .expect("Failed to start rmw_zenohd - ensure rmw_zenoh_cpp is installed");

    // Wait for zenohd to be ready
    thread::sleep(Duration::from_secs(2));
    println!("✅ rmw_zenohd daemon ready on tcp/127.0.0.1:7447");

    ProcessGuard::new(child, "rmw_zenohd")
});

/// Ensure zenohd is running (call this at the start of each test)
pub fn ensure_zenohd_running() {
    // Force lazy initialization
    let _ = &*ZENOHD;
}

/// Create a ros-z context configured to connect to local zenohd
pub fn create_ros_z_context() -> ros_z::Result<ros_z::context::ZContext> {
    use ros_z::{Builder, context::ZContextBuilder};

    ZContextBuilder::default()
        .disable_multicast_scouting()
        .connect_to_local_zenohd()
        .build()
}

/// Helper to wait for nodes to be ready
pub fn wait_for_ready(duration: Duration) {
    thread::sleep(duration);
}

/// Check if ros2 CLI is available
pub fn check_ros2_available() -> bool {
    Command::new("ros2").arg("--version").output().is_ok()
}

/// Check if rmw_zenoh_cpp package is available
pub fn check_rmw_zenoh_available() -> bool {
    Command::new("ros2")
        .args(["pkg", "prefix", "rmw_zenoh_cpp"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|status| status.success())
        .unwrap_or(false)
}

/// Check if demo_nodes_cpp package is available
#[allow(dead_code)]
pub fn check_demo_nodes_cpp_available() -> bool {
    Command::new("ros2")
        .args(["pkg", "prefix", "demo_nodes_cpp"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|status| status.success())
        .unwrap_or(false)
}
