use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU16, Ordering};
use std::thread;
use std::time::Duration;

use nix::sys::signal::{self, Signal};
use nix::unistd::Pid;
use ros_z::Builder;
use zenoh::config::WhatAmI;
use zenoh::Wait;

/// Helper to manage background processes with automatic cleanup
#[allow(dead_code)]
pub struct ProcessGuard {
    pub child: Option<Child>,
    name: String,
}

#[allow(dead_code)]
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
            let pid = child.id() as i32;
            // Negative PID targets the process group
            let pgid = Pid::from_raw(-pid);

            println!("Stopping process group: {}", self.name);

            // 1. Send SIGINT to the whole process group
            // This ensures both the ros2 CLI wrapper and the actual node receive the signal
            if let Err(e) = signal::kill(pgid, Signal::SIGINT) {
                eprintln!("Failed to send SIGINT to group {}: {}", self.name, e);
                // Fallback: try killing just the parent handle we have
                let _ = child.kill();
            }

            // 2. Wait for graceful shutdown with a timeout
            let start = std::time::Instant::now();
            let timeout = Duration::from_secs(5);

            loop {
                match child.try_wait() {
                    Ok(Some(status)) => {
                        println!("Process {} exited gracefully with status: {:?}", self.name, status);
                        return;
                    }
                    Ok(None) => {
                        if start.elapsed() > timeout {
                            eprintln!(
                                "Timeout reached for {}, sending SIGKILL to group",
                                self.name
                            );
                            // 3. Force kill the group if it's still running
                            let _ = signal::kill(pgid, Signal::SIGKILL);
                            let _ = child.wait(); // Clean up zombie
                            break;
                        }
                        thread::sleep(Duration::from_millis(100));
                    }
                    Err(e) => {
                        eprintln!("Error waiting for process {}: {}", self.name, e);
                        let _ = signal::kill(pgid, Signal::SIGKILL);
                        let _ = child.wait();
                        break;
                    }
                }
            }
        }
    }
}

/// Port counter for generating unique Zenoh router ports per test
/// Use process ID to ensure unique ports across test binaries running in parallel
static NEXT_PORT: once_cell::sync::Lazy<AtomicU16> = once_cell::sync::Lazy::new(|| {
    let pid = std::process::id();
    // Start from a port derived from PID to avoid collisions
    // Use higher ports (30000-60000) to avoid common service ports
    let base_port = 30000 + ((pid % 10000) as u16);
    println!("Test process {} using base port {}", pid, base_port);

    AtomicU16::new(base_port)
});

/// Per-test Zenoh router configuration
pub struct TestRouter {
    #[allow(dead_code)]
    pub port: u16,
    pub endpoint: String,
    _session: zenoh::Session,
}

impl TestRouter {
    /// Start a new Zenoh router session on a unique port for this test
    pub fn new() -> Self {
        let port = NEXT_PORT.fetch_add(1, Ordering::SeqCst);
        let endpoint = format!("tcp/127.0.0.1:{}", port);

        println!("Starting Zenoh router on port {}...", port);

        // Create Zenoh router session programmatically
        let mut config = zenoh::Config::default();
        config.set_mode(Some(WhatAmI::Router)).unwrap();
        config.insert_json5("listen/endpoints", &format!("[\"{}\"]", endpoint)).unwrap();
        config.insert_json5("scouting/multicast/enabled", "false").unwrap();

        let session = zenoh::open(config)
            .wait()
            .expect("Failed to open Zenoh router session");

        // Wait for router to be ready
        thread::sleep(Duration::from_millis(500));
        println!("Zenoh router ready on {}", endpoint);

        Self {
            port,
            endpoint: endpoint.clone(),
            _session: session,
        }
    }

    /// Get the endpoint string for this router
    pub fn endpoint(&self) -> &str {
        &self.endpoint
    }

    /// Get environment variable override for RMW Zenoh
    #[allow(dead_code)]
    pub fn rmw_zenoh_env(&self) -> String {
        format!("connect/endpoints=[\"tcp/127.0.0.1:{}\"]", self.port)
    }
}

/// Create a ros-z context configured to connect to a specific Zenoh router
#[allow(dead_code)]
pub fn create_ros_z_context_with_router(router: &TestRouter) -> ros_z::Result<ros_z::context::ZContext> {
    create_ros_z_context_with_endpoint(router.endpoint())
}

/// Create a ros-z context configured to connect to a specific endpoint
pub fn create_ros_z_context_with_endpoint(endpoint: &str) -> ros_z::Result<ros_z::context::ZContext> {
    use ros_z::{Builder, context::ZContextBuilder};

    ZContextBuilder::default()
        .disable_multicast_scouting()
        .with_connect_endpoints([endpoint])
        .with_logging_enabled()
        .build()
}

/// Helper to wait for nodes to be ready
#[allow(dead_code)]
pub fn wait_for_ready(duration: Duration) {
    thread::sleep(duration);
}

/// Deterministically wait for a service to be ready by polling with test requests
#[allow(dead_code)]
pub fn wait_for_service_ready(
    ctx: &ros_z::context::ZContext,
    service_name: &str,
    timeout: Duration,
) -> Result<(), Box<dyn std::error::Error>> {
    let start_time = std::time::Instant::now();

    loop {
        // Try to create a client and send a test request
        if let Ok(node) = ctx.create_node("service_readiness_checker").build()
            && let Ok(client) = node.create_client::<protobuf_demo::Calculate>(service_name).build() {
                // Try a simple test request (add 1 + 1 = 2)
                let test_request = protobuf_demo::CalculateRequest {
                    a: 1.0,
                    b: 1.0,
                    operation: "add".to_string(),
                };

                let rt = tokio::runtime::Runtime::new()?;
                let result = rt.block_on(async {
                    client.send_request(&test_request).await?;
                    client.take_response_timeout(Duration::from_millis(500))
                });

            if result.is_ok() {
                println!("Service '{}' is ready", service_name);
                return Ok(());
            }
        }

        // Check timeout
        if start_time.elapsed() >= timeout {
            return Err(format!("Service '{}' did not become ready within {:?}", service_name, timeout).into());
        }

        // Wait a bit before retrying
        thread::sleep(Duration::from_millis(100));
    }
}

/// Check if ros2 CLI is available
#[allow(dead_code)]
pub fn check_ros2_available() -> bool {
    Command::new("ros2").arg("--version").output().is_ok()
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

/// Check if action_tutorials_cpp package is available
#[allow(dead_code)]
pub fn check_action_tutorials_cpp_available() -> bool {
    Command::new("ros2")
        .args(["pkg", "prefix", "action_tutorials_cpp"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|status| status.success())
        .unwrap_or(false)
}

// ============================================================================
// Python Interop Helpers
// ============================================================================

#[cfg(feature = "python-interop")]
use std::path::PathBuf;

/// Get the path to the Python executable in ros-z-py venv
#[cfg(feature = "python-interop")]
fn python_executable() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("ros-z-py/.venv/bin/python")
}

/// Get the path to a Python example script
#[cfg(feature = "python-interop")]
fn example_script(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("ros-z-py/examples")
        .join(name)
}

/// Check if Python venv is available for interop tests
#[cfg(feature = "python-interop")]
#[allow(dead_code)]
pub fn check_python_venv_available() -> bool {
    python_executable().exists()
}

/// Spawn Python topic_demo.py as talker (publisher)
#[cfg(feature = "python-interop")]
#[allow(dead_code)]
pub fn spawn_python_talker(endpoint: &str, topic: &str, count: u32) -> ProcessGuard {
    use std::os::unix::process::CommandExt;

    let child = Command::new(python_executable())
        .arg(example_script("topic_demo.py"))
        .args(["-r", "talker"])
        .args(["-e", endpoint])
        .args(["-t", topic])
        .args(["-c", &count.to_string()])
        .args(["--interval", "0.3"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .process_group(0)
        .spawn()
        .expect("Failed to spawn Python talker");

    ProcessGuard::new(child, "python_talker")
}

/// Spawn Python topic_demo.py as listener (subscriber)
#[cfg(feature = "python-interop")]
#[allow(dead_code)]
pub fn spawn_python_listener(endpoint: &str, topic: &str, timeout_sec: f32) -> ProcessGuard {
    use std::os::unix::process::CommandExt;

    let child = Command::new(python_executable())
        .arg(example_script("topic_demo.py"))
        .args(["-r", "listener"])
        .args(["-e", endpoint])
        .args(["-t", topic])
        .args(["--timeout", &timeout_sec.to_string()])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .process_group(0)
        .spawn()
        .expect("Failed to spawn Python listener");

    ProcessGuard::new(child, "python_listener")
}

/// Spawn Python service_demo.py as server
#[cfg(feature = "python-interop")]
#[allow(dead_code)]
pub fn spawn_python_service_server(endpoint: &str, service_name: &str, max_requests: u32) -> ProcessGuard {
    use std::os::unix::process::CommandExt;

    let child = Command::new(python_executable())
        .arg(example_script("service_demo.py"))
        .args(["-r", "server"])
        .args(["-e", endpoint])
        .args(["-s", service_name])
        .args(["-c", &max_requests.to_string()])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .process_group(0)
        .spawn()
        .expect("Failed to spawn Python service server");

    ProcessGuard::new(child, "python_service_server")
}

/// Spawn Python service_demo.py as client
#[cfg(feature = "python-interop")]
#[allow(dead_code)]
pub fn spawn_python_service_client(endpoint: &str, service_name: &str, a: i64, b: i64) -> ProcessGuard {
    use std::os::unix::process::CommandExt;

    let child = Command::new(python_executable())
        .arg(example_script("service_demo.py"))
        .args(["-r", "client"])
        .args(["-e", endpoint])
        .args(["-s", service_name])
        .args(["-a", &a.to_string()])
        .args(["-b", &b.to_string()])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .process_group(0)
        .spawn()
        .expect("Failed to spawn Python service client");

    ProcessGuard::new(child, "python_service_client")
}

// ============================================================================
// ros2dds Backend Interop Helpers (with zenoh-bridge-ros2dds)
// ============================================================================

/// Check if zenoh-bridge-ros2dds is available
#[cfg(feature = "ros2dds-interop")]
#[allow(dead_code)]
pub fn check_zenoh_bridge_ros2dds_available() -> bool {
    Command::new("zenoh-bridge-ros2dds")
        .arg("--version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|status| status.success())
        .unwrap_or(false)
}

/// Spawn zenoh-bridge-ros2dds process
///
/// The bridge will:
/// - Act as a Zenoh router (default behavior)
/// - Bridge DDS traffic from ROS 2 nodes using CycloneDDS/FastDDS
/// - Listen on the default Zenoh port (7447) for peer connections
#[cfg(feature = "ros2dds-interop")]
#[allow(dead_code)]
pub fn spawn_zenoh_bridge_ros2dds() -> ProcessGuard {
    use std::os::unix::process::CommandExt;

    let child = Command::new("zenoh-bridge-ros2dds")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .process_group(0)
        .spawn()
        .expect("Failed to spawn zenoh-bridge-ros2dds");

    // Give the bridge time to start up
    thread::sleep(Duration::from_secs(2));

    ProcessGuard::new(child, "zenoh-bridge-ros2dds")
}

/// Spawn zenoh-bridge-ros2dds with a specific listen endpoint
#[cfg(feature = "ros2dds-interop")]
#[allow(dead_code)]
pub fn spawn_zenoh_bridge_ros2dds_with_endpoint(listen_endpoint: &str) -> ProcessGuard {
    use std::os::unix::process::CommandExt;

    let child = Command::new("zenoh-bridge-ros2dds")
        .args(["-l", listen_endpoint])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .process_group(0)
        .spawn()
        .expect("Failed to spawn zenoh-bridge-ros2dds");

    // Give the bridge time to start up
    thread::sleep(Duration::from_secs(2));

    ProcessGuard::new(child, "zenoh-bridge-ros2dds")
}

/// Spawn a ROS 2 demo_nodes_cpp talker using CycloneDDS (default DDS)
#[cfg(feature = "ros2dds-interop")]
#[allow(dead_code)]
pub fn spawn_ros2_cyclone_talker() -> ProcessGuard {
    use std::os::unix::process::CommandExt;

    let child = Command::new("ros2")
        .args(["run", "demo_nodes_cpp", "talker"])
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .process_group(0)
        .spawn()
        .expect("Failed to spawn ROS 2 CycloneDDS talker");

    // Wait for the node to be ready
    thread::sleep(Duration::from_secs(2));

    ProcessGuard::new(child, "ros2_cyclone_talker")
}

/// Spawn a ROS 2 demo_nodes_cpp listener using CycloneDDS (default DDS)
#[cfg(feature = "ros2dds-interop")]
#[allow(dead_code)]
pub fn spawn_ros2_cyclone_listener() -> ProcessGuard {
    use std::os::unix::process::CommandExt;

    let child = Command::new("ros2")
        .args(["run", "demo_nodes_cpp", "listener"])
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .process_group(0)
        .spawn()
        .expect("Failed to spawn ROS 2 CycloneDDS listener");

    // Wait for the node to be ready
    thread::sleep(Duration::from_secs(2));

    ProcessGuard::new(child, "ros2_cyclone_listener")
}

/// Create a ros-z context for ros2dds backend
///
/// This configures ros-z to use peer mode, which will discover
/// zenoh-bridge-ros2dds via multicast scouting.
#[cfg(feature = "ros2dds-interop")]
#[allow(dead_code)]
pub fn create_ros_z_context_ros2dds() -> ros_z::Result<ros_z::context::ZContext> {
    use ros_z::context::ZContextBuilder;

    ZContextBuilder::default()
        .with_mode("peer")
        .with_logging_enabled()
        .build()
}

/// Create a ros-z context for ros2dds backend with a specific endpoint
#[cfg(feature = "ros2dds-interop")]
#[allow(dead_code)]
pub fn create_ros_z_context_ros2dds_with_endpoint(endpoint: &str) -> ros_z::Result<ros_z::context::ZContext> {
    use ros_z::context::ZContextBuilder;

    ZContextBuilder::default()
        .with_mode("client")
        .with_connect_endpoints([endpoint])
        .disable_multicast_scouting()
        .with_logging_enabled()
        .build()
}

/// Spawn a ROS 2 add_two_ints_server using CycloneDDS
#[cfg(feature = "ros2dds-interop")]
#[allow(dead_code)]
pub fn spawn_ros2_cyclone_add_two_ints_server() -> ProcessGuard {
    use std::os::unix::process::CommandExt;

    let child = Command::new("ros2")
        .args(["run", "demo_nodes_cpp", "add_two_ints_server"])
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .process_group(0)
        .spawn()
        .expect("Failed to spawn ROS 2 CycloneDDS add_two_ints_server");

    // Wait for the service server to be ready
    thread::sleep(Duration::from_secs(2));

    ProcessGuard::new(child, "ros2_cyclone_add_two_ints_server")
}

/// Spawn a ROS 2 add_two_ints_client using CycloneDDS
#[cfg(feature = "ros2dds-interop")]
#[allow(dead_code)]
pub fn spawn_ros2_cyclone_add_two_ints_client(a: i64, b: i64) -> ProcessGuard {
    use std::os::unix::process::CommandExt;

    let child = Command::new("ros2")
        .args(["run", "demo_nodes_cpp", "add_two_ints_client"])
        .args([&a.to_string(), &b.to_string()])
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .process_group(0)
        .spawn()
        .expect("Failed to spawn ROS 2 CycloneDDS add_two_ints_client");

    ProcessGuard::new(child, "ros2_cyclone_add_two_ints_client")
}
