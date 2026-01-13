use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU16, Ordering};
use std::thread;
use std::time::Duration;

use nix::sys::signal::{self, Signal};
use nix::unistd::Pid;
use zenoh::config::WhatAmI;
use zenoh::Wait;

/// Helper to manage background processes with automatic cleanup
#[allow(dead_code)]
pub struct ProcessGuard {
    child: Option<Child>,
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
pub fn create_ros_z_context_with_router(router: &TestRouter) -> ros_z::Result<ros_z::context::ZContext> {
    create_ros_z_context_with_endpoint(router.endpoint())
}

/// Create a ros-z context configured to connect to a specific endpoint
pub fn create_ros_z_context_with_endpoint(endpoint: &str) -> ros_z::Result<ros_z::context::ZContext> {
    use ros_z::{Builder, context::ZContextBuilder};

    ZContextBuilder::default()
        .disable_multicast_scouting()
        .with_connect_endpoints([endpoint])
        .build()
}

/// Helper to wait for nodes to be ready
pub fn wait_for_ready(duration: Duration) {
    thread::sleep(duration);
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
