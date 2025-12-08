use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU16, Ordering};
use std::thread;
use std::time::Duration;

use zenoh::config::WhatAmI;
use zenoh::Wait;

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
            // NOTE(Carter12s): cannot directly kill a `ros2` command line command
            // As that will kill the commandline process without giving it time to shutdown whatever
            // child processes it spawns.
            let pid = child.id();

            // Send Ctrl-C to trigger graceful shutdown
            if let Err(e) = nix::sys::signal::kill(
                nix::unistd::Pid::from_raw(pid as _),
                nix::sys::signal::SIGINT,
            ) {
                eprintln!("Failed to send SIGINT to process {}: {}", self.name, e);
                let _ = child.kill();
                let _ = child.wait();
                return;
            }

            // Wait for graceful shutdown with a timeout
            let start = std::time::Instant::now();
            let timeout = Duration::from_secs(5);

            loop {
                match child.try_wait() {
                    Ok(Some(status)) => {
                        println!("Process {} exited with status: {:?}", self.name, status);
                        break;
                    }
                    Ok(None) => {
                        if start.elapsed() > timeout {
                            eprintln!(
                                "Process {} did not exit gracefully, sending SIGKILL",
                                self.name
                            );
                            let _ = child.kill();
                            let _ = child.wait();
                            break;
                        }
                        thread::sleep(Duration::from_millis(100));
                    }
                    Err(e) => {
                        eprintln!("Error waiting for process {}: {}", self.name, e);
                        let _ = child.kill();
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
    pub port: u16,
    pub endpoint: String,
    _session: zenoh::Session,
}

impl TestRouter {
    /// Start a new Zenoh router session on a unique port for this test
    pub fn new() -> Self {
        let port = NEXT_PORT.fetch_add(1, Ordering::SeqCst);
        let endpoint = format!("tcp/127.0.0.1:{}", port);

        println!("🚀 Starting Zenoh router on port {}...", port);

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
        println!("✅ Zenoh router ready on {}", endpoint);

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
