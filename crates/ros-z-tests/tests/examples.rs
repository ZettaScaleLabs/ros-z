//! Integration tests for ros-z examples.
//!
//! These tests verify that examples build and run correctly. All tests are
//! marked `#[ignore]` because they are slow — they invoke `cargo run` which
//! may trigger a build.
//!
//! Run with:
//!   cargo test -p ros-z-tests --test examples --features jazzy -- --ignored

use std::{
    process::{Child, Command, Stdio},
    time::Duration,
};

use nix::{
    sys::signal::{Signal, kill},
    unistd::Pid,
};

/// RAII guard that sends SIGKILL to a child process group on drop.
struct ProcessGroupGuard {
    child: Child,
}

impl ProcessGroupGuard {
    fn new(child: Child) -> Self {
        Self { child }
    }
}

impl Drop for ProcessGroupGuard {
    fn drop(&mut self) {
        // Try SIGTERM first
        let pid = self.child.id() as i32;
        let _ = kill(Pid::from_raw(-pid), Signal::SIGTERM);
        std::thread::sleep(Duration::from_millis(200));
        // Force kill process group
        let _ = kill(Pid::from_raw(-pid), Signal::SIGKILL);
        let _ = self.child.wait();
    }
}

// ---------------------------------------------------------------------------
// Self-contained examples
// ---------------------------------------------------------------------------

#[test]
#[ignore = "Slow: run with: cargo test --test examples --features jazzy -- --ignored"]
fn test_dynamic_message_basic() {
    let output = Command::new("cargo")
        .args([
            "run",
            "--package",
            "ros-z",
            "--example",
            "dynamic_message_basic",
            "--features",
            "jazzy",
        ])
        .output()
        .expect("Failed to run dynamic_message_basic example");

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "dynamic_message_basic failed with exit code: {:?}\nStderr: {}",
        output.status.code(),
        stderr
    );
    assert!(
        stdout.contains("=== Dynamic Message Basic Example ==="),
        "Missing example header. Stdout: {}",
        stdout
    );
    assert!(
        stdout.contains("=== Example Complete ==="),
        "Missing completion message. Stdout: {}",
        stdout
    );
}

#[test]
#[ignore = "Slow: run with: cargo test --test examples --features jazzy -- --ignored"]
fn test_dynamic_message_serialization() {
    let output = Command::new("cargo")
        .args([
            "run",
            "--package",
            "ros-z",
            "--example",
            "dynamic_message_serialization",
            "--features",
            "jazzy",
        ])
        .output()
        .expect("Failed to run dynamic_message_serialization example");

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "dynamic_message_serialization failed with exit code: {:?}\nStderr: {}",
        output.status.code(),
        stderr
    );
    assert!(
        stdout.contains("=== Dynamic Message Serialization Example ==="),
        "Missing example header. Stdout: {}",
        stdout
    );
    assert!(
        stdout.contains("=== Example Complete ==="),
        "Missing completion message. Stdout: {}",
        stdout
    );
}

#[test]
#[ignore = "Slow: run with: cargo test --test examples --features jazzy -- --ignored"]
fn test_dynamic_message_interop() {
    let output = Command::new("cargo")
        .args([
            "run",
            "--package",
            "ros-z",
            "--example",
            "dynamic_message_interop",
            "--features",
            "jazzy",
        ])
        .output()
        .expect("Failed to run dynamic_message_interop example");

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "dynamic_message_interop failed with exit code: {:?}\nStderr: {}",
        output.status.code(),
        stderr
    );
    assert!(
        stdout.contains("Bytes identical: YES"),
        "Static/dynamic CDR bytes should be identical. Stdout: {}",
        stdout
    );
    assert!(
        stdout.contains("Roundtrip successful: YES"),
        "Roundtrip serialization should succeed. Stdout: {}",
        stdout
    );
    assert!(
        stdout.contains("=== Example Complete ==="),
        "Missing completion message. Stdout: {}",
        stdout
    );
}

// ---------------------------------------------------------------------------
// Paired examples — background server + foreground client
// ---------------------------------------------------------------------------

#[test]
#[ignore = "Slow: run with: cargo test --test examples --features jazzy -- --ignored"]
fn test_z_srvcli() {
    use std::os::unix::process::CommandExt;

    // Build the example first to avoid timeout during test
    let build = Command::new("cargo")
        .args([
            "build",
            "--package",
            "ros-z",
            "--example",
            "z_srvcli",
            "--features",
            "jazzy",
        ])
        .status()
        .expect("Failed to build z_srvcli example");
    assert!(build.success(), "z_srvcli build failed");

    // Start server in its own process group so we can kill it cleanly
    let server = unsafe {
        Command::new("cargo")
            .args([
                "run",
                "--package",
                "ros-z",
                "--example",
                "z_srvcli",
                "--features",
                "jazzy",
                "--",
                "--mode",
                "server",
            ])
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .pre_exec(|| {
                // Create a new process group
                nix::unistd::setsid().ok();
                Ok(())
            })
            .spawn()
            .expect("Failed to spawn z_srvcli server")
    };
    let _server_guard = ProcessGroupGuard::new(server);

    // Give the server time to start
    std::thread::sleep(Duration::from_secs(2));

    // Run the client
    let client_output = Command::new("cargo")
        .args([
            "run",
            "--package",
            "ros-z",
            "--example",
            "z_srvcli",
            "--features",
            "jazzy",
            "--",
            "--mode",
            "client",
            "--a",
            "3",
            "--b",
            "4",
        ])
        .output()
        .expect("Failed to run z_srvcli client");

    let stdout = String::from_utf8_lossy(&client_output.stdout);
    let stderr = String::from_utf8_lossy(&client_output.stderr);

    assert!(
        client_output.status.success(),
        "z_srvcli client failed with exit code: {:?}\nStderr: {}",
        client_output.status.code(),
        stderr
    );
    assert!(
        stdout.contains("7"),
        "Expected response sum of 7 (3+4) in output. Stdout: {}",
        stdout
    );
}

#[test]
#[ignore = "Slow: run with: cargo test --test examples --features jazzy -- --ignored"]
fn test_z_custom_message() {
    use std::os::unix::process::CommandExt;

    // Build the example first
    let build = Command::new("cargo")
        .args([
            "build",
            "--package",
            "ros-z",
            "--example",
            "z_custom_message",
            "--features",
            "jazzy",
        ])
        .status()
        .expect("Failed to build z_custom_message example");
    assert!(build.success(), "z_custom_message build failed");

    // Start nav-server in its own process group
    let server = unsafe {
        Command::new("cargo")
            .args([
                "run",
                "--package",
                "ros-z",
                "--example",
                "z_custom_message",
                "--features",
                "jazzy",
                "--",
                "--mode",
                "nav-server",
            ])
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .pre_exec(|| {
                nix::unistd::setsid().ok();
                Ok(())
            })
            .spawn()
            .expect("Failed to spawn z_custom_message server")
    };
    let _server_guard = ProcessGroupGuard::new(server);

    // Give the server time to start
    std::thread::sleep(Duration::from_secs(2));

    // Run the client
    let client_output = Command::new("cargo")
        .args([
            "run",
            "--package",
            "ros-z",
            "--example",
            "z_custom_message",
            "--features",
            "jazzy",
            "--",
            "--mode",
            "nav-client",
            "--target-x",
            "3.0",
            "--target-y",
            "4.0",
            "--max-speed",
            "1.5",
        ])
        .output()
        .expect("Failed to run z_custom_message client");

    let stdout = String::from_utf8_lossy(&client_output.stdout);
    let stderr = String::from_utf8_lossy(&client_output.stderr);

    assert!(
        client_output.status.success(),
        "z_custom_message client failed with exit code: {:?}\nStderr: {}",
        client_output.status.code(),
        stderr
    );
    assert!(
        stdout.contains("Success: true"),
        "Expected successful navigation response. Stdout: {}",
        stdout
    );
}

#[test]
#[ignore = "Slow: run with: cargo test --test examples --features jazzy -- --ignored"]
fn test_z_pingpong() {
    use std::os::unix::process::CommandExt;

    // Build the example first
    let build = Command::new("cargo")
        .args([
            "build",
            "--package",
            "ros-z",
            "--example",
            "z_pingpong",
            "--features",
            "jazzy",
        ])
        .status()
        .expect("Failed to build z_pingpong example");
    assert!(build.success(), "z_pingpong build failed");

    // Start pong in its own process group
    let pong = unsafe {
        Command::new("cargo")
            .args([
                "run",
                "--package",
                "ros-z",
                "--example",
                "z_pingpong",
                "--features",
                "jazzy",
                "--",
                "--mode",
                "pong",
            ])
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .pre_exec(|| {
                nix::unistd::setsid().ok();
                Ok(())
            })
            .spawn()
            .expect("Failed to spawn z_pingpong pong")
    };
    let _pong_guard = ProcessGroupGuard::new(pong);

    // Give pong time to start
    std::thread::sleep(Duration::from_secs(2));

    // Run ping with a small sample count for speed
    let ping_output = Command::new("cargo")
        .args([
            "run",
            "--package",
            "ros-z",
            "--example",
            "z_pingpong",
            "--features",
            "jazzy",
            "--",
            "--mode",
            "ping",
            "--sample",
            "5",
            "--frequency",
            "2",
        ])
        .output()
        .expect("Failed to run z_pingpong ping");

    let stdout = String::from_utf8_lossy(&ping_output.stdout);
    let stderr = String::from_utf8_lossy(&ping_output.stderr);

    assert!(
        ping_output.status.success(),
        "z_pingpong ping failed with exit code: {:?}\nStderr: {}",
        ping_output.status.code(),
        stderr
    );
    assert!(
        stdout.contains("RTT stats"),
        "Expected RTT statistics in output. Stdout: {}",
        stdout
    );
    assert!(
        stdout.contains("Min"),
        "Expected Min RTT value in output. Stdout: {}",
        stdout
    );
}
