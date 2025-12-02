#![cfg(feature = "interop-tests")]

mod common;

// Import the demo_nodes module from the examples directory.
// This uses #[path] to reference code outside the normal module tree,
// allowing tests to reuse the exact same code that users run as examples.
// This is preferable to code duplication and ensures quality.
#[path = "../../ros-z/examples/demo_nodes/mod.rs"]
mod demo_nodes;

use std::{
    process::{Command, Stdio},
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use crate::common::*;

#[serial_test::serial]
#[test]
fn test_ros_z_talker_to_ros_z_listener() {
    ensure_zenohd_running();

    println!("\n=== Test: ros-z talker -> ros-z listener ===");

    let received = Arc::new(Mutex::new(Vec::new()));
    let received_clone = received.clone();

    // Start ros-z listener in a thread using the example code
    let listener_handle = thread::spawn(move || {
        tokio::runtime::Runtime::new().unwrap().block_on(async {
            let ctx = create_ros_z_context().expect("Failed to create ros-z context");

            // Use the actual listener example code with timeout
            let messages =
                demo_nodes::run_listener(ctx, "chatter", Some(3), Some(Duration::from_secs(15)))
                    .await
                    .expect("Listener failed");

            let mut received = received_clone.lock().unwrap();
            *received = messages;
        });
    });

    wait_for_ready(Duration::from_secs(2));

    // Start ros-z talker in a thread using the example code
    let talker_handle = thread::spawn(|| {
        tokio::runtime::Runtime::new().unwrap().block_on(async {
            let ctx = create_ros_z_context().expect("Failed to create ros-z context");

            // Use the actual talker example code with max 5 messages
            demo_nodes::run_talker(ctx, "chatter", Duration::from_secs(1), Some(5))
                .await
                .expect("Talker failed");
        });
    });

    talker_handle.join().expect("Talker thread panicked");
    listener_handle.join().expect("Listener thread panicked");

    let msgs = received.lock().unwrap();
    assert!(
        msgs.len() >= 3,
        "❌ Test failed: Expected at least 3 messages, got {}",
        msgs.len()
    );

    println!(
        "✅ Test passed: ros-z listener received {} messages from ros-z talker",
        msgs.len()
    );
}

#[serial_test::serial]
#[test]
fn test_rcl_talker_to_ros_z_listener() {
    if !check_ros2_available() {
        panic!("ros2 CLI not available - ensure ROS 2 is installed");
    }

    if !check_demo_nodes_cpp_available() {
        panic!("demo_nodes_cpp package not found - ensure it is installed");
    }

    ensure_zenohd_running();

    println!("\n=== Test: RCL demo_nodes_cpp talker -> ros-z listener ===");

    let received = Arc::new(Mutex::new(Vec::new()));
    let received_clone = received.clone();

    // Start ros-z listener in a thread using the example code
    let listener_handle = thread::spawn(move || {
        tokio::runtime::Runtime::new().unwrap().block_on(async {
            let ctx = create_ros_z_context().expect("Failed to create ros-z context");

            // Use the actual listener example code with timeout
            let messages =
                demo_nodes::run_listener(ctx, "chatter", Some(3), Some(Duration::from_secs(15)))
                    .await
                    .expect("Listener failed");

            let mut received = received_clone.lock().unwrap();
            *received = messages;
        });
    });

    wait_for_ready(Duration::from_secs(2));

    // Start RCL talker
    let talker = Command::new("ros2")
        .args(["run", "demo_nodes_cpp", "talker"])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("Failed to start RCL talker");

    let _talker_guard = ProcessGuard::new(talker, "RCL talker");

    listener_handle.join().expect("Listener thread panicked");

    let msgs = received.lock().unwrap();
    assert!(
        msgs.len() >= 3,
        "❌ Test failed: Expected at least 3 messages, got {}",
        msgs.len()
    );

    println!(
        "✅ Test passed: ros-z listener received {} messages from RCL talker",
        msgs.len()
    );
}

#[serial_test::serial]
#[test]
fn test_ros_z_talker_to_rcl_listener() {
    if !check_ros2_available() {
        panic!("ros2 CLI not available - ensure ROS 2 is installed");
    }

    if !check_demo_nodes_cpp_available() {
        panic!("demo_nodes_cpp package not found - ensure it is installed");
    }

    ensure_zenohd_running();

    println!("\n=== Test: ros-z talker -> RCL demo_nodes_cpp listener ===");

    // Start RCL listener
    let listener = Command::new("ros2")
        .args(["run", "demo_nodes_cpp", "listener"])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("Failed to start RCL listener");

    let _listener_guard = ProcessGuard::new(listener, "RCL listener");

    wait_for_ready(Duration::from_secs(2));

    // Start ros-z talker in a thread using the example code
    let talker_handle = thread::spawn(|| {
        tokio::runtime::Runtime::new().unwrap().block_on(async {
            let ctx = create_ros_z_context().expect("Failed to create ros-z context");

            // Use the actual talker example code with max 10 messages
            demo_nodes::run_talker(ctx, "chatter", Duration::from_secs(1), Some(10))
                .await
                .expect("Talker failed");
        });
    });

    talker_handle.join().expect("Talker thread panicked");

    // Give some time for RCL listener to process
    wait_for_ready(Duration::from_secs(2));

    println!("✅ Test passed: ros-z talker published messages to RCL listener");
}
