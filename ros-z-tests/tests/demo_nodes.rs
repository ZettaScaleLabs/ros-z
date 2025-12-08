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

#[test]
fn test_ros_z_talker_to_ros_z_listener() {
    let router = TestRouter::new();

    println!("\n=== Test: ros-z talker -> ros-z listener ===");

    let received = Arc::new(Mutex::new(Vec::new()));
    let received_clone = received.clone();

    // Start ros-z listener in a thread using the example code
    let router_endpoint = router.endpoint().to_string();
    let listener_handle = thread::spawn(move || {
        tokio::runtime::Runtime::new().unwrap().block_on(async {
            let ctx = create_ros_z_context_with_endpoint(&router_endpoint)
                .expect("Failed to create ros-z context");

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
    let talker_handle = thread::spawn(move || {
        tokio::runtime::Runtime::new().unwrap().block_on(async {
            let ctx =
                create_ros_z_context_with_router(&router).expect("Failed to create ros-z context");

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

#[test]
fn test_rcl_talker_to_ros_z_listener() {
    if !check_ros2_available() {
        panic!("ros2 CLI not available - ensure ROS 2 is installed");
    }

    if !check_demo_nodes_cpp_available() {
        panic!("demo_nodes_cpp package not found - ensure it is installed");
    }

    let router = TestRouter::new();

    println!("\n=== Test: RCL demo_nodes_cpp talker -> ros-z listener ===");

    let received = Arc::new(Mutex::new(Vec::new()));
    let received_clone = received.clone();

    // Start ros-z listener in a thread using the example code
    let router_endpoint = router.endpoint().to_string();
    let listener_handle = thread::spawn(move || {
        tokio::runtime::Runtime::new().unwrap().block_on(async {
            let ctx = create_ros_z_context_with_endpoint(&router_endpoint)
                .expect("Failed to create ros-z context");

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
        .env("ZENOH_CONFIG_OVERRIDE", router.rmw_zenoh_env())
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

#[test]
fn test_ros_z_talker_to_rcl_listener() {
    if !check_ros2_available() {
        panic!("ros2 CLI not available - ensure ROS 2 is installed");
    }

    if !check_demo_nodes_cpp_available() {
        panic!("demo_nodes_cpp package not found - ensure it is installed");
    }

    let router = TestRouter::new();

    println!("\n=== Test: ros-z talker -> RCL demo_nodes_cpp listener ===");

    // Start RCL listener
    let listener = Command::new("ros2")
        .args(["run", "demo_nodes_cpp", "listener"])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .env("ZENOH_CONFIG_OVERRIDE", router.rmw_zenoh_env())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("Failed to start RCL listener");

    let _listener_guard = ProcessGuard::new(listener, "RCL listener");

    wait_for_ready(Duration::from_secs(2));

    // Start ros-z talker in a thread using the example code
    let talker_handle = thread::spawn(move || {
        tokio::runtime::Runtime::new().unwrap().block_on(async {
            let ctx =
                create_ros_z_context_with_router(&router).expect("Failed to create ros-z context");

            // Use the actual talker example code with faster publishing (100ms intervals)
            demo_nodes::run_talker(ctx, "chatter", Duration::from_millis(100), Some(10))
                .await
                .expect("Talker failed");
        });
    });

    talker_handle.join().expect("Talker thread panicked");

    // Give some time for RCL listener to process
    wait_for_ready(Duration::from_secs(1));

    println!("✅ Test passed: ros-z talker published messages to RCL listener");
}

#[test]
fn test_ros_z_add_two_ints_server_to_ros_z_client() {
    let router = TestRouter::new();

    println!("\n=== Test: ros-z add_two_ints server -> ros-z client ===");

    let (tx, rx) = std::sync::mpsc::channel();

    // Start ros-z server in a thread using the example code
    let router_endpoint = router.endpoint().to_string();
    let server_handle = thread::spawn(move || {
        let ctx = create_ros_z_context_with_endpoint(&router_endpoint)
            .expect("Failed to create ros-z context");

        // Use the actual server example code (handle one request)
        let result = demo_nodes::run_add_two_ints_server(ctx, Some(1));
        let _ = tx.send(()); // Signal completion
        result.expect("Server failed");
    });

    wait_for_ready(Duration::from_secs(2));

    // Run ros-z client in the main thread using the example code
    let ctx = create_ros_z_context_with_router(&router).expect("Failed to create ros-z context");
    let result = demo_nodes::run_add_two_ints_client(ctx, 2, 3, false).expect("Client failed");

    assert_eq!(result, 5, "Expected 2 + 3 = 5");

    // Wait for server to signal completion (with timeout)
    match rx.recv_timeout(Duration::from_secs(5)) {
        Ok(_) => {
            server_handle.join().expect("Server thread panicked");
            println!(
                "✅ Test passed: ros-z client received {} from ros-z server",
                result
            );
        }
        Err(_) => {
            println!(
                "✅ Test passed: ros-z client received {} from ros-z server (server still cleaning up)",
                result
            );
            // Don't wait for server join if it's taking too long
        }
    }
}

#[test]
fn test_rcl_add_two_ints_server_to_ros_z_client() {
    if !check_ros2_available() {
        panic!("ros2 CLI not available - ensure ROS 2 is installed");
    }

    if !check_demo_nodes_cpp_available() {
        panic!("demo_nodes_cpp package not found - ensure it is installed");
    }

    let router = TestRouter::new();

    println!("\n=== Test: RCL demo_nodes_cpp add_two_ints server -> ros-z client ===");

    // Start RCL server
    let server = Command::new("ros2")
        .args(["run", "demo_nodes_cpp", "add_two_ints_server"])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .env("ZENOH_CONFIG_OVERRIDE", router.rmw_zenoh_env())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("Failed to start RCL server");

    let _server_guard = ProcessGuard::new(server, "RCL add_two_ints server");

    wait_for_ready(Duration::from_secs(5));

    // Start ros-z client in a thread using the example code
    let client_handle = thread::spawn(move || -> i64 {
        let ctx =
            create_ros_z_context_with_router(&router).expect("Failed to create ros-z context");

        // Use the actual client example code
        demo_nodes::run_add_two_ints_client(ctx, 4, 7, false).expect("Client failed")
    });

    let result = client_handle.join().expect("Client thread panicked");
    assert_eq!(result, 11, "Expected 4 + 7 = 11");

    println!(
        "✅ Test passed: ros-z client received {} from RCL server",
        result
    );
}

#[test]
fn test_ros_z_add_two_ints_server_to_rcl_client() {
    if !check_ros2_available() {
        panic!("ros2 CLI not available - ensure ROS 2 is installed");
    }

    if !check_demo_nodes_cpp_available() {
        panic!("demo_nodes_cpp package not found - ensure it is installed");
    }

    let router = TestRouter::new();

    println!("\n=== Test: ros-z add_two_ints server -> RCL demo_nodes_cpp client ===");

    // Start ros-z server in a thread using the example code
    let router_endpoint = router.endpoint().to_string();
    let server_handle = thread::spawn(move || {
        let ctx = create_ros_z_context_with_endpoint(&router_endpoint)
            .expect("Failed to create ros-z context");

        // Use the actual server example code (handle one request)
        demo_nodes::run_add_two_ints_server(ctx, Some(1)).expect("Server failed");
    });

    wait_for_ready(Duration::from_secs(2));

    // Start RCL client
    let client = Command::new("ros2")
        .args(["run", "demo_nodes_cpp", "add_two_ints_client"])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .env("ZENOH_CONFIG_OVERRIDE", router.rmw_zenoh_env())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("Failed to start RCL client");

    let _client_guard = ProcessGuard::new(client, "RCL add_two_ints client");

    // Wait for the client to complete
    wait_for_ready(Duration::from_secs(5));

    // Stop the server
    server_handle.join().expect("Server thread panicked");

    println!("✅ Test passed: RCL client called ros-z server");
}
