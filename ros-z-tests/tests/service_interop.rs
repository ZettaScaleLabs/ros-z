#![cfg(feature = "interop-tests")]

mod common;

use std::process::{Command, Stdio};
use std::time::Duration;
use std::thread;

use ros_z::Builder;
use ros_z_msgs::example_interfaces::srv::{AddTwoInts, AddTwoInts_Request, AddTwoInts_Response};

use common::*;

#[test]
fn test_ros_z_server_ros_z_client() {
    ensure_zenohd_running();

    println!("\n=== Test: ros-z server <-> ros-z client ===");

    // Start server in background thread
    let server_handle = thread::spawn(|| {
        let ctx = create_ros_z_context()
            .expect("Failed to create context");
        
        let node = ctx.create_node("test_server")
            .build()
            .expect("Failed to create node");
        
        let _service = node.create_service::<AddTwoInts>(
            "/add_two_ints",
            |req: AddTwoInts_Request| async move {
                AddTwoInts_Response { sum: req.a + req.b }
            }
        ).expect("Failed to create service");
        
        // Keep server running
        thread::sleep(Duration::from_secs(5));
    });

    wait_for_ready(Duration::from_secs(2));

    // Run client
    let rt = tokio::runtime::Runtime::new().unwrap();
    let result = rt.block_on(async {
        let ctx = create_ros_z_context()
            .expect("Failed to create context");
        
        let node = ctx.create_node("test_client")
            .build()
            .expect("Failed to create node");
        
        let client = node.create_client::<AddTwoInts>("/add_two_ints")
            .expect("Failed to create client");

        let request = AddTwoInts_Request { a: 5, b: 3 };
        client.call(request).await
    });

    assert!(result.is_ok(), "Service call failed");
    assert_eq!(result.unwrap().sum, 8, "Expected 5 + 3 = 8");

    server_handle.join().unwrap();
    println!("✅ Test passed: ros-z service call successful");
}

#[test]
fn test_ros_z_server_ros2_client() {
    if !check_ros2_available() {
        panic!("ros2 CLI not available");
    }

    ensure_zenohd_running();

    println!("\n=== Test: ros-z server <-> ROS2 client ===");

    // Start ros-z server
    let _server = thread::spawn(|| {
        let ctx = create_ros_z_context()
            .expect("Failed to create context");
        
        let node = ctx.create_node("rosz_server")
            .build()
            .expect("Failed to create node");
        
        let _service = node.create_service::<AddTwoInts>(
            "/add_two_ints",
            |req: AddTwoInts_Request| async move {
                AddTwoInts_Response { sum: req.a + req.b }
            }
        ).expect("Failed to create service");
        
        thread::sleep(Duration::from_secs(10));
    });

    wait_for_ready(Duration::from_secs(2));

    // Call from ros2 CLI
    let output = Command::new("timeout")
        .args([
            "5", "ros2", "service", "call",
            "/add_two_ints",
            "example_interfaces/srv/AddTwoInts",
            "{a: 10, b: 7}"
        ])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .output()
        .expect("Failed to call service");

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("sum: 17"), "Expected sum: 17, got: {}", stdout);

    println!("✅ Test passed: ROS2 client called ros-z service");
}

// Add more service tests...
