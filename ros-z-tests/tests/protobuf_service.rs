#![cfg(feature = "interop-tests")]

mod common;

use std::{thread, time::Duration};

use crate::common::*;

#[test]
fn test_protobuf_service_ros_z_to_ros_z() {
    let router = TestRouter::new();

    println!("\n=== Test: protobuf service (ros-z server -> ros-z client) ===");

    // Start service server in a thread
    let router_endpoint = router.endpoint().to_string();
    let server_handle = thread::spawn(move || {
        let ctx = create_ros_z_context_with_endpoint(&router_endpoint)
            .expect("Failed to create ros-z context");

        // Use the actual server example code (handle 1 request)
        protobuf_demo::run_service_server(ctx, "/test_calculator", Some(1))
            .expect("Service server failed");
    });

    wait_for_ready(Duration::from_secs(2));

    // Run client in the main thread
    let ctx = create_ros_z_context_with_router(&router).expect("Failed to create ros-z context");

    let operations = vec![
        ("add", 10.0, 32.0), // This should give 42
    ];

    protobuf_demo::run_service_client(ctx, "/test_calculator", operations)
        .expect("Service client failed");

    // Wait for server to complete
    server_handle.join().expect("Server thread panicked");

    println!("Test passed: protobuf service client-server communication works");
}

#[test]
fn test_protobuf_service_multiple_calls() {
    let router = TestRouter::new();

    println!("\n=== Test: protobuf service with multiple operation types ===");

    // Start service server in a thread
    let router_endpoint = router.endpoint().to_string();
    let server_handle = thread::spawn(move || {
        let ctx = create_ros_z_context_with_endpoint(&router_endpoint)
            .expect("Failed to create ros-z context");

        // Handle 3 requests
        protobuf_demo::run_service_server(ctx, "/multi_calc", Some(3))
            .expect("Service server failed");
    });

    wait_for_ready(Duration::from_secs(2));

    // Run client with various operations
    let ctx = create_ros_z_context_with_router(&router).expect("Failed to create ros-z context");

    let operations = vec![
        ("add", 5.0, 7.0),     // = 12.0
        ("add", 100.0, 200.0), // = 300.0
        ("add", 15.0, 27.0),   // = 42.0
    ];

    protobuf_demo::run_service_client(ctx, "/multi_calc", operations)
        .expect("Service client failed");

    // Wait for server to complete
    server_handle.join().expect("Server thread panicked");

    println!("Test passed: protobuf service handles multiple operation types correctly");
}
