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

#[test]
fn test_protobuf_service_comprehensive_operations() {
    let router = TestRouter::new();

    println!("\n=== Test: protobuf service comprehensive operations ===");

    // Start service server in a thread
    let router_endpoint = router.endpoint().to_string();
    let server_handle = thread::spawn(move || {
        let ctx = create_ros_z_context_with_endpoint(&router_endpoint)
            .expect("Failed to create ros-z context");

        // Handle 10 requests
        protobuf_demo::run_service_server(ctx, "/comprehensive_calc", Some(10))
            .expect("Service server failed");
    });

    wait_for_ready(Duration::from_secs(2));

    // Run client with comprehensive operations
    let ctx = create_ros_z_context_with_router(&router).expect("Failed to create ros-z context");

    let operations = vec![
        ("add", 100.0, 50.0),      // = 150.0
        ("subtract", 100.0, 50.0), // = 50.0
        ("multiply", 100.0, 50.0), // = 5000.0
        ("divide", 100.0, 50.0),   // = 2.0
        ("add", -5.0, 3.0),        // = -2.0
        ("multiply", -5.0, 3.0),   // = -15.0
        ("divide", 1.0, 3.0),      // ≈ 0.333...
        ("divide", 10.0, 0.0),     // Error: division by zero
        ("unknown", 1.0, 1.0),     // Error: unknown operation
        ("add", 0.0, 0.0),         // = 0.0
    ];

    protobuf_demo::run_service_client(ctx, "/comprehensive_calc", operations)
        .expect("Service client failed");

    // Wait for server to complete
    server_handle.join().expect("Server thread panicked");

    println!("Test passed: protobuf service handles comprehensive operation types correctly");
}

#[test]
fn test_protobuf_service_error_handling() {
    let router = TestRouter::new();

    println!("\n=== Test: protobuf service error handling ===");

    // Start service server in a thread
    let router_endpoint = router.endpoint().to_string();
    let server_handle = thread::spawn(move || {
        let ctx = create_ros_z_context_with_endpoint(&router_endpoint)
            .expect("Failed to create ros-z context");

        // Handle 3 error cases
        protobuf_demo::run_service_server(ctx, "/error_calc", Some(3))
            .expect("Service server failed");
    });

    wait_for_ready(Duration::from_secs(2));

    // Run client with error cases
    let ctx = create_ros_z_context_with_router(&router).expect("Failed to create ros-z context");

    let operations = vec![
        ("divide", 10.0, 0.0),    // Error: division by zero
        ("invalid", 5.0, 5.0),    // Error: unknown operation
        ("unknown_op", 1.0, 1.0), // Error: unknown operation
    ];

    protobuf_demo::run_service_client(ctx, "/error_calc", operations)
        .expect("Service client failed");

    // Wait for server to complete
    server_handle.join().expect("Server thread panicked");

    println!("Test passed: protobuf service error handling works correctly");
}
