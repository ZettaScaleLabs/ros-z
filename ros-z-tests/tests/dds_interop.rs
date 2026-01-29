//! DDS Interoperability Tests
//!
//! These tests verify that ros-z can communicate with standard ROS 2 nodes
//! using DDS (CycloneDDS) via zenoh-bridge-ros2dds.
//!
//! Architecture:
//! ```
//! ┌─────────────────┐     DDS      ┌──────────────────────┐     Zenoh     ┌─────────────┐
//! │  ROS 2 Node     │◄────────────►│ zenoh-bridge-ros2dds │◄─────────────►│   ros-z     │
//! │  (CycloneDDS)   │              │     (router)         │               │ (ros2dds)   │
//! └─────────────────┘              └──────────────────────┘               └─────────────┘
//! ```
//!
//! Requirements:
//! - ROS 2 with CycloneDDS installed
//! - zenoh-bridge-ros2dds installed
//! - demo_nodes_cpp package available
//!
//! Run with:
//! ```bash
//! cargo test -p ros-z-tests --features ros2dds-interop
//! ```

#![cfg(feature = "ros2dds-interop")]

mod common;

use std::{
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use crate::common::*;

/// Test: ROS 2 CycloneDDS talker -> ros-z listener (via zenoh-bridge-ros2dds)
///
/// This test verifies that ros-z can receive messages from a standard ROS 2 node
/// using DDS, with zenoh-bridge-ros2dds translating between DDS and Zenoh.
#[test]
fn test_ros2_dds_pub_to_ros_z_sub() {
    if !check_ros2_available() {
        eprintln!("Skipping test: ros2 CLI not available");
        return;
    }
    if !check_demo_nodes_cpp_available() {
        eprintln!("Skipping test: demo_nodes_cpp not available");
        return;
    }
    if !check_zenoh_bridge_ros2dds_available() {
        eprintln!("Skipping test: zenoh-bridge-ros2dds not available");
        return;
    }

    println!("\n=== Test: ROS2 DDS publisher -> ros-z subscriber (via zenoh-bridge-ros2dds) ===");

    // Step 1: Start zenoh-bridge-ros2dds (acts as router)
    let _bridge = spawn_zenoh_bridge_ros2dds();
    println!("Bridge started");

    // Step 2: Start ROS 2 talker using CycloneDDS
    let _talker = spawn_ros2_cyclone_talker();
    println!("ROS 2 talker started");

    // Step 3: Start ros-z subscriber using ros2dds backend
    let received = Arc::new(Mutex::new(Vec::new()));
    let received_clone = received.clone();

    let listener_handle = thread::spawn(move || {
        tokio::runtime::Runtime::new().unwrap().block_on(async {
            let ctx = create_ros_z_context_ros2dds().expect("Failed to create ros-z context");

            // Create node and subscriber with ros2dds backend
            use ros_z::{
                Builder,
                backend::Ros2DdsBackend,
                qos::{QosHistory, QosProfile},
            };
            use ros_z_msgs::std_msgs::String as RosString;

            let node = ctx
                .create_node("listener")
                .build()
                .expect("Failed to create node");
            let qos = QosProfile {
                history: QosHistory::KeepLast(10),
                ..Default::default()
            };
            let subscriber = node
                .create_sub::<RosString>("chatter")
                .with_qos(qos)
                .with_backend::<Ros2DdsBackend>()
                .build()
                .expect("Failed to create subscriber");

            let mut messages = Vec::new();
            let start = std::time::Instant::now();
            let timeout = Duration::from_secs(15);
            let max_count = 3;

            // Receive messages
            loop {
                if start.elapsed() > timeout {
                    break;
                }

                match subscriber.recv_timeout(Duration::from_millis(100)) {
                    Ok(msg) => {
                        println!("I heard: [{}]", msg.data);
                        messages.push(msg.data.clone());
                        if messages.len() >= max_count {
                            break;
                        }
                    }
                    Err(_) => continue,
                }
            }

            let mut received = received_clone.lock().unwrap();
            *received = messages;
        });
    });

    listener_handle.join().expect("Listener thread panicked");

    let msgs = received.lock().unwrap();
    assert!(
        msgs.len() >= 3,
        "Test failed: Expected at least 3 messages, got {}",
        msgs.len()
    );

    println!(
        "Test passed: ros-z listener received {} messages from ROS 2 DDS talker via zenoh-bridge-ros2dds",
        msgs.len()
    );
}

/// Test: ros-z talker -> ROS 2 CycloneDDS listener (via zenoh-bridge-ros2dds)
///
/// This test verifies that ros-z can send messages to a standard ROS 2 node
/// using DDS, with zenoh-bridge-ros2dds translating between Zenoh and DDS.
#[test]
fn test_ros_z_pub_to_ros2_dds_sub() {
    if !check_ros2_available() {
        eprintln!("Skipping test: ros2 CLI not available");
        return;
    }
    if !check_demo_nodes_cpp_available() {
        eprintln!("Skipping test: demo_nodes_cpp not available");
        return;
    }
    if !check_zenoh_bridge_ros2dds_available() {
        eprintln!("Skipping test: zenoh-bridge-ros2dds not available");
        return;
    }

    println!("\n=== Test: ros-z publisher -> ROS2 DDS subscriber (via zenoh-bridge-ros2dds) ===");

    // Step 1: Start zenoh-bridge-ros2dds (acts as router)
    let _bridge = spawn_zenoh_bridge_ros2dds();
    println!("Bridge started");

    // Step 2: Start ROS 2 listener using CycloneDDS
    let _listener = spawn_ros2_cyclone_listener();
    println!("ROS 2 listener started");

    wait_for_ready(Duration::from_secs(2));

    // Step 3: Start ros-z publisher using ros2dds backend
    let talker_handle = thread::spawn(move || {
        tokio::runtime::Runtime::new().unwrap().block_on(async {
            let ctx = create_ros_z_context_ros2dds().expect("Failed to create ros-z context");

            // Create node and publisher with ros2dds backend
            use ros_z::{
                Builder,
                backend::Ros2DdsBackend,
                qos::{QosHistory, QosProfile},
            };
            use ros_z_msgs::std_msgs::String as RosString;

            let node = ctx
                .create_node("talker")
                .build()
                .expect("Failed to create node");
            let qos = QosProfile {
                history: QosHistory::KeepLast(7),
                ..Default::default()
            };
            let publisher = node
                .create_pub::<RosString>("chatter")
                .with_qos(qos)
                .with_backend::<Ros2DdsBackend>()
                .build()
                .expect("Failed to create publisher");

            let max_count = 10;
            let period = Duration::from_millis(500);

            // Publish messages
            for count in 1..=max_count {
                let msg = RosString {
                    data: format!("Hello World: {}", count),
                };
                println!("Publishing: '{}'", msg.data);
                publisher
                    .async_publish(&msg)
                    .await
                    .expect("Failed to publish");
                tokio::time::sleep(period).await;
            }
        });
    });

    talker_handle.join().expect("Talker thread panicked");

    // Give some time for ROS 2 listener to process
    wait_for_ready(Duration::from_secs(1));

    println!(
        "Test passed: ros-z talker published messages to ROS 2 DDS listener via zenoh-bridge-ros2dds"
    );
}

/// Test: ROS 2 CycloneDDS service server -> ros-z client (via zenoh-bridge-ros2dds)
///
/// This test verifies that ros-z can call services provided by a standard ROS 2 node
/// using DDS, with zenoh-bridge-ros2dds translating between DDS and Zenoh.
#[test]
fn test_ros2_dds_service_server_to_ros_z_client() {
    if !check_ros2_available() {
        eprintln!("Skipping test: ros2 CLI not available");
        return;
    }
    if !check_demo_nodes_cpp_available() {
        eprintln!("Skipping test: demo_nodes_cpp not available");
        return;
    }
    if !check_zenoh_bridge_ros2dds_available() {
        eprintln!("Skipping test: zenoh-bridge-ros2dds not available");
        return;
    }

    println!("\n=== Test: ROS2 DDS service server -> ros-z client (via zenoh-bridge-ros2dds) ===");

    // Step 1: Start zenoh-bridge-ros2dds (acts as router)
    let _bridge = spawn_zenoh_bridge_ros2dds();
    println!("Bridge started");

    // Step 2: Start ROS 2 service server using CycloneDDS
    let _server = spawn_ros2_cyclone_add_two_ints_server();
    println!("ROS 2 service server started");

    wait_for_ready(Duration::from_secs(2));

    // Step 3: Call service using ros-z client with ros2dds backend
    tokio::runtime::Runtime::new().unwrap().block_on(async {
        let ctx = create_ros_z_context_ros2dds().expect("Failed to create ros-z context");

        // Create node and client with ros2dds backend
        use ros_z::{Builder, backend::Ros2DdsBackend};
        use ros_z_msgs::example_interfaces::{AddTwoIntsRequest, srv::AddTwoInts};

        let node = ctx
            .create_node("add_two_ints_client")
            .build()
            .expect("Failed to create node");
        let client = node
            .create_client::<AddTwoInts>("add_two_ints")
            .with_backend::<Ros2DdsBackend>()
            .build()
            .expect("Failed to create client");

        let req = AddTwoIntsRequest { a: 15, b: 27 };
        println!("Sending request: {} + {}", req.a, req.b);

        client
            .send_request(&req)
            .await
            .expect("Failed to send request");

        match client.take_response_timeout(Duration::from_secs(10)) {
            Ok(resp) => {
                println!("Received response: {}", resp.sum);
                assert_eq!(resp.sum, 42, "Expected 15 + 27 = 42, got {}", resp.sum);
            }
            Err(e) => {
                panic!("Failed to receive response: {}", e);
            }
        }
    });

    println!("Test passed: ros-z client called ROS 2 DDS service server via zenoh-bridge-ros2dds");
}

/// Test: ros-z service server -> ROS 2 CycloneDDS client (via zenoh-bridge-ros2dds)
///
/// This test verifies that standard ROS 2 nodes can call services provided by ros-z
/// using DDS, with zenoh-bridge-ros2dds translating between Zenoh and DDS.
#[test]
fn test_ros_z_service_server_to_ros2_dds_client() {
    if !check_ros2_available() {
        eprintln!("Skipping test: ros2 CLI not available");
        return;
    }
    if !check_demo_nodes_cpp_available() {
        eprintln!("Skipping test: demo_nodes_cpp not available");
        return;
    }
    if !check_zenoh_bridge_ros2dds_available() {
        eprintln!("Skipping test: zenoh-bridge-ros2dds not available");
        return;
    }

    println!("\n=== Test: ros-z service server -> ROS2 DDS client (via zenoh-bridge-ros2dds) ===");

    // Step 1: Start zenoh-bridge-ros2dds (acts as router)
    let _bridge = spawn_zenoh_bridge_ros2dds();
    println!("Bridge started");

    // Step 2: Start ros-z service server using ros2dds backend
    let server_handle = std::thread::spawn(move || {
        tokio::runtime::Runtime::new().unwrap().block_on(async {
            let ctx = create_ros_z_context_ros2dds().expect("Failed to create ros-z context");

            // Create node and server with ros2dds backend
            use ros_z::{Builder, backend::Ros2DdsBackend};
            use ros_z_msgs::example_interfaces::{AddTwoIntsResponse, srv::AddTwoInts};

            let node = ctx
                .create_node("add_two_ints_server")
                .build()
                .expect("Failed to create node");
            let mut server = node
                .create_service::<AddTwoInts>("add_two_ints")
                .with_backend::<Ros2DdsBackend>()
                .build()
                .expect("Failed to create service server");

            println!("ros-z service server ready");

            // Handle one request
            match server.take_request_async().await {
                Ok((key, req)) => {
                    println!("Received request: {} + {}", req.a, req.b);
                    let resp = AddTwoIntsResponse { sum: req.a + req.b };
                    server
                        .send_response(&resp, &key)
                        .expect("Failed to send response");
                    println!("Sent response: {}", resp.sum);
                }
                Err(e) => {
                    eprintln!("Failed to receive request: {}", e);
                }
            }
        });
    });

    wait_for_ready(Duration::from_secs(3));

    // Step 3: Call service using ROS 2 client
    println!("Calling service from ROS 2 client");
    let _client = spawn_ros2_cyclone_add_two_ints_client(10, 20);

    // Wait for server to complete
    wait_for_ready(Duration::from_secs(3));
    server_handle.join().expect("Server thread panicked");

    println!("Test passed: ROS 2 DDS client called ros-z service server via zenoh-bridge-ros2dds");
}
