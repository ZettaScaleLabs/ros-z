#![cfg(feature = "interop-tests")]

mod common;

use std::{
    process::{Command, Stdio},
    thread,
    time::Duration,
};

use common::*;
use ros_z::Builder;
use ros_z_msgs::example_interfaces::{AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse};

#[test]
fn test_ros_z_server_ros_z_client() {
    let router = TestRouter::new();

    println!("\n=== Test: ros-z server <-> ros-z client ===");

    // Start server in background thread
    let router_endpoint = router.endpoint().to_string();
    let _server_handle = thread::spawn(move || {
        let ctx =
            create_ros_z_context_with_endpoint(&router_endpoint).expect("Failed to create context");

        let node = ctx
            .create_node("test_server")
            .build()
            .expect("Failed to create node");

        // Correct API: use tuple type and take_request/send_response pattern
        let mut zsrv = node
            .create_service::<AddTwoInts>("add_two_ints_test1")
            .build()
            .expect("Failed to create service");

        println!("Server ready, waiting for requests...");

        // Handle one request
        if let Ok((key, req)) = zsrv.take_request() {
            println!("Received request: {} + {}", req.a, req.b);
            let resp = AddTwoIntsResponse { sum: req.a + req.b };
            println!("Sending response: {}", resp.sum);
            zsrv.send_response(&resp, &key)
                .expect("Failed to send response");
        }
    });

    wait_for_ready(Duration::from_secs(3));

    // Run client
    let client_handle = thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async {
            let ctx = create_ros_z_context_with_router(&router).expect("Failed to create context");

            let node = ctx
                .create_node("test_client")
                .build()
                .expect("Failed to create node");

            let zcli = node
                .create_client::<AddTwoInts>("add_two_ints_test1")
                .build()
                .expect("Failed to create client");

            println!("Client ready, waiting for service discovery...");
            // Give some time for service discovery to complete
            tokio::time::sleep(Duration::from_millis(500)).await;

            println!("Sending request...");

            let req = AddTwoIntsRequest { a: 5, b: 3 };
            zcli.send_request(&req)
                .await
                .expect("Failed to send request");

            let resp = zcli
                .take_response_timeout(Duration::from_secs(5))
                .expect("Failed to receive response");
            println!("Received response: {}", resp.sum);

            assert_eq!(resp.sum, 8, "Expected 5 + 3 = 8");
            resp
        })
    });

    let result = client_handle.join().expect("Client thread panicked");
    assert_eq!(result.sum, 8);
    println!("✅ Test passed: ros-z service call successful");
}

#[test]
fn test_ros_z_server_ros2_client() {
    if !check_ros2_available() {
        panic!("ros2 CLI not available");
    }

    let router = TestRouter::new();

    println!("\n=== Test: ros-z server <-> ROS2 client ===");

    // Start ros-z server
    let router_endpoint = router.endpoint().to_string();
    let _server = thread::spawn(move || {
        let ctx =
            create_ros_z_context_with_endpoint(&router_endpoint).expect("Failed to create context");

        let node = ctx
            .create_node("rosz_server")
            .build()
            .expect("Failed to create node");

        let mut zsrv = node
            .create_service::<AddTwoInts>("add_two_ints_test2")
            .build()
            .expect("Failed to create service");

        println!("Server ready for ROS2 client...");

        // Handle one request
        if let Ok((key, req)) = zsrv.take_request() {
            println!("Received request from ROS2: {} + {}", req.a, req.b);
            let resp = AddTwoIntsResponse { sum: req.a + req.b };
            println!("Sending response: {}", resp.sum);
            zsrv.send_response(&resp, &key)
                .expect("Failed to send response");
        }
    });

    wait_for_ready(Duration::from_secs(10));

    // Call from ros2 CLI
    let output = Command::new("timeout")
        .args([
            "5",
            "ros2",
            "service",
            "call",
            "/add_two_ints_test2",
            "example_interfaces/srv/AddTwoInts",
            "{a: 10, b: 7}",
        ])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .env("ZENOH_CONFIG_OVERRIDE", router.rmw_zenoh_env())
        .output()
        .expect("Failed to call service");

    let stdout = String::from_utf8_lossy(&output.stdout);
    println!("ROS2 output: {}", stdout);
    assert!(
        stdout.contains("sum: 17") || stdout.contains("sum=17"),
        "Expected sum: 17, got: {}",
        stdout
    );

    println!("✅ Test passed: ROS2 client called ros-z service");
}

#[test]
fn test_ros2_server_ros_z_client() {
    if !check_ros2_available() {
        panic!("ros2 CLI not available");
    }

    if !check_demo_nodes_cpp_available() {
        panic!(
            "demo_nodes_cpp package not found!\n\
             Please install it with: apt install ros-$ROS_DISTRO-demo-nodes-cpp\n\
             Or ensure ROS environment is sourced: source /opt/ros/$ROS_DISTRO/setup.bash"
        );
    }

    let router = TestRouter::new();

    println!("\n=== Test: ROS2 server <-> ros-z client ===");

    // Start ROS2 server with service name remapping to avoid conflicts
    let server = Command::new("ros2")
        .args([
            "run",
            "demo_nodes_cpp",
            "add_two_ints_server",
            "--ros-args",
            "-r",
            "add_two_ints:=add_two_ints_test_ros2",
        ])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .env("ZENOH_CONFIG_OVERRIDE", router.rmw_zenoh_env())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("Failed to start ROS2 server");

    let _guard = ProcessGuard::new(server, "ros2 server");

    wait_for_ready(Duration::from_secs(8));

    // Call from ros-z client
    let result = thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async {
            let ctx = create_ros_z_context_with_router(&router).expect("Failed to create context");

            let node = ctx
                .create_node("rosz_client")
                .build()
                .expect("Failed to create node");

            let zcli = node
                .create_client::<AddTwoInts>("add_two_ints_test_ros2")
                .build()
                .expect("Failed to create client");

            println!("Client ready, waiting for service discovery...");
            // Give some time for service discovery to complete
            tokio::time::sleep(Duration::from_millis(500)).await;

            println!("Calling ROS2 server...");

            let req = AddTwoIntsRequest { a: 15, b: 9 };
            zcli.send_request(&req)
                .await
                .expect("Failed to send request");

            let resp = zcli
                .take_response_timeout(std::time::Duration::from_secs(5))
                .expect("Failed to receive response");
            println!("Received response from ROS2: {}", resp.sum);

            resp
        })
    })
    .join()
    .expect("Client thread panicked");

    assert_eq!(result.sum, 24, "Expected 15 + 9 = 24");

    println!("✅ Test passed: ros-z client called ROS2 service");
}
