#![cfg(feature = "interop-tests")]

mod common;

use std::{
    os::unix::process::CommandExt,
    process::{Command, Stdio},
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use common::*;
use ros_z::Builder;
use ros_z_msgs::std_msgs::String as RosString;

#[test]
fn test_ros_z_pub_to_ros2_sub() {
    if !check_ros2_available() {
        panic!("ros2 CLI not available - ensure ROS 2 is installed");
    }

    let router = TestRouter::new();

    println!("\n=== Test: ros-z publisher -> ROS2 subscriber ===");

    // Start ROS2 subscriber
    let subscriber = Command::new("ros2")
        .args(["topic", "echo", "/chatter", "std_msgs/msg/String", "--once"])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .env("ZENOH_CONFIG_OVERRIDE", router.rmw_zenoh_env())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .process_group(0)
        .spawn()
        .expect("Failed to start ROS2 subscriber");

    let _sub_guard = ProcessGuard::new(subscriber, "ros2 subscriber");

    wait_for_ready(Duration::from_secs(2));

    // Start ros-z publisher
    let publisher_handle = thread::spawn(move || {
        let ctx =
            create_ros_z_context_with_router(&router).expect("Failed to create ros-z context");

        let node = ctx
            .create_node("test_publisher")
            .build()
            .expect("Failed to create node");

        let publisher = node
            .create_pub::<RosString>("/chatter")
            .build()
            .expect("Failed to create publisher");

        for i in 0..5 {
            let msg = RosString {
                data: format!("Hello from ros-z {}", i),
            };
            publisher.publish(&msg).expect("Failed to publish");
            println!("Published message {}", i);
            thread::sleep(Duration::from_millis(500));
        }
    });

    publisher_handle.join().expect("Publisher thread panicked");
    wait_for_ready(Duration::from_secs(2));

    println!("Test passed: ROS2 subscriber received message");
}

#[test]
fn test_ros2_pub_to_ros_z_sub() {
    if !check_ros2_available() {
        panic!("ros2 CLI not available - ensure ROS 2 is installed");
    }

    let router = TestRouter::new();

    println!("\n=== Test: ROS2 publisher -> ros-z subscriber ===");

    let received = Arc::new(Mutex::new(false));
    let received_clone = received.clone();

    let router_endpoint = router.endpoint().to_string();
    let subscriber_handle = thread::spawn(move || {
        let ctx = create_ros_z_context_with_endpoint(&router_endpoint)
            .expect("Failed to create ros-z context");

        let node = ctx
            .create_node("test_subscriber")
            .build()
            .expect("Failed to create node");

        let subscriber = node
            .create_sub::<RosString>("/chatter")
            .build()
            .expect("Failed to create subscriber");

        println!("ros-z subscriber waiting for messages...");

        let start = std::time::Instant::now();
        let timeout = Duration::from_secs(10);

        while start.elapsed() < timeout {
            match subscriber.recv_timeout(Duration::from_millis(100)) {
                Ok(msg) => {
                    println!("Received message: {}", msg.data);
                    if msg.data.contains("Hello from ROS2") {
                        let mut flag = received_clone.lock().unwrap();
                        *flag = true;
                        break;
                    }
                }
                Err(_) => continue,
            }
        }
    });

    wait_for_ready(Duration::from_secs(2));

    let publisher = Command::new("ros2")
        .args([
            "topic",
            "pub",
            "/chatter",
            "std_msgs/msg/String",
            "{data: 'Hello from ROS2'}",
            "--rate",
            "1",
            "--times",
            "5",
        ])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .env("ZENOH_CONFIG_OVERRIDE", router.rmw_zenoh_env())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .process_group(0)
        .spawn()
        .expect("Failed to start ROS2 publisher");

    let _pub_guard = ProcessGuard::new(publisher, "ros2 publisher");

    subscriber_handle
        .join()
        .expect("Subscriber thread panicked");

    let received_flag = received.lock().unwrap();
    assert!(
        *received_flag,
        "Test failed: Did not receive message from ROS2"
    );

    println!("Test passed: Received message from ROS2");
}

// Add more pubsub tests...
