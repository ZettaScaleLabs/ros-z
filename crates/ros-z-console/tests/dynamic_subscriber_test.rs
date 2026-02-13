#![cfg(feature = "ros-interop")]

mod common;

use common::{ProcessGuard, TestRouter, check_ros2_available, spawn_ros2_topic_pub};
use serde_json::Value;
use std::io::{BufRead, BufReader};
use std::process::{Command, Stdio};
use std::thread;
use std::time::Duration;

/// Spawn ros-z-console in headless mode with echo topics
fn spawn_console_headless(router_endpoint: &str, echo_topics: &[&str]) -> ProcessGuard {
    use std::os::unix::process::CommandExt;

    let mut args = vec!["--headless", "--json", router_endpoint];

    for topic in echo_topics {
        args.push("--echo");
        args.push(topic);
    }

    let child = Command::new(env!("CARGO_BIN_EXE_ros-z-console"))
        .args(&args)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .process_group(0)
        .spawn()
        .expect("Failed to spawn ros-z-console");

    thread::sleep(Duration::from_millis(500));

    ProcessGuard::new(child, "ros-z-console")
}

#[test]
fn test_dynamic_subscriber_std_msgs_string() {
    if !check_ros2_available() {
        println!("Skipping test: ros2 CLI not available");
        return;
    }

    println!("\n=== Test: Dynamic subscriber with std_msgs/String ===");

    // Start Zenoh router
    let router = TestRouter::new();
    println!("Router endpoint: {}", router.endpoint());

    // Start ros-z-console with --echo /chatter
    let mut console = spawn_console_headless(router.endpoint(), &["/chatter"]);
    println!("Console started");

    // Start ros2 topic pub
    let _publisher = spawn_ros2_topic_pub(
        "/chatter",
        "std_msgs/msg/String",
        "{data: \"hello from ros2\"}",
        router.port,
    );
    println!("Publisher started");

    // Read console output and look for:
    // 1. topic_subscribed event with schema info
    // 2. message_received events with data
    let stdout = console
        .child
        .as_mut()
        .unwrap()
        .stdout
        .take()
        .expect("Failed to capture stdout");
    let reader = BufReader::new(stdout);

    let mut found_subscription = false;
    let mut found_message = false;
    let mut type_hash = String::new();

    for line in reader.lines().take(50) {
        if let Ok(line) = line {
            println!("Console output: {}", line);

            if let Ok(json) = serde_json::from_str::<Value>(&line) {
                if json["event"] == "topic_subscribed" {
                    assert_eq!(json["topic"], "/chatter");
                    assert_eq!(json["type_name"], "std_msgs/msg/String");

                    // Type hash should start with RIHS01_
                    if let Some(hash) = json["type_hash"].as_str() {
                        assert!(
                            hash.starts_with("RIHS01_"),
                            "Type hash should start with RIHS01_"
                        );
                        type_hash = hash.to_string();
                    }

                    // Schema should have "data" field
                    if let Some(fields) = json["fields"].as_array() {
                        assert!(
                            fields.iter().any(|f| f == "data"),
                            "Schema should contain 'data' field"
                        );
                    }

                    found_subscription = true;
                    println!(
                        "✓ Dynamic subscription successful with type hash: {}",
                        type_hash
                    );
                }

                if json["event"] == "message_received" {
                    assert_eq!(json["topic"], "/chatter");
                    assert_eq!(json["type"], "std_msgs/msg/String");

                    // Check that data field contains "hello from ros2"
                    if let Some(data) = json["data"]["data"].as_str() {
                        assert_eq!(data, "hello from ros2");
                        found_message = true;
                        println!("✓ Message received and decoded: {}", data);
                        break;
                    }
                }
            }
        }
    }

    assert!(found_subscription, "Should receive topic_subscribed event");
    assert!(found_message, "Should receive message_received event");
}

#[test]
fn test_dynamic_subscriber_sensor_msgs_laser_scan() {
    if !check_ros2_available() {
        println!("Skipping test: ros2 CLI not available");
        return;
    }

    println!("\n=== Test: Dynamic subscriber with sensor_msgs/LaserScan ===");

    let router = TestRouter::new();
    println!("Router endpoint: {}", router.endpoint());

    let mut console = spawn_console_headless(router.endpoint(), &["/scan"]);
    println!("Console started");

    let laser_scan_data = r#"{
        "header": {
          "stamp": {"sec": 0, "nanosec": 0},
          "frame_id": "laser_frame"
        },
        "angle_min": -1.57,
        "angle_max": 1.57,
        "angle_increment": 0.01,
        "time_increment": 0.0,
        "scan_time": 0.1,
        "range_min": 0.12,
        "range_max": 30.0,
        "ranges": [1.0, 2.0, 3.0],
        "intensities": [100.0, 100.0, 100.0]
    }"#;

    let _publisher = spawn_ros2_topic_pub(
        "/scan",
        "sensor_msgs/msg/LaserScan",
        laser_scan_data,
        router.port,
    );
    println!("Publisher started");

    let stdout = console
        .child
        .as_mut()
        .unwrap()
        .stdout
        .take()
        .expect("Failed to capture stdout");
    let reader = BufReader::new(stdout);

    let mut found_subscription = false;
    let mut found_message = false;

    for line in reader.lines().take(50) {
        if let Ok(line) = line {
            println!("Console output: {}", line);

            if let Ok(json) = serde_json::from_str::<Value>(&line) {
                if json["event"] == "topic_subscribed" {
                    assert_eq!(json["topic"], "/scan");
                    assert_eq!(json["type_name"], "sensor_msgs/msg/LaserScan");

                    if let Some(hash) = json["type_hash"].as_str() {
                        assert!(hash.starts_with("RIHS01_"));
                    }

                    // Check for key LaserScan fields
                    if let Some(fields) = json["fields"].as_array() {
                        let field_names: Vec<&str> =
                            fields.iter().filter_map(|f| f.as_str()).collect();

                        assert!(field_names.contains(&"header"));
                        assert!(field_names.contains(&"ranges"));
                        assert!(field_names.contains(&"intensities"));
                    }

                    found_subscription = true;
                    println!("✓ Dynamic subscription to LaserScan successful");
                }

                if json["event"] == "message_received" {
                    assert_eq!(json["topic"], "/scan");
                    assert_eq!(json["type"], "sensor_msgs/msg/LaserScan");

                    // Verify some fields from the data
                    let data = &json["data"];

                    // Check header.frame_id
                    if let Some(frame_id) = data["header"]["frame_id"].as_str() {
                        assert_eq!(frame_id, "laser_frame");
                    }

                    // Check ranges array
                    if let Some(ranges) = data["ranges"].as_array() {
                        assert_eq!(ranges.len(), 3);
                        assert_eq!(ranges[0].as_f64().unwrap(), 1.0);
                        assert_eq!(ranges[1].as_f64().unwrap(), 2.0);
                        assert_eq!(ranges[2].as_f64().unwrap(), 3.0);
                    }

                    found_message = true;
                    println!("✓ LaserScan message received and decoded correctly");
                    break;
                }
            }
        }
    }

    assert!(
        found_subscription,
        "Should receive topic_subscribed event for LaserScan"
    );
    assert!(found_message, "Should receive and decode LaserScan message");
}

#[test]
fn test_dynamic_subscriber_multiple_topics() {
    if !check_ros2_available() {
        println!("Skipping test: ros2 CLI not available");
        return;
    }

    println!("\n=== Test: Dynamic subscriber with multiple topics ===");

    let router = TestRouter::new();
    println!("Router endpoint: {}", router.endpoint());

    // Echo both /chatter and /scan
    let mut console = spawn_console_headless(router.endpoint(), &["/chatter", "/scan"]);
    println!("Console started");

    // Publish to both topics
    let _pub1 = spawn_ros2_topic_pub(
        "/chatter",
        "std_msgs/msg/String",
        "{data: \"multi-topic test\"}",
        router.port,
    );

    let _pub2 = spawn_ros2_topic_pub(
        "/scan",
        "sensor_msgs/msg/LaserScan",
        r#"{
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "test"},
            "angle_min": 0.0,
            "angle_max": 1.0,
            "angle_increment": 0.1,
            "time_increment": 0.0,
            "scan_time": 0.0,
            "range_min": 0.0,
            "range_max": 10.0,
            "ranges": [1.0],
            "intensities": [1.0]
        }"#,
        router.port,
    );

    println!("Publishers started");

    let stdout = console
        .child
        .as_mut()
        .unwrap()
        .stdout
        .take()
        .expect("Failed to capture stdout");
    let reader = BufReader::new(stdout);

    let mut subscribed_topics = std::collections::HashSet::new();
    let mut received_messages = std::collections::HashSet::new();

    for line in reader.lines().take(100) {
        if let Ok(line) = line {
            println!("Console output: {}", line);

            if let Ok(json) = serde_json::from_str::<Value>(&line) {
                if json["event"] == "topic_subscribed" {
                    if let Some(topic) = json["topic"].as_str() {
                        subscribed_topics.insert(topic.to_string());
                        println!("✓ Subscribed to: {}", topic);
                    }
                }

                if json["event"] == "message_received" {
                    if let Some(topic) = json["topic"].as_str() {
                        received_messages.insert(topic.to_string());
                        println!("✓ Received message from: {}", topic);
                    }
                }

                // Exit once we've received messages from both topics
                if received_messages.len() >= 2 {
                    break;
                }
            }
        }
    }

    assert!(
        subscribed_topics.contains("/chatter"),
        "Should subscribe to /chatter"
    );
    assert!(
        subscribed_topics.contains("/scan"),
        "Should subscribe to /scan"
    );
    assert!(
        received_messages.contains("/chatter"),
        "Should receive messages from /chatter"
    );
    assert!(
        received_messages.contains("/scan"),
        "Should receive messages from /scan"
    );
}
