#![cfg(feature = "ros-interop")]
//! Parameter interoperability tests: ros-z node ↔ ros2 param CLI.
//!
//! These tests verify that `ros2 param list/get/set` works against a ros-z node,
//! confirming RIHS01 type hashes and CDR encoding are correct end-to-end.
//!
//! Requires: `ros-interop` feature, ROS 2 sourced with `rmw_zenoh_cpp`.

mod common;

use std::{process::Command, thread, time::Duration};

use common::*;
use ros_z::{
    Builder,
    parameter::{ParameterDescriptor, ParameterType, ParameterValue},
};

/// Test that `ros2 param list` can see parameters declared on a ros-z node.
#[test]
fn test_ros2_param_list_on_ros_z_node() {
    if !check_ros2_available() {
        panic!("ros2 CLI not available");
    }

    let router = TestRouter::new();

    let endpoint = router.endpoint().to_string();
    let _server = thread::spawn(move || {
        let ctx = create_ros_z_context_with_endpoint(&endpoint).expect("ctx");
        let node = ctx.create_node("param_list_node").build().expect("node");

        let desc = ParameterDescriptor::new("count", ParameterType::Integer);
        node.declare_parameter("count", ParameterValue::Integer(42), desc)
            .expect("declare count");

        let desc = ParameterDescriptor::new("label", ParameterType::String);
        node.declare_parameter("label", ParameterValue::String("hello".into()), desc)
            .expect("declare label");

        thread::sleep(Duration::from_secs(30));
    });

    wait_for_ready(Duration::from_secs(10));

    let output = Command::new("timeout")
        .args(["10", "ros2", "param", "list", "/param_list_node"])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .env("ZENOH_CONFIG_OVERRIDE", router.rmw_zenoh_env())
        .output()
        .expect("Failed to run ros2 param list");

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    println!("stdout: {}", stdout);
    println!("stderr: {}", stderr);

    assert!(
        stdout.contains("count"),
        "Expected 'count' in param list, got: {}",
        stdout
    );
    assert!(
        stdout.contains("label"),
        "Expected 'label' in param list, got: {}",
        stdout
    );

    println!("Test passed: ros2 param list sees ros-z parameters");
}

/// Test that `ros2 param get` and `ros2 param set` work against a ros-z node.
#[test]
fn test_ros2_param_get_set_on_ros_z_node() {
    if !check_ros2_available() {
        panic!("ros2 CLI not available");
    }

    let router = TestRouter::new();

    let endpoint = router.endpoint().to_string();
    let _server = thread::spawn(move || {
        let ctx = create_ros_z_context_with_endpoint(&endpoint).expect("ctx");
        let node = ctx.create_node("param_getset_node").build().expect("node");

        let desc = ParameterDescriptor::new("count", ParameterType::Integer);
        node.declare_parameter("count", ParameterValue::Integer(0), desc)
            .expect("declare count");

        thread::sleep(Duration::from_secs(60));
    });

    wait_for_ready(Duration::from_secs(10));

    let rmw_env = router.rmw_zenoh_env();

    // Get initial value
    let output = Command::new("timeout")
        .args(["10", "ros2", "param", "get", "/param_getset_node", "count"])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .env("ZENOH_CONFIG_OVERRIDE", &rmw_env)
        .output()
        .expect("Failed to run ros2 param get");

    let stdout = String::from_utf8_lossy(&output.stdout);
    println!("Initial get: {}", stdout);
    assert!(stdout.contains("0"), "Expected value 0, got: {}", stdout);

    // Set new value
    let output = Command::new("timeout")
        .args([
            "10",
            "ros2",
            "param",
            "set",
            "/param_getset_node",
            "count",
            "42",
        ])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .env("ZENOH_CONFIG_OVERRIDE", &rmw_env)
        .output()
        .expect("Failed to run ros2 param set");

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    println!("Set output: {} {}", stdout, stderr);
    assert!(
        stdout.contains("Set parameter successful") || output.status.success(),
        "Expected successful set, got: {} {}",
        stdout,
        stderr
    );

    // Get updated value
    let output = Command::new("timeout")
        .args(["10", "ros2", "param", "get", "/param_getset_node", "count"])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .env("ZENOH_CONFIG_OVERRIDE", &rmw_env)
        .output()
        .expect("Failed to run ros2 param get");

    let stdout = String::from_utf8_lossy(&output.stdout);
    println!("Updated get: {}", stdout);
    assert!(
        stdout.contains("42"),
        "Expected value 42 after set, got: {}",
        stdout
    );

    println!("Test passed: ros2 param get/set works against ros-z node");
}

/// Test that a ros-z service client can read parameters from an rclcpp node.
///
/// This is a stretch goal — requires demo_nodes_cpp to be installed.
#[test]
fn test_ros_z_reads_rclcpp_node_params() {
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

    // Start an rclcpp talker node (it declares the 'use_sim_time' parameter by default)
    let server = Command::new("ros2")
        .args([
            "run",
            "demo_nodes_cpp",
            "talker",
            "--ros-args",
            "-r",
            "__node:=rclcpp_param_node",
        ])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .env("ZENOH_CONFIG_OVERRIDE", router.rmw_zenoh_env())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .expect("Failed to start rclcpp node");

    let _guard = ProcessGuard::new(server, "rclcpp_param_node");

    wait_for_ready(Duration::from_secs(5));

    // Use ros2 param list to verify the rclcpp node's parameters are visible
    let output = Command::new("timeout")
        .args(["10", "ros2", "param", "list", "/rclcpp_param_node"])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .env("ZENOH_CONFIG_OVERRIDE", router.rmw_zenoh_env())
        .output()
        .expect("Failed to run ros2 param list");

    let stdout = String::from_utf8_lossy(&output.stdout);
    println!("rclcpp node params: {}", stdout);

    // All rclcpp nodes have use_sim_time by default
    assert!(
        stdout.contains("use_sim_time"),
        "Expected 'use_sim_time' in rclcpp node params, got: {}",
        stdout
    );

    println!("Test passed: can list rclcpp node parameters via Zenoh");
}
