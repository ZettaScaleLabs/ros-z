// Copyright 2025 ZettaScale Technology
// SPDX-License-Identifier: Apache-2.0
//
// Ported from ros2/rcl:
// Copyright 2019 Open Source Robotics Foundation, Inc.

#![cfg(feature = "test-msgs")]

use std::ptr;

use rcl_z::{
    context::{rcl_context_fini, rcl_get_zero_initialized_context, rcl_shutdown},
    init::{
        rcl_get_default_allocator, rcl_get_zero_initialized_init_options, rcl_init,
        rcl_init_options_fini, rcl_init_options_init,
    },
    node::{
        rcl_get_zero_initialized_node, rcl_node_fini, rcl_node_get_default_options, rcl_node_init,
    },
    pubsub::{
        rcl_get_zero_initialized_publisher, rcl_get_zero_initialized_subscription,
        rcl_publisher_fini, rcl_publisher_get_actual_qos, rcl_publisher_get_default_options,
        rcl_publisher_init, rcl_subscription_fini, rcl_subscription_get_actual_qos,
        rcl_subscription_get_default_options, rcl_subscription_init,
    },
    ros::*,
};

mod test_msgs_support;

/// Test fixture that provides an initialized RCL context and node
struct TestGetActualQoSFixture {
    context: rcl_context_t,
    node: rcl_node_t,
}

impl TestGetActualQoSFixture {
    fn new() -> Self {
        unsafe {
            // Initialize context
            let mut init_options = rcl_get_zero_initialized_init_options();
            let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize init options");

            let mut context = rcl_get_zero_initialized_context();
            let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize context");

            let _ = rcl_init_options_fini(&mut init_options);

            // Initialize node
            let mut node = rcl_get_zero_initialized_node();
            let node_name = c"test_get_actual_qos_node";
            let namespace = c"";
            let node_options = rcl_node_get_default_options();
            let ret = rcl_node_init(
                &mut node,
                node_name.as_ptr(),
                namespace.as_ptr(),
                &mut context,
                &node_options,
            );
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize node");

            TestGetActualQoSFixture { context, node }
        }
    }

    fn node(&mut self) -> *mut rcl_node_t {
        &mut self.node
    }
}

impl Drop for TestGetActualQoSFixture {
    fn drop(&mut self) {
        let ret = rcl_node_fini(&mut self.node);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize node");

        let ret = rcl_shutdown(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to shutdown context");

        let ret = rcl_context_fini(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize context");
    }
}

/// Test that publisher returns non-null QoS profile
#[test]
fn test_publisher_get_actual_qos_returns_profile() {
    let mut fixture = TestGetActualQoSFixture::new();

    unsafe {
        let mut publisher = rcl_get_zero_initialized_publisher();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"/test_publisher_get_actual_qos";
        let publisher_options = rcl_publisher_get_default_options();

        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize publisher");

        // Get actual QoS
        let qos = rcl_publisher_get_actual_qos(&publisher);
        assert!(!qos.is_null(), "QoS profile should not be null");

        // Verify it returns the same QoS we set
        let qos_ref = &*qos;
        assert_eq!(
            qos_ref.history, publisher_options.qos.history,
            "History policy mismatch"
        );
        assert_eq!(qos_ref.depth, publisher_options.qos.depth, "Depth mismatch");
        assert_eq!(
            qos_ref.reliability, publisher_options.qos.reliability,
            "Reliability policy mismatch"
        );
        assert_eq!(
            qos_ref.durability, publisher_options.qos.durability,
            "Durability policy mismatch"
        );

        // Finalize
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize publisher");
    }
}

/// Test that subscription returns non-null QoS profile
#[test]
fn test_subscription_get_actual_qos_returns_profile() {
    let mut fixture = TestGetActualQoSFixture::new();

    unsafe {
        let mut subscription = rcl_get_zero_initialized_subscription();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"/test_subscription_get_actual_qos";
        let subscription_options = rcl_subscription_get_default_options();

        let ret = rcl_subscription_init(
            &mut subscription,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &subscription_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize subscription");

        // Get actual QoS
        let qos = rcl_subscription_get_actual_qos(&subscription);
        assert!(!qos.is_null(), "QoS profile should not be null");

        // Verify it returns the same QoS we set
        let qos_ref = &*qos;
        assert_eq!(
            qos_ref.history, subscription_options.qos.history,
            "History policy mismatch"
        );
        assert_eq!(
            qos_ref.depth, subscription_options.qos.depth,
            "Depth mismatch"
        );
        assert_eq!(
            qos_ref.reliability, subscription_options.qos.reliability,
            "Reliability policy mismatch"
        );
        assert_eq!(
            qos_ref.durability, subscription_options.qos.durability,
            "Durability policy mismatch"
        );

        // Finalize
        let ret = rcl_subscription_fini(&mut subscription, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize subscription");
    }
}

/// Test publisher QoS with custom settings
#[test]
fn test_publisher_get_actual_qos_with_custom_settings() {
    let mut fixture = TestGetActualQoSFixture::new();

    unsafe {
        let mut publisher = rcl_get_zero_initialized_publisher();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"/test_publisher_custom_qos";
        let mut publisher_options = rcl_publisher_get_default_options();

        // Customize QoS settings
        publisher_options.qos.history = rmw_qos_history_policy_e::KEEP_ALL;
        publisher_options.qos.depth = 100;
        publisher_options.qos.reliability = rmw_qos_reliability_policy_e::BEST_EFFORT;
        publisher_options.qos.durability = rmw_qos_durability_policy_e::TRANSIENT_LOCAL;

        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize publisher");

        // Get actual QoS
        let qos = rcl_publisher_get_actual_qos(&publisher);
        assert!(!qos.is_null(), "QoS profile should not be null");

        let qos_ref = &*qos;
        assert_eq!(
            qos_ref.history,
            rmw_qos_history_policy_e::KEEP_ALL,
            "History policy should be KEEP_ALL"
        );
        assert_eq!(qos_ref.depth, 100, "Depth should be 100");
        assert_eq!(
            qos_ref.reliability,
            rmw_qos_reliability_policy_e::BEST_EFFORT,
            "Reliability should be BEST_EFFORT"
        );
        assert_eq!(
            qos_ref.durability,
            rmw_qos_durability_policy_e::TRANSIENT_LOCAL,
            "Durability should be TRANSIENT_LOCAL"
        );

        // Finalize
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize publisher");
    }
}

/// Test subscription QoS with custom settings
#[test]
fn test_subscription_get_actual_qos_with_custom_settings() {
    let mut fixture = TestGetActualQoSFixture::new();

    unsafe {
        let mut subscription = rcl_get_zero_initialized_subscription();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"/test_subscription_custom_qos";
        let mut subscription_options = rcl_subscription_get_default_options();

        // Customize QoS settings
        subscription_options.qos.history = rmw_qos_history_policy_e::KEEP_LAST;
        subscription_options.qos.depth = 50;
        subscription_options.qos.reliability = rmw_qos_reliability_policy_e::RELIABLE;
        subscription_options.qos.durability = rmw_qos_durability_policy_e::VOLATILE;

        let ret = rcl_subscription_init(
            &mut subscription,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &subscription_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize subscription");

        // Get actual QoS
        let qos = rcl_subscription_get_actual_qos(&subscription);
        assert!(!qos.is_null(), "QoS profile should not be null");

        let qos_ref = &*qos;
        assert_eq!(
            qos_ref.history,
            rmw_qos_history_policy_e::KEEP_LAST,
            "History policy should be KEEP_LAST"
        );
        assert_eq!(qos_ref.depth, 50, "Depth should be 50");
        assert_eq!(
            qos_ref.reliability,
            rmw_qos_reliability_policy_e::RELIABLE,
            "Reliability should be RELIABLE"
        );
        assert_eq!(
            qos_ref.durability,
            rmw_qos_durability_policy_e::VOLATILE,
            "Durability should be VOLATILE"
        );

        // Finalize
        let ret = rcl_subscription_fini(&mut subscription, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize subscription");
    }
}

/// Test that get_actual_qos returns null for null publisher
#[test]
fn test_publisher_get_actual_qos_null_publisher() {
    let qos = rcl_publisher_get_actual_qos(ptr::null());
    assert!(qos.is_null(), "QoS should be null for null publisher");
}

/// Test that get_actual_qos returns null for null subscription
#[test]
fn test_subscription_get_actual_qos_null_subscription() {
    let qos = rcl_subscription_get_actual_qos(ptr::null());
    assert!(qos.is_null(), "QoS should be null for null subscription");
}

/// Test that get_actual_qos returns null for uninitialized publisher
#[test]
fn test_publisher_get_actual_qos_uninitialized() {
    let publisher = rcl_get_zero_initialized_publisher();
    let qos = rcl_publisher_get_actual_qos(&publisher);
    assert!(
        qos.is_null(),
        "QoS should be null for uninitialized publisher"
    );
}

/// Test that get_actual_qos returns null for uninitialized subscription
#[test]
fn test_subscription_get_actual_qos_uninitialized() {
    let subscription = rcl_get_zero_initialized_subscription();
    let qos = rcl_subscription_get_actual_qos(&subscription);
    assert!(
        qos.is_null(),
        "QoS should be null for uninitialized subscription"
    );
}
