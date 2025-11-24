// Ported from Open Source Robotics Foundation code (2018)
// https://github.com/ros2/rcl
// Licensed under the Apache License 2.0
// Copyright 2025 ZettaScale Technology

#![allow(clippy::needless_return)]
#![cfg(feature = "test-msgs")]

mod test_msgs_support;

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
        rcl_publisher_fini, rcl_publisher_get_default_options,
        rcl_publisher_get_subscription_count, rcl_publisher_init, rcl_subscription_fini,
        rcl_subscription_get_default_options, rcl_subscription_get_publisher_count,
        rcl_subscription_init,
    },
    ros::*,
};

/// Test fixture that provides an initialized RCL context and node
struct TestCountFixture {
    context: rcl_context_t,
    node: rcl_node_t,
}

impl TestCountFixture {
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
            let node_name = c"test_count_node";
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

            TestCountFixture { context, node }
        }
    }

    fn node(&mut self) -> *mut rcl_node_t {
        &mut self.node
    }
}

impl Drop for TestCountFixture {
    fn drop(&mut self) {
        let ret = rcl_node_fini(&mut self.node);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize node");

        let ret = rcl_shutdown(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to shutdown context");

        let ret = rcl_context_fini(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize context");
    }
}

/// Test rcl_publisher_get_subscription_count
#[test]
fn test_publisher_get_subscription_count() {
    let mut fixture = TestCountFixture::new();

    unsafe {
        let mut publisher = rcl_get_zero_initialized_publisher();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"test_count_topic";
        let publisher_options = rcl_publisher_get_default_options();

        // Initialize publisher
        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize publisher");

        // Test valid call
        let mut count: usize = 999;
        let ret = rcl_publisher_get_subscription_count(&publisher, &mut count);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to get subscription count");
        // For Zenoh implementation, this should return 0
        assert_eq!(count, 0, "Expected subscription count to be 0");

        // Test null publisher
        let ret = rcl_publisher_get_subscription_count(ptr::null(), &mut count);
        assert_eq!(
            ret, RCL_RET_PUBLISHER_INVALID as i32,
            "Expected error for null publisher"
        );

        // Test null count
        let ret = rcl_publisher_get_subscription_count(&publisher, ptr::null_mut());
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Expected error for null count"
        );

        // Finalize publisher
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize publisher");
    }
}

/// Test rcl_subscription_get_publisher_count
#[test]
fn test_subscription_get_publisher_count() {
    let mut fixture = TestCountFixture::new();

    unsafe {
        let mut subscription = rcl_get_zero_initialized_subscription();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"test_count_topic";
        let subscription_options = rcl_subscription_get_default_options();

        // Initialize subscription
        let ret = rcl_subscription_init(
            &mut subscription,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &subscription_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize subscription");

        // Test valid call
        let mut count: usize = 999;
        let ret = rcl_subscription_get_publisher_count(&subscription, &mut count);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to get publisher count");
        // For Zenoh implementation, this should return 0
        assert_eq!(count, 0, "Expected publisher count to be 0");

        // Test null subscription
        let ret = rcl_subscription_get_publisher_count(ptr::null(), &mut count);
        assert_eq!(
            ret, RCL_RET_SUBSCRIPTION_INVALID as i32,
            "Expected error for null subscription"
        );

        // Test null count
        let ret = rcl_subscription_get_publisher_count(&subscription, ptr::null_mut());
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Expected error for null count"
        );

        // Finalize subscription
        let ret = rcl_subscription_fini(&mut subscription, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize subscription");
    }
}
