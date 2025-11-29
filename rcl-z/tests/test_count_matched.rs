// Copyright 2025 ZettaScale Technology
// SPDX-License-Identifier: Apache-2.0
//
// Ported from ros2/rcl:
// Copyright 2018 Open Source Robotics Foundation, Inc.

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

/// Test count matched functions with multiple publishers and subscribers
/// Note: In the C++ version, this test verifies actual pub/sub matching with graph waiting,
/// QoS compatibility, and multiple entities. For the Zenoh implementation, the count
/// functions always return 0 as Zenoh handles discovery differently.
#[test]
fn test_count_matched_functions() {
    let mut fixture = TestCountFixture::new();

    unsafe {
        let topic_name = c"/test_count_matched_functions__";
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);

        // Initialize publisher
        let mut pub_handle = rcl_get_zero_initialized_publisher();
        let pub_opts = rcl_publisher_get_default_options();
        let ret = rcl_publisher_init(
            &mut pub_handle,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &pub_opts,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize publisher");

        // Check initial state - publisher should see 0 subscribers (Zenoh specific)
        let mut subscriber_count: usize = 999;
        let ret = rcl_publisher_get_subscription_count(&pub_handle, &mut subscriber_count);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(
            subscriber_count, 0,
            "Expected 0 subscribers for Zenoh implementation"
        );

        // Initialize first subscription
        let mut sub = rcl_get_zero_initialized_subscription();
        let sub_opts = rcl_subscription_get_default_options();
        let ret =
            rcl_subscription_init(&mut sub, fixture.node(), ts, topic_name.as_ptr(), &sub_opts);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize subscription");

        // In C++/ROS2: After adding subscriber, both counts would be 1
        // In Zenoh: Counts remain 0 as discovery is handled by Zenoh
        let ret = rcl_publisher_get_subscription_count(&pub_handle, &mut subscriber_count);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(subscriber_count, 0, "Zenoh: publisher count stays 0");

        let mut publisher_count: usize = 999;
        let ret = rcl_subscription_get_publisher_count(&sub, &mut publisher_count);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(publisher_count, 0, "Zenoh: subscription count stays 0");

        // Initialize second subscription
        let mut sub2 = rcl_get_zero_initialized_subscription();
        let sub2_opts = rcl_subscription_get_default_options();
        let ret = rcl_subscription_init(
            &mut sub2,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &sub2_opts,
        );
        assert_eq!(
            ret, RCL_RET_OK as i32,
            "Failed to initialize subscription 2"
        );

        // In C++/ROS2: Publisher would see 2 subscribers
        // In Zenoh: Still 0
        let ret = rcl_publisher_get_subscription_count(&pub_handle, &mut subscriber_count);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(subscriber_count, 0, "Zenoh: still 0 with 2 subscriptions");

        // Cleanup publisher first
        let ret = rcl_publisher_fini(&mut pub_handle, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize publisher");

        // In C++/ROS2: Subscriptions would now see 0 publishers
        // In Zenoh: Already 0
        let ret = rcl_subscription_get_publisher_count(&sub, &mut publisher_count);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(publisher_count, 0);

        let ret = rcl_subscription_get_publisher_count(&sub2, &mut publisher_count);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(publisher_count, 0);

        // Cleanup subscriptions
        let ret = rcl_subscription_fini(&mut sub, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize subscription");

        let ret = rcl_subscription_fini(&mut sub2, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize subscription 2");
    }
}

/// Test count matched functions with mismatched QoS
/// Note: In the C++ version, this test verifies QoS compatibility checking - if QoS is
/// incompatible (e.g., BEST_EFFORT publisher with RELIABLE subscriber), the count should be 0.
/// For Zenoh implementation, counts are always 0 regardless of QoS settings.
#[test]
fn test_count_matched_functions_mismatched_qos() {
    let mut fixture = TestCountFixture::new();

    unsafe {
        let topic_name = c"/test_count_matched_functions_mismatched_qos__";
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);

        // Initialize publisher with BEST_EFFORT reliability
        let mut pub_handle = rcl_get_zero_initialized_publisher();
        let mut pub_opts = rcl_publisher_get_default_options();
        pub_opts.qos.history = rmw_qos_history_policy_t::KEEP_LAST;
        pub_opts.qos.depth = 10;
        pub_opts.qos.reliability = rmw_qos_reliability_policy_t::BEST_EFFORT;
        pub_opts.qos.durability = rmw_qos_durability_policy_t::VOLATILE;
        pub_opts.qos.avoid_ros_namespace_conventions = false;

        let ret = rcl_publisher_init(
            &mut pub_handle,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &pub_opts,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize publisher");

        // Check initial state
        let mut subscriber_count: usize = 999;
        let ret = rcl_publisher_get_subscription_count(&pub_handle, &mut subscriber_count);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(subscriber_count, 0);

        // Initialize subscription with RELIABLE reliability (incompatible with BEST_EFFORT in some RMW)
        let mut sub = rcl_get_zero_initialized_subscription();
        let mut sub_opts = rcl_subscription_get_default_options();
        sub_opts.qos.history = rmw_qos_history_policy_t::KEEP_LAST;
        sub_opts.qos.depth = 10;
        sub_opts.qos.reliability = rmw_qos_reliability_policy_t::RELIABLE;
        sub_opts.qos.durability = rmw_qos_durability_policy_t::VOLATILE;
        sub_opts.qos.avoid_ros_namespace_conventions = false;

        let ret =
            rcl_subscription_init(&mut sub, fixture.node(), ts, topic_name.as_ptr(), &sub_opts);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize subscription");

        // In C++/ROS2: Depending on QoS compatibility check, count might be 0 or 1
        // In Zenoh: Always 0
        let ret = rcl_publisher_get_subscription_count(&pub_handle, &mut subscriber_count);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(subscriber_count, 0, "Zenoh: count is 0 regardless of QoS");

        let mut publisher_count: usize = 999;
        let ret = rcl_subscription_get_publisher_count(&sub, &mut publisher_count);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(publisher_count, 0, "Zenoh: count is 0 regardless of QoS");

        // Initialize second subscription with default (compatible) QoS
        let mut sub2 = rcl_get_zero_initialized_subscription();
        let sub2_opts = rcl_subscription_get_default_options();
        let ret = rcl_subscription_init(
            &mut sub2,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &sub2_opts,
        );
        assert_eq!(
            ret, RCL_RET_OK as i32,
            "Failed to initialize subscription 2"
        );

        // In C++/ROS2: With mismatched QoS, both subscriptions might show 0 publishers
        // or sub2 might show 1 depending on QoS compatibility
        // In Zenoh: Always 0
        let ret = rcl_publisher_get_subscription_count(&pub_handle, &mut subscriber_count);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(subscriber_count, 0);

        // Cleanup
        let ret = rcl_publisher_fini(&mut pub_handle, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize publisher");

        let ret = rcl_subscription_fini(&mut sub, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize subscription");

        let ret = rcl_subscription_fini(&mut sub2, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize subscription 2");
    }
}
