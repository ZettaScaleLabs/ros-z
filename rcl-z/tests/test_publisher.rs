// Ported from Open Source Robotics Foundation code (2015)
// https://github.com/ros2/rcl
// Licensed under the Apache License 2.0
// Copyright 2025 ZettaScale Technology

#![cfg(feature = "test-msgs")]

mod test_msgs_support;

use std::ptr;

// Re-export from test_msgs_support for convenience
use rcl_z::{
    c_void,
    context::{rcl_context_fini, rcl_get_zero_initialized_context, rcl_shutdown},
    init::{
        rcl_get_default_allocator, rcl_get_zero_initialized_init_options, rcl_init,
        rcl_init_options_fini, rcl_init_options_init,
    },
    node::{
        rcl_get_zero_initialized_node, rcl_node_fini, rcl_node_get_default_options, rcl_node_init,
    },
    pubsub::{
        rcl_borrow_loaned_message, rcl_get_zero_initialized_publisher, rcl_publish,
        rcl_publish_loaned_message, rcl_publisher_assert_liveliness,
        rcl_publisher_can_loan_messages, rcl_publisher_fini, rcl_publisher_get_default_options,
        rcl_publisher_get_options, rcl_publisher_get_subscription_count,
        rcl_publisher_get_topic_name, rcl_publisher_init, rcl_publisher_is_valid,
        rcl_publisher_is_valid_except_context, rcl_publisher_wait_for_all_acked,
    },
    ros::*,
};
use test_msgs_support::{
    rosidl_runtime_c__String__assign, test_msgs__msg__BasicTypes, test_msgs__msg__Strings,
    test_msgs__msg__Strings__fini, test_msgs__msg__Strings__init,
};

/// Test fixture that provides an initialized RCL context and node
struct TestPublisherFixture {
    context: rcl_context_t,
    node: rcl_node_t,
}

impl TestPublisherFixture {
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
            let node_name = c"test_publisher_node";
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

            TestPublisherFixture { context, node }
        }
    }

    fn node(&mut self) -> *mut rcl_node_t {
        &mut self.node
    }
}

impl Drop for TestPublisherFixture {
    fn drop(&mut self) {
        let ret = rcl_node_fini(&mut self.node);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize node");

        let ret = rcl_shutdown(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to shutdown context");

        let ret = rcl_context_fini(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize context");
    }
}

/// Basic nominal test of a publisher
///
/// This test verifies that:
/// - A publisher can be initialized with valid parameters
/// - The publisher can be finalized properly
/// - Basic publish operation works (even though we can't verify reception in this unit test)
#[test]
fn test_publisher_nominal() {
    let mut fixture = TestPublisherFixture::new();

    unsafe {
        let mut publisher = rcl_get_zero_initialized_publisher();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"chatter";
        let expected_topic_name = c"/chatter";
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

        // Check topic name
        let topic_ptr = rcl_publisher_get_topic_name(&publisher);
        assert!(!topic_ptr.is_null());
        let topic_str = std::ffi::CStr::from_ptr(topic_ptr);
        let expected_str = std::ffi::CStr::from_ptr(expected_topic_name.as_ptr());
        assert_eq!(
            topic_str, expected_str,
            "Expected topic name {:?}, got {:?}",
            expected_str, topic_str
        );

        // Create and publish a message
        let mut msg = test_msgs__msg__BasicTypes { int64_value: 42 };
        let ret = rcl_publish(
            &publisher,
            &mut msg as *mut _ as *mut c_void,
            ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to publish message");

        // Finalize publisher
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize publisher");
    }
}

/// Basic nominal test of a publisher with Strings message type
///
/// This test verifies publishing with Strings message type
#[test]
fn test_publisher_nominal_string() {
    let mut fixture = TestPublisherFixture::new();

    unsafe {
        let mut publisher = rcl_get_zero_initialized_publisher();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, Strings);
        let topic_name = c"chatter";
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

        // Create and publish a Strings message
        let mut msg: test_msgs__msg__Strings = std::mem::zeroed();
        assert!(test_msgs__msg__Strings__init(&mut msg));
        let test_string = c"testing";
        assert!(rosidl_runtime_c__String__assign(
            &mut msg.string_value,
            test_string.as_ptr()
        ));
        let ret = rcl_publish(
            &publisher,
            &mut msg as *mut _ as *mut c_void,
            ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to publish message");
        test_msgs__msg__Strings__fini(&mut msg);

        // Finalize publisher
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize publisher");
    }
}

/// Test two publishers using different message types with the same basename
///
/// Regression test for issue where RMW implementations could not support
/// publishers on topics with the same basename but different namespaces
/// using different message types
#[test]
fn test_publishers_different_types() {
    let mut fixture = TestPublisherFixture::new();

    unsafe {
        // First publisher with BasicTypes on "basename"
        let mut publisher1 = rcl_get_zero_initialized_publisher();
        let ts_int = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name1 = c"basename";
        let publisher_options = rcl_publisher_get_default_options();

        let ret = rcl_publisher_init(
            &mut publisher1,
            fixture.node(),
            ts_int,
            topic_name1.as_ptr(),
            &publisher_options,
        );
        assert_eq!(
            ret, RCL_RET_OK as i32,
            "Failed to initialize first publisher"
        );

        // Check topic name
        let topic_ptr = rcl_publisher_get_topic_name(&publisher1);
        assert!(!topic_ptr.is_null());
        let topic_str = std::ffi::CStr::from_ptr(topic_ptr);
        let expected_topic1 = c"/basename";
        let expected_str1 = std::ffi::CStr::from_ptr(expected_topic1.as_ptr());
        assert_eq!(
            topic_str, expected_str1,
            "Expected topic name {:?}, got {:?}",
            expected_str1, topic_str
        );

        // Second publisher with Strings on "namespace/basename"
        let mut publisher2 = rcl_get_zero_initialized_publisher();
        let ts_string = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, Strings);
        let topic_name2 = c"namespace/basename";

        let ret = rcl_publisher_init(
            &mut publisher2,
            fixture.node(),
            ts_string,
            topic_name2.as_ptr(),
            &publisher_options,
        );
        assert_eq!(
            ret, RCL_RET_OK as i32,
            "Failed to initialize second publisher"
        );

        // Check topic name
        let topic_ptr = rcl_publisher_get_topic_name(&publisher2);
        assert!(!topic_ptr.is_null());
        let topic_str = std::ffi::CStr::from_ptr(topic_ptr);
        let expected_topic2 = c"/namespace/basename";
        let expected_str2 = std::ffi::CStr::from_ptr(expected_topic2.as_ptr());
        assert_eq!(
            topic_str, expected_str2,
            "Expected topic name {:?}, got {:?}",
            expected_str2, topic_str
        );

        // Publish messages
        let mut msg_int = test_msgs__msg__BasicTypes { int64_value: 42 };
        let ret = rcl_publish(
            &publisher1,
            &mut msg_int as *mut _ as *mut c_void,
            ptr::null_mut(),
        );
        assert_eq!(
            ret, RCL_RET_OK as i32,
            "Failed to publish BasicTypes message"
        );

        let mut msg_string: test_msgs__msg__Strings = std::mem::zeroed();
        assert!(test_msgs__msg__Strings__init(&mut msg_string));
        let test_string = c"testing";
        assert!(rosidl_runtime_c__String__assign(
            &mut msg_string.string_value,
            test_string.as_ptr()
        ));
        let ret = rcl_publish(
            &publisher2,
            &mut msg_string as *mut _ as *mut c_void,
            ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to publish Strings message");
        test_msgs__msg__Strings__fini(&mut msg_string);

        // Finalize publishers
        let ret = rcl_publisher_fini(&mut publisher1, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize first publisher");

        let ret = rcl_publisher_fini(&mut publisher2, fixture.node());
        assert_eq!(
            ret, RCL_RET_OK as i32,
            "Failed to finalize second publisher"
        );
    }
}

/// Test loaned message functionality
///
/// Tests the loan message functionality which may or may not be supported
/// depending on the RMW implementation
#[test]
fn test_publisher_loan() {
    let mut fixture = TestPublisherFixture::new();

    unsafe {
        let mut publisher = rcl_get_zero_initialized_publisher();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, Strings);
        let topic_name = c"chatter";
        let publisher_options = rcl_publisher_get_default_options();

        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize publisher");

        if rcl_publisher_can_loan_messages(&publisher) {
            let mut loaned_msg: *mut c_void = ptr::null_mut();
            let ret = rcl_borrow_loaned_message(&publisher, ts, &mut loaned_msg);
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to borrow loaned message");

            if !loaned_msg.is_null() {
                // Cast to Strings message and assign value
                let msg = loaned_msg as *mut test_msgs__msg__Strings;
                let test_string = c"testing";
                assert!(rosidl_runtime_c__String__assign(
                    &mut (*msg).string_value,
                    test_string.as_ptr()
                ));

                let ret = rcl_publish_loaned_message(&publisher, loaned_msg, ptr::null_mut());
                assert_eq!(ret, RCL_RET_OK as i32, "Failed to publish loaned message");
            }
        }

        // Finalize publisher
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize publisher");
    }
}

/// Test publisher options with environment variables
///
/// Tests the ROS_DISABLE_LOANED_MESSAGES environment variable
#[test]
fn test_publisher_option() {
    unsafe {
        // Test default state (no env var set)
        let publisher_options = rcl_publisher_get_default_options();
        assert!(!publisher_options.disable_loaned_message);

        // Test with ROS_DISABLE_LOANED_MESSAGES=0
        std::env::set_var("ROS_DISABLE_LOANED_MESSAGES", "0");
        let publisher_options = rcl_publisher_get_default_options();
        assert!(!publisher_options.disable_loaned_message);

        // Test with ROS_DISABLE_LOANED_MESSAGES=1
        std::env::set_var("ROS_DISABLE_LOANED_MESSAGES", "1");
        let publisher_options = rcl_publisher_get_default_options();
        assert!(publisher_options.disable_loaned_message);

        // Test with ROS_DISABLE_LOANED_MESSAGES=2 (invalid)
        std::env::set_var("ROS_DISABLE_LOANED_MESSAGES", "2");
        let publisher_options = rcl_publisher_get_default_options();
        assert!(!publisher_options.disable_loaned_message);

        // Test with ROS_DISABLE_LOANED_MESSAGES=Unexpected (invalid)
        std::env::set_var("ROS_DISABLE_LOANED_MESSAGES", "Unexpected");
        let publisher_options = rcl_publisher_get_default_options();
        assert!(!publisher_options.disable_loaned_message);
    }
}

/// Test disabling loaned messages via environment variable
///
/// Tests that setting ROS_DISABLE_LOANED_MESSAGES=1 prevents loaning messages
/// and ROS_DISABLE_LOANED_MESSAGES=0 allows loaning (if supported by RMW)
#[test]
fn test_publisher_loan_disable() {
    let mut fixture = TestPublisherFixture::new();

    unsafe {
        // Save original value of ROS_DISABLE_LOANED_MESSAGES
        let original_env = std::env::var("ROS_DISABLE_LOANED_MESSAGES").ok();

        // Test with ROS_DISABLE_LOANED_MESSAGES=1
        std::env::set_var("ROS_DISABLE_LOANED_MESSAGES", "1");

        let mut publisher = rcl_get_zero_initialized_publisher();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"pod_msg";
        let publisher_options = rcl_publisher_get_default_options();
        assert!(publisher_options.disable_loaned_message);

        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize publisher");

        // Should not be able to loan messages
        assert!(!rcl_publisher_can_loan_messages(&publisher));

        // Finalize publisher
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize publisher");

        // Test with ROS_DISABLE_LOANED_MESSAGES=0
        std::env::set_var("ROS_DISABLE_LOANED_MESSAGES", "0");

        let mut publisher2 = rcl_get_zero_initialized_publisher();
        let publisher_options2 = rcl_publisher_get_default_options();
        assert!(!publisher_options2.disable_loaned_message);

        let ret = rcl_publisher_init(
            &mut publisher2,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options2,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize publisher");

        // Whether can loan messages depends on RMW implementation
        // We just check the call doesn't crash
        let _can_loan = rcl_publisher_can_loan_messages(&publisher2);

        // Finalize publisher
        let ret = rcl_publisher_fini(&mut publisher2, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize publisher");

        // Restore original environment variable
        if let Some(val) = original_env {
            std::env::set_var("ROS_DISABLE_LOANED_MESSAGES", val);
        } else {
            std::env::remove_var("ROS_DISABLE_LOANED_MESSAGES");
        }
    }
}

/// Test publisher initialization and finalization with various invalid inputs
///
/// This test covers:
/// - Null publisher pointer
/// - Null node pointer
/// - Null type support
/// - Null topic name
/// - Null options
/// - Already initialized publisher
#[test]
fn test_publisher_init_fini() {
    let mut fixture = TestPublisherFixture::new();

    unsafe {
        let mut publisher: rcl_publisher_t;
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"chatter";
        let default_publisher_options = rcl_publisher_get_default_options();

        // Check if null publisher is valid - should return false
        assert!(!rcl_publisher_is_valid(ptr::null()));

        // Check if zero initialized publisher is valid - should return false
        publisher = rcl_get_zero_initialized_publisher();
        assert!(!rcl_publisher_is_valid(&publisher));

        // Initialize a valid publisher and check it's valid
        publisher = rcl_get_zero_initialized_publisher();
        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &default_publisher_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(rcl_publisher_is_valid(&publisher));

        // Try to init an already initialized publisher - should fail
        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &default_publisher_options,
        );
        assert_eq!(ret, RCL_RET_ALREADY_INIT as i32);

        // Finalize the publisher
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Pass invalid node to fini
        let ret = rcl_publisher_fini(&mut publisher, ptr::null_mut());
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Pass nullptr publisher to fini
        let ret = rcl_publisher_fini(ptr::null_mut(), fixture.node());
        assert_eq!(ret, RCL_RET_PUBLISHER_INVALID as i32);

        // Try passing null for publisher in init
        let ret = rcl_publisher_init(
            ptr::null_mut(),
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &default_publisher_options,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Try passing null for a node pointer in init
        publisher = rcl_get_zero_initialized_publisher();
        let ret = rcl_publisher_init(
            &mut publisher,
            ptr::null_mut(),
            ts,
            topic_name.as_ptr(),
            &default_publisher_options,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Try passing an invalid (uninitialized) node in init
        publisher = rcl_get_zero_initialized_publisher();
        let invalid_node = rcl_get_zero_initialized_node();
        let ret = rcl_publisher_init(
            &mut publisher,
            &invalid_node,
            ts,
            topic_name.as_ptr(),
            &default_publisher_options,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Try passing null for the type support in init
        publisher = rcl_get_zero_initialized_publisher();
        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ptr::null(),
            topic_name.as_ptr(),
            &default_publisher_options,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        let _ = rcl_publisher_fini(&mut publisher, fixture.node());

        // Try passing null for the topic name in init
        publisher = rcl_get_zero_initialized_publisher();
        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            ptr::null(),
            &default_publisher_options,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        let _ = rcl_publisher_fini(&mut publisher, fixture.node());

        // Try passing null for the options in init
        publisher = rcl_get_zero_initialized_publisher();
        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            ptr::null(),
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        let _ = rcl_publisher_fini(&mut publisher, fixture.node());

        // Try passing options with an invalid allocate in allocator with init
        publisher = rcl_get_zero_initialized_publisher();
        let mut publisher_options_with_invalid_allocator = rcl_publisher_get_default_options();
        publisher_options_with_invalid_allocator.allocator.allocate = None;
        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options_with_invalid_allocator,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        let _ = rcl_publisher_fini(&mut publisher, fixture.node());

        // Try passing options with an invalid deallocate in allocator with init
        publisher = rcl_get_zero_initialized_publisher();
        publisher_options_with_invalid_allocator = rcl_publisher_get_default_options();
        publisher_options_with_invalid_allocator
            .allocator
            .deallocate = None;
        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options_with_invalid_allocator,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        let _ = rcl_publisher_fini(&mut publisher, fixture.node());
    }
}

/// Test invalid publisher handle behavior
///
/// This test verifies that various publisher operations correctly handle
/// null and invalid publisher pointers by returning appropriate error codes
#[test]
fn test_invalid_publisher() {
    let mut fixture = TestPublisherFixture::new();

    unsafe {
        let mut publisher = rcl_get_zero_initialized_publisher();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"chatter";
        let publisher_options = rcl_publisher_get_default_options();

        // Initialize a valid publisher
        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Test that valid publisher returns non-null options
        let publisher_options_rcv = rcl_publisher_get_options(&publisher);
        assert!(!publisher_options_rcv.is_null());

        // Test that valid publisher returns non-null topic name
        let topic_ptr = rcl_publisher_get_topic_name(&publisher);
        assert!(!topic_ptr.is_null());

        // Test null publisher with various operations
        assert!(!rcl_publisher_is_valid_except_context(ptr::null()));
        assert!(!rcl_publisher_is_valid(ptr::null()));
        assert!(rcl_publisher_get_topic_name(ptr::null()).is_null());
        assert!(rcl_publisher_get_options(ptr::null()).is_null());
        assert!(!rcl_publisher_can_loan_messages(ptr::null()));

        let ret = rcl_publisher_assert_liveliness(ptr::null());
        assert_eq!(ret, RCL_RET_PUBLISHER_INVALID as i32);

        let ret = rcl_publisher_wait_for_all_acked(ptr::null(), 10000000);
        assert_eq!(ret, RCL_RET_PUBLISHER_INVALID as i32);

        // Test publish with null publisher
        let dummy_msg: *const c_void = ptr::null();
        let ret = rcl_publish(ptr::null(), dummy_msg, ptr::null_mut());
        assert_eq!(ret, RCL_RET_PUBLISHER_INVALID as i32);

        // Test publish with null message
        let ret = rcl_publish(&publisher, ptr::null(), ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Finalize the publisher
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test that publisher subscription count API handles invalid publishers
#[test]
fn test_publisher_get_subscription_count() {
    let mut fixture = TestPublisherFixture::new();

    unsafe {
        let mut publisher = rcl_get_zero_initialized_publisher();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"chatter";
        let publisher_options = rcl_publisher_get_default_options();

        // Initialize publisher
        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Test with valid publisher (count might be 0 since no subscribers)
        let mut count: usize = 999;
        let _ret = rcl_publisher_get_subscription_count(&publisher, &mut count);
        // The return value depends on implementation - might be OK or ERROR
        // We mainly test that it doesn't crash

        // Test with null publisher
        let ret = rcl_publisher_get_subscription_count(ptr::null(), &mut count);
        assert_eq!(ret, RCL_RET_PUBLISHER_INVALID as i32);

        // Test with null count pointer
        let ret = rcl_publisher_get_subscription_count(&publisher, ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Finalize
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test publisher with loaned messages API
///
/// Tests the loan message functionality which may or may not be supported
/// depending on the RMW implementation
#[test]
fn test_publisher_can_loan_messages() {
    let mut fixture = TestPublisherFixture::new();

    unsafe {
        let mut publisher = rcl_get_zero_initialized_publisher();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"chatter";
        let publisher_options = rcl_publisher_get_default_options();

        // Initialize publisher
        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check if publisher can loan messages (implementation dependent)
        let _can_loan = rcl_publisher_can_loan_messages(&publisher);
        // We just verify the call doesn't crash - result is implementation dependent

        // Test with null publisher
        let can_loan_null = rcl_publisher_can_loan_messages(ptr::null());
        assert!(!can_loan_null);

        // Finalize
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test assert liveliness functionality
#[test]
fn test_publisher_assert_liveliness() {
    let mut fixture = TestPublisherFixture::new();

    unsafe {
        let mut publisher = rcl_get_zero_initialized_publisher();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"chatter";
        let publisher_options = rcl_publisher_get_default_options();

        // Initialize publisher
        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Assert liveliness with valid publisher
        let ret = rcl_publisher_assert_liveliness(&publisher);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Test with null publisher
        let ret = rcl_publisher_assert_liveliness(ptr::null());
        assert_eq!(ret, RCL_RET_PUBLISHER_INVALID as i32);

        // Finalize
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test wait for all acked functionality
#[test]
fn test_publisher_wait_for_all_acked() {
    let mut fixture = TestPublisherFixture::new();

    unsafe {
        let mut publisher = rcl_get_zero_initialized_publisher();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"chatter";
        let publisher_options = rcl_publisher_get_default_options();

        // Initialize publisher
        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Wait for all acked with valid publisher (timeout = 0 means return immediately)
        let _ret = rcl_publisher_wait_for_all_acked(&publisher, 0);
        // Return value is implementation dependent - might be OK, TIMEOUT, or UNSUPPORTED

        // Test with null publisher
        let ret = rcl_publisher_wait_for_all_acked(ptr::null(), 10000000);
        assert_eq!(ret, RCL_RET_PUBLISHER_INVALID as i32);

        // Finalize
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test publisher is_valid and is_valid_except_context
#[test]
fn test_publisher_is_valid() {
    let mut fixture = TestPublisherFixture::new();

    unsafe {
        // Null publisher should not be valid
        assert!(!rcl_publisher_is_valid(ptr::null()));
        assert!(!rcl_publisher_is_valid_except_context(ptr::null()));

        // Zero-initialized publisher should not be valid
        let mut publisher = rcl_get_zero_initialized_publisher();
        assert!(!rcl_publisher_is_valid(&publisher));
        assert!(!rcl_publisher_is_valid_except_context(&publisher));

        // Properly initialized publisher should be valid
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"chatter";
        let publisher_options = rcl_publisher_get_default_options();

        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(rcl_publisher_is_valid(&publisher));
        assert!(rcl_publisher_is_valid_except_context(&publisher));

        // Finalized publisher should not be valid
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test getting publisher options
#[test]
fn test_publisher_get_options() {
    let mut fixture = TestPublisherFixture::new();

    unsafe {
        // Null publisher should return null options
        assert!(rcl_publisher_get_options(ptr::null()).is_null());

        // Valid publisher should return non-null options
        let mut publisher = rcl_get_zero_initialized_publisher();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"chatter";
        let publisher_options = rcl_publisher_get_default_options();

        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        let options_ptr = rcl_publisher_get_options(&publisher);
        assert!(!options_ptr.is_null());

        // Finalize
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test getting publisher topic name
#[test]
fn test_publisher_get_topic_name() {
    let mut fixture = TestPublisherFixture::new();

    unsafe {
        // Null publisher should return null topic name
        assert!(rcl_publisher_get_topic_name(ptr::null()).is_null());

        // Valid publisher should return non-null topic name
        let mut publisher = rcl_get_zero_initialized_publisher();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"chatter";
        let publisher_options = rcl_publisher_get_default_options();

        let ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            topic_name.as_ptr(),
            &publisher_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        let topic_ptr = rcl_publisher_get_topic_name(&publisher);
        assert!(!topic_ptr.is_null());

        // The topic name should match what we set (possibly with namespace prefix)
        let topic_str = std::ffi::CStr::from_ptr(topic_ptr)
            .to_string_lossy()
            .to_string();
        // Topic might be "/chatter" or "chatter" depending on implementation
        assert!(
            topic_str.contains("chatter"),
            "Expected topic to contain 'chatter', got: {}",
            topic_str
        );

        // Finalize
        let ret = rcl_publisher_fini(&mut publisher, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}
