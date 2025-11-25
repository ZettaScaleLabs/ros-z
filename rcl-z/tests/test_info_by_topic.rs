// Ported from Open Source Robotics Foundation code (2019)
// https://github.com/ros2/rcl
// Licensed under the Apache License 2.0
// Copyright 2025 ZettaScale Technology

#![cfg(feature = "test-msgs")]

mod test_msgs_support;

use std::ptr;

use rcl_z::{
    context::{rcl_context_fini, rcl_context_is_valid, rcl_get_zero_initialized_context, rcl_shutdown},
    graph::{
        rcl_get_publishers_info_by_topic, rcl_get_subscriptions_info_by_topic,
    },
    init::{
        rcl_get_default_allocator, rcl_get_zero_initialized_init_options, rcl_init,
        rcl_init_options_fini, rcl_init_options_init,
    },
    node::{rcl_get_zero_initialized_node, rcl_node_fini, rcl_node_get_default_options, rcl_node_init},
    ros::*,
};


/// Test fixture for topic info by topic tests
struct TestInfoByTopicFixture {
    old_context: rcl_context_t,
    context: rcl_context_t,
    old_node: rcl_node_t,
    node: rcl_node_t,
    init_options: rcl_init_options_t,
    #[allow(dead_code)]
    test_graph_node_name: &'static str,
    topic_name: &'static str,
}

impl TestInfoByTopicFixture {
    fn new() -> Self {
        let mut init_options = rcl_get_zero_initialized_init_options();
        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut old_context = rcl_get_zero_initialized_context();
        let ret = rcl_init(0, ptr::null(), &init_options, &mut old_context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut old_node = rcl_get_zero_initialized_node();
        let old_name = c"old_node_name";
        let node_options = rcl_node_get_default_options();
            let ret = unsafe { rcl_node_init(
                &mut old_node,
                old_name.as_ptr(),
                c"".as_ptr(),
                &mut old_context,
                &node_options,
            ) };
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_shutdown(&mut old_context);
        assert_eq!(ret, RCL_RET_OK as i32); // after this, the old_node should be invalid

        let mut context = rcl_get_zero_initialized_context();
        let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut node = rcl_get_zero_initialized_node();
        let name = c"test_graph_node";
            let ret = unsafe { rcl_node_init(
                &mut node,
                name.as_ptr(),
                c"".as_ptr(),
                &mut context,
                &node_options,
            ) };
        assert_eq!(ret, RCL_RET_OK as i32);

        TestInfoByTopicFixture {
            old_context,
            context,
            old_node,
            node,
            init_options,
            test_graph_node_name: "test_graph_node",
            topic_name: "valid_topic_name",
        }
    }

    fn node(&mut self) -> *mut rcl_node_t {
        &mut self.node
    }

    fn old_node(&mut self) -> *mut rcl_node_t {
        &mut self.old_node
    }
}

impl Drop for TestInfoByTopicFixture {
    fn drop(&mut self) {
        let ret = rcl_node_fini(&mut self.old_node);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_node_fini(&mut self.node);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_shutdown(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_context_fini(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32);

        // old_context was supposed to have been shutdown already during SetUp()
        if unsafe { rcl_context_is_valid(&self.old_context) } {
            let ret = rcl_shutdown(&mut self.old_context);
            assert_eq!(ret, RCL_RET_OK as i32);
        }
        let ret = rcl_context_fini(&mut self.old_context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_init_options_fini(&mut self.init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

#[test]
fn test_rcl_get_publishers_info_by_topic_null_node() {
    let fixture = TestInfoByTopicFixture::new();
    let mut allocator = rcl_get_default_allocator();
    let ret = rcl_get_publishers_info_by_topic(
        ptr::null_mut(),
        &mut allocator,
        fixture.topic_name.as_ptr() as *const _,
        false,
        ptr::null_mut(),
    );
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
}

#[test]
fn test_rcl_get_subscriptions_info_by_topic_null_node() {
    let fixture = TestInfoByTopicFixture::new();
    let mut allocator = rcl_get_default_allocator();
    let ret = rcl_get_subscriptions_info_by_topic(
        ptr::null_mut(),
        &mut allocator,
        fixture.topic_name.as_ptr() as *const _,
        false,
        ptr::null_mut(),
    );
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
}

#[test]
fn test_rcl_get_publishers_info_by_topic_invalid_node() {
    let mut fixture = TestInfoByTopicFixture::new();
    // this->old_node is an invalid node.
    let mut allocator = rcl_get_default_allocator();
    let mut topic_endpoint_info_array = rmw_topic_endpoint_info_array_t::default();
    let ret = rcl_get_publishers_info_by_topic(
        fixture.old_node(),
        &mut allocator,
        fixture.topic_name.as_ptr() as *const _,
        false,
        &mut topic_endpoint_info_array,
    );
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
}

#[test]
fn test_rcl_get_subscriptions_info_by_topic_invalid_node() {
    let mut fixture = TestInfoByTopicFixture::new();
    // this->old_node is an invalid node.
    let mut allocator = rcl_get_default_allocator();
    let mut topic_endpoint_info_array = rmw_topic_endpoint_info_array_t::default();
    let ret = rcl_get_subscriptions_info_by_topic(
        fixture.old_node(),
        &mut allocator,
        fixture.topic_name.as_ptr() as *const _,
        false,
        &mut topic_endpoint_info_array,
    );
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
}

#[test]
fn test_rcl_get_publishers_info_by_topic_null_allocator() {
    let mut fixture = TestInfoByTopicFixture::new();
    let mut topic_endpoint_info_array = rmw_topic_endpoint_info_array_t::default();
    let ret = rcl_get_publishers_info_by_topic(
        fixture.node(),
        ptr::null_mut(),
        fixture.topic_name.as_ptr() as *const _,
        false,
        &mut topic_endpoint_info_array,
    );
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
}

#[test]
fn test_rcl_get_subscriptions_info_by_topic_null_allocator() {
    let mut fixture = TestInfoByTopicFixture::new();
    let mut topic_endpoint_info_array = rmw_topic_endpoint_info_array_t::default();
    let ret = rcl_get_subscriptions_info_by_topic(
        fixture.node(),
        ptr::null_mut(),
        fixture.topic_name.as_ptr() as *const _,
        false,
        &mut topic_endpoint_info_array,
    );
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
}

#[test]
fn test_rcl_get_publishers_info_by_topic_null_topic() {
    let mut fixture = TestInfoByTopicFixture::new();
    let mut allocator = rcl_get_default_allocator();
    let mut topic_endpoint_info_array = rmw_topic_endpoint_info_array_t::default();
    let ret = rcl_get_publishers_info_by_topic(
        fixture.node(),
        &mut allocator,
        ptr::null_mut(),
        false,
        &mut topic_endpoint_info_array,
    );
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
}

#[test]
fn test_rcl_get_subscriptions_info_by_topic_null_topic() {
    let mut fixture = TestInfoByTopicFixture::new();
    let mut allocator = rcl_get_default_allocator();
    let mut topic_endpoint_info_array = rmw_topic_endpoint_info_array_t::default();
    let ret = rcl_get_subscriptions_info_by_topic(
        fixture.node(),
        &mut allocator,
        ptr::null_mut(),
        false,
        &mut topic_endpoint_info_array,
    );
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
}

#[test]
fn test_rcl_get_publishers_info_by_topic_null_participants() {
    let mut fixture = TestInfoByTopicFixture::new();
    let mut allocator = rcl_get_default_allocator();
    let ret = rcl_get_publishers_info_by_topic(
        fixture.node(),
        &mut allocator,
        fixture.topic_name.as_ptr() as *const _,
        false,
        ptr::null_mut(),
    );
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
}

#[test]
fn test_rcl_get_subscriptions_info_by_topic_null_participants() {
    let mut fixture = TestInfoByTopicFixture::new();
    let mut allocator = rcl_get_default_allocator();
    let ret = rcl_get_subscriptions_info_by_topic(
        fixture.node(),
        &mut allocator,
        fixture.topic_name.as_ptr() as *const _,
        false,
        ptr::null_mut(),
    );
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
}

#[test]
fn test_rcl_get_publishers_info_by_topic_invalid_participants() {
    let mut fixture = TestInfoByTopicFixture::new();
    // topic_endpoint_info_array is invalid because it is expected to be zero initialized
    // and the info_array variable inside it is expected to be null.
    let invalid_ptr = Box::into_raw(Box::new(rmw_topic_endpoint_info_t::default())) as *mut _;
    let mut topic_endpoint_info_array = rmw_topic_endpoint_info_array_t {
        info_array: invalid_ptr,
        ..Default::default()
    };
    let mut allocator = rcl_get_default_allocator();
    let ret = rcl_get_publishers_info_by_topic(
        fixture.node(),
        &mut allocator,
        fixture.topic_name.as_ptr() as *const _,
        false,
        &mut topic_endpoint_info_array,
    );
    assert_eq!(ret, RCL_RET_ERROR as i32);
    // Clean up
    unsafe { let _ = Box::from_raw(invalid_ptr); }
}

#[test]
fn test_rcl_get_subscriptions_info_by_topic_invalid_participants() {
    let mut fixture = TestInfoByTopicFixture::new();
    // topic_endpoint_info_array is invalid because it is expected to be zero initialized
    // and the info_array variable inside it is expected to be null.
    let invalid_ptr = Box::into_raw(Box::new(rmw_topic_endpoint_info_t::default())) as *mut _;
    let mut topic_endpoint_info_array = rmw_topic_endpoint_info_array_t {
        info_array: invalid_ptr,
        ..Default::default()
    };
    let mut allocator = rcl_get_default_allocator();
    let ret = rcl_get_subscriptions_info_by_topic(
        fixture.node(),
        &mut allocator,
        fixture.topic_name.as_ptr() as *const _,
        false,
        &mut topic_endpoint_info_array,
    );
    assert_eq!(ret, RCL_RET_ERROR as i32);
    // Clean up
    unsafe { let _ = Box::from_raw(invalid_ptr); }
}

// TODO: Implement the main test when message types are properly supported
