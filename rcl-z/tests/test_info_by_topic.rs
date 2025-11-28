// Copyright 2025 ZettaScale Technology
// SPDX-License-Identifier: Apache-2.0
//
// Ported from ros2/rcl:
// Copyright 2019 Open Source Robotics Foundation, Inc.

#![cfg(feature = "test-msgs")]

mod test_msgs_support;

use std::ptr;

use rcl_z::{
    context::{
        rcl_context_fini, rcl_context_is_valid, rcl_get_zero_initialized_context, rcl_shutdown,
    },
    graph::{
        rcl_get_publishers_info_by_topic, rcl_get_subscriptions_info_by_topic,
        rcl_wait_for_publishers, rcl_wait_for_subscribers,
    },
    init::{
        rcl_get_default_allocator, rcl_get_zero_initialized_init_options, rcl_init,
        rcl_init_options_fini, rcl_init_options_init,
    },
    node::{
        rcl_get_zero_initialized_node, rcl_node_fini, rcl_node_get_default_options, rcl_node_init,
    },
    pubsub::{
        rcl_get_zero_initialized_publisher, rcl_get_zero_initialized_subscription,
        rcl_publisher_fini, rcl_publisher_get_default_options, rcl_publisher_init,
        rcl_subscription_fini, rcl_subscription_get_default_options, rcl_subscription_init,
    },
    ros::*,
};

/// Test fixture for topic info by topic tests
struct TestInfoByTopicFixture {
    old_context: Box<rcl_context_t>,
    context: Box<rcl_context_t>,
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

        // Use Box to allocate contexts on the heap with stable addresses
        let mut old_context = Box::new(rcl_get_zero_initialized_context());
        let ret = rcl_init(0, ptr::null(), &init_options, &mut *old_context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut old_node = rcl_get_zero_initialized_node();
        let old_name = c"old_node_name";
        let node_options = rcl_node_get_default_options();
        let ret = unsafe {
            rcl_node_init(
                &mut old_node,
                old_name.as_ptr(),
                c"".as_ptr(),
                &mut *old_context,
                &node_options,
            )
        };
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_shutdown(&mut *old_context);
        assert_eq!(ret, RCL_RET_OK as i32); // after this, the old_node should be invalid

        let mut context = Box::new(rcl_get_zero_initialized_context());
        let ret = rcl_init(0, ptr::null(), &init_options, &mut *context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut node = rcl_get_zero_initialized_node();
        let name = c"test_graph_node";
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                name.as_ptr(),
                c"".as_ptr(),
                &mut *context,
                &node_options,
            )
        };
        assert_eq!(ret, RCL_RET_OK as i32);

        TestInfoByTopicFixture {
            old_context,
            context,
            old_node,
            node,
            init_options,
            test_graph_node_name: "test_graph_node",
            topic_name: "/unique_topic_for_info_by_topic_test",
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

        let ret = rcl_shutdown(&mut *self.context);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_context_fini(&mut *self.context);
        assert_eq!(ret, RCL_RET_OK as i32);

        // old_context was supposed to have been shutdown already during SetUp()
        if unsafe { rcl_context_is_valid(&*self.old_context) } {
            let ret = rcl_shutdown(&mut *self.old_context);
            assert_eq!(ret, RCL_RET_OK as i32);
        }
        let ret = rcl_context_fini(&mut *self.old_context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_init_options_fini(&mut self.init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/*
 * This does not test content of the response.
 * It only tests if the return code is the one expected.
 */
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

/*
 * This does not test content of the response.
 * It only tests if the return code is the one expected.
 */
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

/*
 * This does not test content of the response.
 * It only tests if the return code is the one expected.
 */
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

/*
 * This does not test content of the response.
 * It only tests if the return code is the one expected.
 */
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

/*
 * This does not test content of the response.
 * It only tests if the return code is the one expected.
 */
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

/*
 * This does not test content of the response.
 * It only tests if the return code is the one expected.
 */
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

/*
 * This does not test content of the response.
 * It only tests if the return code is the one expected.
 */
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

/*
 * This does not test content of the response.
 * It only tests if the return code is the one expected.
 */
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

/*
 * This does not test content of the response.
 * It only tests if the return code is the one expected.
 */
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

/*
 * This does not test content of the response.
 * It only tests if the return code is the one expected.
 */
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

/*
 * This does not test content of the response.
 * It only tests if the return code is the one expected.
 */
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
    unsafe {
        let _ = Box::from_raw(invalid_ptr);
    }
}

/*
 * This does not test content of the response.
 * It only tests if the return code is the one expected.
 */
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
    unsafe {
        let _ = Box::from_raw(invalid_ptr);
    }
}

/// Test main functionality of rcl_get_publishers_info_by_topic
#[test]
fn test_rcl_get_publishers_info_by_topic() {
    let mut fixture = TestInfoByTopicFixture::new();

    let mut topic_endpoint_info_array = rmw_topic_endpoint_info_array_t::default();
    let mut allocator = rcl_get_default_allocator();

    // Valid call (should succeed even with no publishers)
    let ret = rcl_get_publishers_info_by_topic(
        fixture.node(),
        &mut allocator,
        fixture.topic_name.as_ptr() as *const _,
        false,
        &mut topic_endpoint_info_array,
    );
    assert_eq!(ret, RCL_RET_OK as i32);

    // Should have zero publishers initially
    assert_eq!(topic_endpoint_info_array.size, 0);
}

/// Test main functionality of rcl_get_subscriptions_info_by_topic
#[test]
fn test_rcl_get_subscriptions_info_by_topic() {
    let mut fixture = TestInfoByTopicFixture::new();

    let mut topic_endpoint_info_array = rmw_topic_endpoint_info_array_t::default();
    let mut allocator = rcl_get_default_allocator();

    // Valid call (should succeed even with no subscriptions)
    let ret = rcl_get_subscriptions_info_by_topic(
        fixture.node(),
        &mut allocator,
        fixture.topic_name.as_ptr() as *const _,
        false,
        &mut topic_endpoint_info_array,
    );
    assert_eq!(ret, RCL_RET_OK as i32);

    // Should have zero subscriptions initially
    assert_eq!(topic_endpoint_info_array.size, 0);
}

impl TestInfoByTopicFixture {
    fn assert_qos_equality(
        &self,
        qos_profile1: rmw_qos_profile_t,
        qos_profile2: rmw_qos_profile_t,
        is_publisher: bool,
    ) {
        assert_eq!(qos_profile1.deadline.sec, qos_profile2.deadline.sec);
        assert_eq!(qos_profile1.deadline.nsec, qos_profile2.deadline.nsec);
        if is_publisher {
            assert_eq!(qos_profile1.lifespan.sec, qos_profile2.lifespan.sec);
            assert_eq!(qos_profile1.lifespan.nsec, qos_profile2.lifespan.nsec);
        }
        assert_eq!(qos_profile1.reliability, qos_profile2.reliability);
        assert_eq!(qos_profile1.liveliness, qos_profile2.liveliness);
        assert_eq!(
            qos_profile1.liveliness_lease_duration.sec,
            qos_profile2.liveliness_lease_duration.sec
        );
        assert_eq!(
            qos_profile1.liveliness_lease_duration.nsec,
            qos_profile2.liveliness_lease_duration.nsec
        );
        assert_eq!(qos_profile1.durability, qos_profile2.durability);
        assert_eq!(qos_profile1.history, qos_profile2.history);
        assert_eq!(qos_profile1.depth, qos_profile2.depth);
    }
}

#[test]
fn test_rcl_get_publishers_subscription_info_by_topic() {
    let mut fixture = TestInfoByTopicFixture::new();

    let default_qos_profile = rmw_qos_profile_t {
        history: rmw_qos_history_policy_e::KEEP_LAST,
        depth: 9,
        reliability: rmw_qos_reliability_policy_e::BEST_EFFORT,
        durability: rmw_qos_durability_policy_e::VOLATILE,
        lifespan: rmw_time_s { sec: 10, nsec: 0 },
        deadline: rmw_time_s { sec: 11, nsec: 0 },
        liveliness_lease_duration: rmw_time_s { sec: 20, nsec: 0 },
        liveliness: rmw_qos_liveliness_policy_e::MANUAL_BY_TOPIC,
        ..Default::default()
    };

    let mut ret;
    let ts = unsafe { ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, Strings) };
    let mut allocator = rcl_get_default_allocator();

    let mut publisher = rcl_get_zero_initialized_publisher();
    let mut publisher_options = rcl_publisher_get_default_options();
    publisher_options.qos = default_qos_profile;
    unsafe {
        ret = rcl_publisher_init(
            &mut publisher,
            fixture.node(),
            ts,
            fixture.topic_name.as_ptr() as *const _,
            &publisher_options,
        );
    }
    assert_eq!(ret, RCL_RET_OK as i32);

    let mut subscription = rcl_get_zero_initialized_subscription();
    let mut subscription_options = rcl_subscription_get_default_options();
    subscription_options.qos = default_qos_profile;
    unsafe {
        ret = rcl_subscription_init(
            &mut subscription,
            fixture.node(),
            ts,
            fixture.topic_name.as_ptr() as *const _,
            &subscription_options,
        );
    }
    assert_eq!(ret, RCL_RET_OK as i32);

    let fqdn = fixture.topic_name;
    // Wait until GraphCache publishers are updated
    let mut success = false;
    let ret = unsafe {
        rcl_wait_for_publishers(
            fixture.node(),
            &mut allocator,
            fqdn.as_ptr() as *const _,
            1,
            1000000000, // 1 second in nanoseconds
            &mut success,
        )
    };
    assert_eq!(ret, RCL_RET_OK as i32);
    assert!(success);

    // Get publishers info by topic
    let mut topic_endpoint_info_array_pub = rmw_topic_endpoint_info_array_t::default();
    let ret = rcl_get_publishers_info_by_topic(
        fixture.node(),
        &mut allocator,
        fqdn.as_ptr() as *const _,
        false,
        &mut topic_endpoint_info_array_pub,
    );
    assert_eq!(ret, RCL_RET_OK as i32);
    assert_eq!(topic_endpoint_info_array_pub.size, 1);

    let topic_endpoint_info_pub = unsafe { &*topic_endpoint_info_array_pub.info_array };
    let node_name = unsafe {
        std::ffi::CStr::from_ptr(topic_endpoint_info_pub.node_name)
            .to_str()
            .unwrap()
    };
    assert_eq!(node_name, fixture.test_graph_node_name);
    let node_namespace = unsafe {
        std::ffi::CStr::from_ptr(topic_endpoint_info_pub.node_namespace)
            .to_str()
            .unwrap()
    };
    assert_eq!(node_namespace, "/");
    let topic_type = unsafe {
        std::ffi::CStr::from_ptr(topic_endpoint_info_pub.topic_type)
            .to_str()
            .unwrap()
    };
    assert_eq!(topic_type, "test_msgs/msg/Strings");
    fixture.assert_qos_equality(
        topic_endpoint_info_pub.qos_profile,
        default_qos_profile,
        true,
    );

    // Wait until GraphCache subscribers are updated
    let mut success = false;
    let ret = unsafe {
        rcl_wait_for_subscribers(
            fixture.node(),
            &mut allocator,
            fqdn.as_ptr() as *const _,
            1,
            1000000000, // 1 second in nanoseconds
            &mut success,
        )
    };
    assert_eq!(ret, RCL_RET_OK as i32);
    assert!(success);

    // Get subscribers info by topic
    let mut topic_endpoint_info_array_sub = rmw_topic_endpoint_info_array_t::default();
    let ret = rcl_get_subscriptions_info_by_topic(
        fixture.node(),
        &mut allocator,
        fqdn.as_ptr() as *const _,
        false,
        &mut topic_endpoint_info_array_sub,
    );
    assert_eq!(ret, RCL_RET_OK as i32);
    assert_eq!(topic_endpoint_info_array_sub.size, 1);

    let topic_endpoint_info_sub = unsafe { &*topic_endpoint_info_array_sub.info_array };
    let node_name = unsafe {
        std::ffi::CStr::from_ptr(topic_endpoint_info_sub.node_name)
            .to_str()
            .unwrap()
    };
    assert_eq!(node_name, fixture.test_graph_node_name);
    let node_namespace = unsafe {
        std::ffi::CStr::from_ptr(topic_endpoint_info_sub.node_namespace)
            .to_str()
            .unwrap()
    };
    assert_eq!(node_namespace, "/");
    let topic_type = unsafe {
        std::ffi::CStr::from_ptr(topic_endpoint_info_sub.topic_type)
            .to_str()
            .unwrap()
    };
    assert_eq!(topic_type, "test_msgs/msg/Strings");
    fixture.assert_qos_equality(
        topic_endpoint_info_sub.qos_profile,
        default_qos_profile,
        false,
    );

    // Clean up
    let ret = rcl_subscription_fini(&mut subscription, fixture.node());
    assert_eq!(ret, RCL_RET_OK as i32);
    let ret = rcl_publisher_fini(&mut publisher, fixture.node());
    assert_eq!(ret, RCL_RET_OK as i32);
}
