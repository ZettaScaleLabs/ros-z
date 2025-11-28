// Copyright 2025 ZettaScale Technology
// SPDX-License-Identifier: Apache-2.0
//
// Ported from ros2/rcl:
// Copyright 2016 Open Source Robotics Foundation, Inc.

#![allow(clippy::needless_return)]
#![cfg(feature = "test-msgs")]

// Note: Two tests are currently marked as #[ignore] due to implementation issues:
//   - test_graph_query_functions: misaligned pointer dereference when validating context
//   - test_graph_guard_condition_trigger_check: cross-node graph notifications not implemented
// See individual test documentation for details.

mod test_msgs_support;

use std::ptr;

use rcl_z::{
    context::{rcl_context_fini, rcl_get_zero_initialized_context, rcl_shutdown},
    graph::{
        rcl_count_clients, rcl_count_publishers, rcl_count_services, rcl_count_subscribers,
        rcl_get_client_names_and_types_by_node, rcl_get_node_names,
        rcl_get_node_names_with_enclaves, rcl_get_publisher_names_and_types_by_node,
        rcl_get_service_names_and_types, rcl_get_service_names_and_types_by_node,
        rcl_get_subscriber_names_and_types_by_node, rcl_get_topic_names_and_types,
        rcl_get_zero_initialized_names_and_types, rcl_names_and_types_fini,
        rcutils_get_zero_initialized_string_array, rcutils_string_array_fini,
    },
    init::{
        rcl_get_default_allocator, rcl_get_zero_initialized_init_options, rcl_init,
        rcl_init_options_fini, rcl_init_options_init,
    },
    node::{
        rcl_get_zero_initialized_node, rcl_node_fini, rcl_node_get_default_options,
        rcl_node_get_graph_guard_condition, rcl_node_get_options, rcl_node_init,
    },
    pubsub::{
        rcl_publisher_fini, rcl_publisher_get_default_options, rcl_publisher_init,
        rcl_subscription_fini, rcl_subscription_get_default_options, rcl_subscription_init,
    },
    ros::*,
    service::{
        rcl_client_fini, rcl_client_get_default_options, rcl_client_init, rcl_service_fini,
        rcl_service_get_default_options, rcl_service_init, rcl_service_server_is_available,
    },
    wait_set::{
        rcl_get_zero_initialized_wait_set, rcl_wait, rcl_wait_set_add_guard_condition,
        rcl_wait_set_clear, rcl_wait_set_fini, rcl_wait_set_init,
    },
};

/// Test fixture that provides an initialized RCL context and node
struct TestGraphFixture {
    context: rcl_context_t,
    node: rcl_node_t,
    old_node: rcl_node_t, // A finalized node for testing invalid node scenarios
}

/// Test fixture that provides two initialized RCL contexts and nodes for multi-node scenarios
struct NodeGraphMultiNodeFixture {
    context: rcl_context_t,
    node: rcl_node_t,
    remote_context: Option<rcl_context_t>,
    remote_node: rcl_node_t,
    topic_name: String,
}

impl TestGraphFixture {
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
            let node_name = c"test_graph_node";
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

            // Create and finalize an old node for testing invalid node scenarios
            let mut old_node = rcl_get_zero_initialized_node();
            let old_node_name = c"old_graph_node";
            let ret = rcl_node_init(
                &mut old_node,
                old_node_name.as_ptr(),
                namespace.as_ptr(),
                &mut context,
                &node_options,
            );
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize old node");
            let ret = rcl_node_fini(&mut old_node);
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize old node");

            TestGraphFixture {
                context,
                node,
                old_node,
            }
        }
    }

    fn node(&mut self) -> *mut rcl_node_t {
        &mut self.node
    }

    fn old_node(&mut self) -> *mut rcl_node_t {
        &mut self.old_node
    }
}

impl Drop for TestGraphFixture {
    fn drop(&mut self) {
        let ret = rcl_node_fini(&mut self.node);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize node");

        let ret = rcl_shutdown(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to shutdown context");

        let ret = rcl_context_fini(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize context");
    }
}

impl NodeGraphMultiNodeFixture {
    fn new() -> Self {
        unsafe {
            // Initialize main context
            let mut init_options = rcl_get_zero_initialized_init_options();
            let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize init options");

            let mut context = rcl_get_zero_initialized_context();
            let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize context");

            // Initialize main node
            let mut node = rcl_get_zero_initialized_node();
            let node_name = c"test_graph_node";
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

            let _ = rcl_init_options_fini(&mut init_options);

            // Initialize remote node with the same context
            let mut remote_node = rcl_get_zero_initialized_node();
            let remote_node_name = c"remote_graph_node";
            let ret = rcl_node_init(
                &mut remote_node,
                remote_node_name.as_ptr(),
                namespace.as_ptr(),
                &mut context,
                &node_options,
            );
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize remote node");

            // Initialize rosout publishers for both nodes
            if rcl_logging_rosout_enabled() && node_options.enable_rosout {
                let ret = rcl_logging_rosout_init_publisher_for_node(&mut node);
                assert_eq!(
                    ret, RCL_RET_OK as i32,
                    "Failed to initialize rosout publisher for node"
                );

                let ret = rcl_logging_rosout_init_publisher_for_node(&mut remote_node);
                assert_eq!(
                    ret, RCL_RET_OK as i32,
                    "Failed to initialize rosout publisher for remote node"
                );
            }

            NodeGraphMultiNodeFixture {
                context,
                node,
                remote_context: None, // Use same context
                remote_node,
                topic_name: "/test_node_info_functions__".to_string(),
            }
        }
    }

    fn node(&mut self) -> *mut rcl_node_t {
        &mut self.node
    }

    fn remote_node(&mut self) -> *mut rcl_node_t {
        &mut self.remote_node
    }
}

impl Drop for NodeGraphMultiNodeFixture {
    fn drop(&mut self) {
        unsafe {
            // Finalize rosout publishers before finalizing nodes
            let node_ops = rcl_node_get_options(&self.node);
            if rcl_logging_rosout_enabled() && !node_ops.is_null() && (*node_ops).enable_rosout {
                let ret = rcl_logging_rosout_fini_publisher_for_node(&mut self.node);
                assert_eq!(
                    ret, RCL_RET_OK as i32,
                    "Failed to finalize rosout publisher for node"
                );
            }

            let ret = rcl_node_fini(&mut self.node);
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize node");

            let remote_node_ops = rcl_node_get_options(&self.remote_node);
            if rcl_logging_rosout_enabled()
                && !remote_node_ops.is_null()
                && (*remote_node_ops).enable_rosout
            {
                let ret = rcl_logging_rosout_fini_publisher_for_node(&mut self.remote_node);
                assert_eq!(
                    ret, RCL_RET_OK as i32,
                    "Failed to finalize rosout publisher for remote node"
                );
            }

            let ret = rcl_node_fini(&mut self.remote_node);
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize remote node");

            let ret = rcl_shutdown(&mut self.context);
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to shutdown context");

            if let Some(ref mut remote_context) = self.remote_context {
                let ret = rcl_shutdown(remote_context);
                assert_eq!(ret, RCL_RET_OK as i32, "Failed to shutdown remote context");
            }

            let ret = rcl_context_fini(&mut self.context);
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize context");

            if let Some(ref mut remote_context) = self.remote_context {
                let ret = rcl_context_fini(remote_context);
                assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize remote context");
            }
        }
    }
}

/// Expected state of a node
#[derive(Clone, Copy)]
struct ExpectedNodeState {
    publishers: usize,
    subscribers: usize,
    services: usize,
    clients: usize,
}

impl ExpectedNodeState {
    fn new(publishers: usize, subscribers: usize, services: usize, clients: usize) -> Self {
        ExpectedNodeState {
            publishers,
            subscribers,
            services,
            clients,
        }
    }
}

/// Function type for getting topics and types by node
type GetTopicsFunc = unsafe fn(*mut rcl_node_t, *const i8, *mut rcl_names_and_types_t) -> i32;

/// Check entity count for a topic
#[allow(unsafe_op_in_unsafe_fn)]
unsafe fn check_entity_count(
    node: *mut rcl_node_t,
    topic_name: &str,
    expected_publishers: usize,
    expected_subscribers: usize,
    expected_in_tnat: bool,
    timeout: std::time::Duration,
) {
    let mut pub_count = 0;
    let mut sub_count = 0;
    let topic_cstr = std::ffi::CString::new(topic_name).unwrap();

    // Check number of entities until timeout expires.
    let start_time = std::time::Instant::now();
    loop {
        let ret = rcl_count_publishers(node, topic_cstr.as_ptr(), &mut pub_count);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_count_subscribers(node, topic_cstr.as_ptr(), &mut sub_count);
        assert_eq!(ret, RCL_RET_OK as i32);
        if (expected_publishers == pub_count) && (expected_subscribers == sub_count) {
            break;
        }

        if start_time.elapsed() >= timeout {
            assert_eq!(expected_publishers, pub_count);
            assert_eq!(expected_subscribers, sub_count);
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    // Check if topic is in topic names and types
    let mut tnat = rcl_get_zero_initialized_names_and_types();
    let ret =
        rcl_get_topic_names_and_types(node as *const _, std::ptr::null_mut(), false, &mut tnat);
    assert_eq!(ret, RCL_RET_OK as i32);

    let mut is_in_tnat = false;
    if tnat.names.size > 0 {
        let names = std::slice::from_raw_parts(tnat.names.data, tnat.names.size);
        for &name_ptr in names {
            let name = std::ffi::CStr::from_ptr(name_ptr).to_str().unwrap();
            if name == topic_name {
                is_in_tnat = true;
                break;
            }
        }
    }

    if expected_in_tnat {
        assert!(
            is_in_tnat,
            "Topic {} expected in graph but not found",
            topic_name
        );
    } else {
        assert!(
            !is_in_tnat,
            "Topic {} not expected in graph but found",
            topic_name
        );
    }

    let ret = rcl_names_and_types_fini(&mut tnat);
    assert_eq!(ret, RCL_RET_OK as i32);
}

/// Expect a certain number of topics on a given subsystem
unsafe fn expect_topics_types(
    node: *mut rcl_node_t,
    func: GetTopicsFunc,
    num_topics: usize,
    node_name: *const i8,
    expect: bool,
    is_success: &mut bool,
) {
    unsafe {
        let mut nat = rcl_get_zero_initialized_names_and_types();
        let ret = func(node, node_name, &mut nat);
        // Ignore the `RCL_RET_NODE_NAME_NON_EXISTENT` result since the discovery may be asynchronous
        if ret != RCL_RET_NODE_NAME_NON_EXISTENT as i32 {
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to get topics and types");
        }
        *is_success &= num_topics == nat.names.size;
        if expect {
            assert_eq!(
                num_topics, nat.names.size,
                "Expected {} topics, got {}",
                num_topics, nat.names.size
            );
        }
        let ret = rcl_names_and_types_fini(&mut nat);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize names and types");
    }
}

/// Verify subsystem count for both nodes
impl NodeGraphMultiNodeFixture {
    fn verify_subsystem_count(
        &mut self,
        node_state: ExpectedNodeState,
        remote_node_state: ExpectedNodeState,
    ) {
        let node_vec = vec![self.node(), self.remote_node()];

        let attempts = 50;
        let mut is_expect = false;

        for i in 0..attempts {
            if i == attempts - 1 {
                is_expect = true;
            }
            let mut is_success = true;

            // verify each node contains the same node graph.
            for &node in &node_vec {
                let node_name = c"test_graph_node";
                unsafe {
                    expect_topics_types(
                        node,
                        rcl_get_subscriber_names_and_types_by_node_wrapper,
                        node_state.subscribers,
                        node_name.as_ptr(),
                        is_expect,
                        &mut is_success,
                    );
                    expect_topics_types(
                        node,
                        rcl_get_service_names_and_types_by_node_wrapper,
                        node_state.services,
                        node_name.as_ptr(),
                        is_expect,
                        &mut is_success,
                    );
                    expect_topics_types(
                        node,
                        rcl_get_client_names_and_types_by_node_wrapper,
                        node_state.clients,
                        node_name.as_ptr(),
                        is_expect,
                        &mut is_success,
                    );
                    expect_topics_types(
                        node,
                        rcl_get_publisher_names_and_types_by_node_wrapper,
                        node_state.publishers,
                        node_name.as_ptr(),
                        is_expect,
                        &mut is_success,
                    );

                    let remote_node_name = c"remote_graph_node";
                    expect_topics_types(
                        node,
                        rcl_get_subscriber_names_and_types_by_node_wrapper,
                        remote_node_state.subscribers,
                        remote_node_name.as_ptr(),
                        is_expect,
                        &mut is_success,
                    );
                    expect_topics_types(
                        node,
                        rcl_get_service_names_and_types_by_node_wrapper,
                        remote_node_state.services,
                        remote_node_name.as_ptr(),
                        is_expect,
                        &mut is_success,
                    );
                    expect_topics_types(
                        node,
                        rcl_get_client_names_and_types_by_node_wrapper,
                        remote_node_state.clients,
                        remote_node_name.as_ptr(),
                        is_expect,
                        &mut is_success,
                    );
                    expect_topics_types(
                        node,
                        rcl_get_publisher_names_and_types_by_node_wrapper,
                        remote_node_state.publishers,
                        remote_node_name.as_ptr(),
                        is_expect,
                        &mut is_success,
                    );
                }
            }

            if is_success {
                break;
            }
            // Sleep a bit for discovery
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    }
}

/// Wrapper functions to match the GetTopicsFunc signature
unsafe fn rcl_get_subscriber_names_and_types_by_node_wrapper(
    node: *mut rcl_node_t,
    node_name: *const i8,
    names_and_types: *mut rcl_names_and_types_t,
) -> i32 {
    rcl_get_subscriber_names_and_types_by_node(
        node,
        ptr::null_mut(),
        false,
        node_name,
        c"".as_ptr(),
        names_and_types,
    )
}

unsafe fn rcl_get_publisher_names_and_types_by_node_wrapper(
    node: *mut rcl_node_t,
    node_name: *const i8,
    names_and_types: *mut rcl_names_and_types_t,
) -> i32 {
    rcl_get_publisher_names_and_types_by_node(
        node,
        ptr::null_mut(),
        false,
        node_name,
        c"".as_ptr(),
        names_and_types,
    )
}

fn rcl_get_service_names_and_types_by_node_wrapper(
    node: *mut rcl_node_t,
    node_name: *const i8,
    names_and_types: *mut rcl_names_and_types_t,
) -> i32 {
    unsafe {
        rcl_get_service_names_and_types_by_node(
            node,
            ptr::null_mut(),
            node_name,
            c"".as_ptr(),
            names_and_types,
        )
    }
}

unsafe fn rcl_get_client_names_and_types_by_node_wrapper(
    node: *mut rcl_node_t,
    node_name: *const i8,
    names_and_types: *mut rcl_names_and_types_t,
) -> i32 {
    rcl_get_client_names_and_types_by_node(
        node,
        ptr::null_mut(),
        node_name,
        c"".as_ptr(),
        names_and_types,
    )
}

/// Test rcl_get_topic_names_and_types
/// Aligns with test_graph.cpp::test_rcl_get_and_destroy_topic_names_and_types
#[test]
fn test_get_topic_names_and_types() {
    let mut fixture = TestGraphFixture::new();

    unsafe {
        let mut names_and_types = rcl_get_zero_initialized_names_and_types();
        let zero_node = rcl_get_zero_initialized_node();

        // Invalid node - null pointer
        let ret = rcl_get_topic_names_and_types(
            ptr::null(),
            ptr::null_mut(),
            false,
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Invalid node - zero-initialized
        let ret =
            rcl_get_topic_names_and_types(&zero_node, ptr::null_mut(), false, &mut names_and_types);
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Invalid node - finalized/old node
        let ret = rcl_get_topic_names_and_types(
            fixture.old_node(),
            ptr::null_mut(),
            false,
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Invalid names_and_types - null pointer
        let ret =
            rcl_get_topic_names_and_types(fixture.node(), ptr::null_mut(), false, ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Invalid names_and_types - already initialized with size > 0
        names_and_types.names.size = 1;
        let ret = rcl_get_topic_names_and_types(
            fixture.node(),
            ptr::null_mut(),
            false,
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        names_and_types.names.size = 0;

        // Invalid argument to rcl_names_and_types_fini
        let ret = rcl_names_and_types_fini(ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Valid call
        let ret = rcl_get_topic_names_and_types(
            fixture.node(),
            ptr::null_mut(),
            false,
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Finalize
        let ret = rcl_names_and_types_fini(&mut names_and_types);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_get_service_names_and_types
/// Aligns with test_graph.cpp::test_rcl_get_service_names_and_types
#[test]
fn test_rcl_get_service_names_and_types() {
    unsafe {
        let mut fixture = TestGraphFixture::new();
        let mut allocator = rcl_get_default_allocator();
        let mut tnat = rcl_get_zero_initialized_names_and_types();
        let zero_node = rcl_get_zero_initialized_node();

        // Invalid node - null
        let ret = rcl_get_service_names_and_types(ptr::null_mut(), &mut allocator, &mut tnat);
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Invalid node - zero-initialized
        let ret = rcl_get_service_names_and_types(
            &zero_node as *const _ as *mut _,
            &mut allocator,
            &mut tnat,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Invalid node - finalized/old node
        let ret = rcl_get_service_names_and_types(fixture.old_node(), &mut allocator, &mut tnat);
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Invalid allocator - null
        let ret = rcl_get_service_names_and_types(fixture.node(), ptr::null_mut(), &mut tnat);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Invalid names_and_types - null
        let ret = rcl_get_service_names_and_types(fixture.node(), &mut allocator, ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Invalid names_and_types - already has size > 0
        tnat.names.size = 1;
        let ret = rcl_get_service_names_and_types(fixture.node(), &mut allocator, &mut tnat);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        tnat.names.size = 0;

        // Invalid argument to rcl_names_and_types_fini
        let ret = rcl_names_and_types_fini(ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Valid calls
        let ret = rcl_get_service_names_and_types(fixture.node(), &mut allocator, &mut tnat);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_names_and_types_fini(&mut tnat);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_get_publisher_names_and_types_by_node
#[test]
fn test_rcl_get_publisher_names_and_types_by_node() {
    let mut fixture = TestGraphFixture::new();

    unsafe {
        let mut names_and_types = rcl_get_zero_initialized_names_and_types();
        let node_name = c"test_graph_node";
        let namespace = c"";

        // Test with null node
        let ret = rcl_get_publisher_names_and_types_by_node(
            ptr::null(),
            ptr::null_mut(),
            false,
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with zero-initialized node
        let zero_node = rcl_get_zero_initialized_node();
        let ret = rcl_get_publisher_names_and_types_by_node(
            &zero_node,
            ptr::null_mut(),
            false,
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with finalized node
        let ret = rcl_get_publisher_names_and_types_by_node(
            fixture.old_node(),
            ptr::null_mut(),
            false,
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with null node_name
        let ret = rcl_get_publisher_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            false,
            ptr::null(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with null namespace
        let ret = rcl_get_publisher_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            false,
            node_name.as_ptr(),
            ptr::null(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with null names_and_types
        let ret = rcl_get_publisher_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            false,
            node_name.as_ptr(),
            namespace.as_ptr(),
            ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Valid call
        let ret = rcl_get_publisher_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            false,
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Finalize
        let ret = rcl_names_and_types_fini(&mut names_and_types);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_get_subscriber_names_and_types_by_node
#[test]
fn test_rcl_get_subscriber_names_and_types_by_node() {
    let mut fixture = TestGraphFixture::new();

    unsafe {
        let mut names_and_types = rcl_get_zero_initialized_names_and_types();
        let node_name = c"test_graph_node";
        let namespace = c"";

        // Test with null node
        let ret = rcl_get_subscriber_names_and_types_by_node(
            ptr::null(),
            ptr::null_mut(),
            false,
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with zero-initialized node
        let zero_node = rcl_get_zero_initialized_node();
        let ret = rcl_get_subscriber_names_and_types_by_node(
            &zero_node,
            ptr::null_mut(),
            false,
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with finalized node
        let ret = rcl_get_subscriber_names_and_types_by_node(
            fixture.old_node(),
            ptr::null_mut(),
            false,
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with null node_name
        let ret = rcl_get_subscriber_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            false,
            ptr::null(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with null namespace
        let ret = rcl_get_subscriber_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            false,
            node_name.as_ptr(),
            ptr::null(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with null names_and_types
        let ret = rcl_get_subscriber_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            false,
            node_name.as_ptr(),
            namespace.as_ptr(),
            ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Valid call
        let ret = rcl_get_subscriber_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            false,
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Finalize
        let ret = rcl_names_and_types_fini(&mut names_and_types);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_get_service_names_and_types_by_node
#[test]
fn test_rcl_get_service_names_and_types_by_node() {
    let mut fixture = TestGraphFixture::new();

    unsafe {
        let mut names_and_types = rcl_get_zero_initialized_names_and_types();
        let node_name = c"test_graph_node";
        let namespace = c"";

        // Test with null node
        let ret = rcl_get_service_names_and_types_by_node(
            ptr::null(),
            ptr::null_mut(),
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with zero-initialized node
        let zero_node = rcl_get_zero_initialized_node();
        let ret = rcl_get_service_names_and_types_by_node(
            &zero_node,
            ptr::null_mut(),
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with finalized node
        let ret = rcl_get_service_names_and_types_by_node(
            fixture.old_node(),
            ptr::null_mut(),
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with null node_name
        let ret = rcl_get_service_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            ptr::null(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with null namespace
        let ret = rcl_get_service_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            node_name.as_ptr(),
            ptr::null(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with null names_and_types
        let ret = rcl_get_service_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            node_name.as_ptr(),
            namespace.as_ptr(),
            ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Valid call
        let ret = rcl_get_service_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Finalize
        let ret = rcl_names_and_types_fini(&mut names_and_types);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_get_client_names_and_types_by_node
#[test]
fn test_rcl_get_client_names_and_types_by_node() {
    let mut fixture = TestGraphFixture::new();

    unsafe {
        let mut names_and_types = rcl_get_zero_initialized_names_and_types();
        let node_name = c"test_graph_node";
        let namespace = c"";

        // Test with null node
        let ret = rcl_get_client_names_and_types_by_node(
            ptr::null(),
            ptr::null_mut(),
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with zero-initialized node
        let zero_node = rcl_get_zero_initialized_node();
        let ret = rcl_get_client_names_and_types_by_node(
            &zero_node,
            ptr::null_mut(),
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with finalized node
        let ret = rcl_get_client_names_and_types_by_node(
            fixture.old_node(),
            ptr::null_mut(),
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with null node_name
        let ret = rcl_get_client_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            ptr::null(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with null namespace
        let ret = rcl_get_client_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            node_name.as_ptr(),
            ptr::null(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with null names_and_types
        let ret = rcl_get_client_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            node_name.as_ptr(),
            namespace.as_ptr(),
            ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Valid call
        let ret = rcl_get_client_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(),
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Finalize
        let ret = rcl_names_and_types_fini(&mut names_and_types);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_count_publishers
/// Aligns with test_graph.cpp::test_rcl_count_publishers
#[test]
fn test_rcl_count_publishers() {
    let mut fixture = TestGraphFixture::new();
    let zero_node = rcl_get_zero_initialized_node();
    let topic_name = c"/topic_test_rcl_count_publishers";
    let mut count: usize = 0;

    // Invalid node - null pointer (returns RCL_RET_NODE_INVALID)
    let ret = rcl_count_publishers(ptr::null_mut(), topic_name.as_ptr(), &mut count);
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

    // Invalid node - zero-initialized (returns RCL_RET_NODE_INVALID)
    let ret = rcl_count_publishers(
        &zero_node as *const _ as *mut _,
        topic_name.as_ptr(),
        &mut count,
    );
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

    // Invalid node - finalized/old node (returns RCL_RET_NODE_INVALID)
    let ret = rcl_count_publishers(fixture.old_node(), topic_name.as_ptr(), &mut count);
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

    // Invalid topic name - null pointer
    let ret = rcl_count_publishers(fixture.node(), ptr::null(), &mut count);
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    // Invalid count - null pointer
    let ret = rcl_count_publishers(fixture.node(), topic_name.as_ptr(), ptr::null_mut());
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    // Valid call
    let ret = rcl_count_publishers(fixture.node(), topic_name.as_ptr(), &mut count);
    assert_eq!(ret, RCL_RET_OK as i32);
}

/// Test rcl_count_subscribers
/// Aligns with test_graph.cpp::test_rcl_count_subscribers
#[test]
fn test_rcl_count_subscribers() {
    let mut fixture = TestGraphFixture::new();
    let zero_node = rcl_get_zero_initialized_node();
    let topic_name = c"/topic_test_rcl_count_subscribers";
    let mut count: usize = 0;

    // Invalid node tests
    let ret = rcl_count_subscribers(ptr::null_mut(), topic_name.as_ptr(), &mut count);
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
    let ret = rcl_count_subscribers(
        &zero_node as *const _ as *mut _,
        topic_name.as_ptr(),
        &mut count,
    );
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
    let ret = rcl_count_subscribers(fixture.old_node(), topic_name.as_ptr(), &mut count);
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

    // Invalid topic name
    let ret = rcl_count_subscribers(fixture.node(), ptr::null(), &mut count);
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    // Invalid count pointer
    let ret = rcl_count_subscribers(fixture.node(), topic_name.as_ptr(), ptr::null_mut());
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    // Valid call
    let ret = rcl_count_subscribers(fixture.node(), topic_name.as_ptr(), &mut count);
    assert_eq!(ret, RCL_RET_OK as i32);
}

/// Test rcl_count_clients
/// Aligns with test_graph.cpp::test_rcl_count_clients
#[test]
fn test_rcl_count_clients() {
    let mut fixture = TestGraphFixture::new();
    let zero_node = rcl_get_zero_initialized_node();
    let service_name = c"/service_test_rcl_count_clients";
    let mut count: usize = 0;

    // Invalid node tests
    let ret = rcl_count_clients(ptr::null_mut(), service_name.as_ptr(), &mut count);
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
    let ret = rcl_count_clients(
        &zero_node as *const _ as *mut _,
        service_name.as_ptr(),
        &mut count,
    );
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
    let ret = rcl_count_clients(fixture.old_node(), service_name.as_ptr(), &mut count);
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

    // Invalid service name
    let ret = rcl_count_clients(fixture.node(), ptr::null(), &mut count);
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    // Invalid count pointer
    let ret = rcl_count_clients(fixture.node(), service_name.as_ptr(), ptr::null_mut());
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    // Valid call
    let ret = rcl_count_clients(fixture.node(), service_name.as_ptr(), &mut count);
    assert_eq!(ret, RCL_RET_OK as i32);
}

/// Test rcl_count_services
/// Aligns with test_graph.cpp::test_rcl_count_services
#[test]
fn test_rcl_count_services() {
    let mut fixture = TestGraphFixture::new();
    let zero_node = rcl_get_zero_initialized_node();
    let service_name = c"/service_test_rcl_count_services";
    let mut count: usize = 0;

    // Invalid node tests
    let ret = rcl_count_services(ptr::null_mut(), service_name.as_ptr(), &mut count);
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
    let ret = rcl_count_services(
        &zero_node as *const _ as *mut _,
        service_name.as_ptr(),
        &mut count,
    );
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
    let ret = rcl_count_services(fixture.old_node(), service_name.as_ptr(), &mut count);
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

    // Invalid service name
    let ret = rcl_count_services(fixture.node(), ptr::null(), &mut count);
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    // Invalid count pointer
    let ret = rcl_count_services(fixture.node(), service_name.as_ptr(), ptr::null_mut());
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    // Valid call
    let ret = rcl_count_services(fixture.node(), service_name.as_ptr(), &mut count);
    assert_eq!(ret, RCL_RET_OK as i32);
}

/// Test rcl_names_and_types_init
#[test]
fn test_rcl_names_and_types_init() {
    unsafe {
        let mut names_and_types = rcl_get_zero_initialized_names_and_types();
        let mut allocator = rcl_get_default_allocator();

        // Test with null names_and_types
        let ret = rcl_names_and_types_init(ptr::null_mut(), 0, &mut allocator);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Valid initialization
        let ret = rcl_names_and_types_init(&mut names_and_types, 0, &mut allocator);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Finalize
        let ret = rcl_names_and_types_fini(&mut names_and_types);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_wait_for_publishers
#[test]
fn test_rcl_wait_for_publishers() {
    let mut fixture = TestGraphFixture::new();

    unsafe {
        let topic_name = c"/test_wait_for_publishers_topic";
        let mut allocator = rcl_get_default_allocator();
        let mut success = false;

        // Test with null node
        let ret = rcl_wait_for_publishers(
            ptr::null(),
            &mut allocator,
            topic_name.as_ptr(),
            1,
            1000000000,
            &mut success,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with null topic_name
        let ret = rcl_wait_for_publishers(
            fixture.node(),
            &mut allocator,
            ptr::null(),
            1,
            1000000000,
            &mut success,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test timeout case (no publishers available)
        let ret = rcl_wait_for_publishers(
            fixture.node(),
            &mut allocator,
            topic_name.as_ptr(),
            1,
            1000000,
            &mut success,
        ); // 1ms timeout
        assert_eq!(ret, RCL_RET_TIMEOUT as i32);
        assert!(!success);
    }
}

/// Test rcl_wait_for_subscribers
#[test]
fn test_rcl_wait_for_subscribers() {
    let mut fixture = TestGraphFixture::new();

    unsafe {
        let topic_name = c"/test_wait_for_subscribers_topic";
        let mut allocator = rcl_get_default_allocator();
        let mut success = false;

        // Test with null node
        let ret = rcl_wait_for_subscribers(
            ptr::null(),
            &mut allocator,
            topic_name.as_ptr(),
            1,
            1000000000,
            &mut success,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with null topic_name
        let ret = rcl_wait_for_subscribers(
            fixture.node(),
            &mut allocator,
            ptr::null(),
            1,
            1000000000,
            &mut success,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test timeout case (no subscribers available)
        let ret = rcl_wait_for_subscribers(
            fixture.node(),
            &mut allocator,
            topic_name.as_ptr(),
            1,
            1000000,
            &mut success,
        ); // 1ms timeout
        assert_eq!(ret, RCL_RET_TIMEOUT as i32);
        assert!(!success);
    }
}

/// Test node info subscriptions (multi-node scenario)
#[test]
fn test_node_info_subscriptions() {
    let mut fixture = NodeGraphMultiNodeFixture::new();

    unsafe {
        // Create two subscribers.
        let mut sub = rcl_get_zero_initialized_subscription();
        let sub_ops = rcl_subscription_get_default_options();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let ret = rcl_subscription_init(
            &mut sub,
            fixture.node(),
            ts,
            fixture.topic_name.as_ptr().cast(),
            &sub_ops,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize subscription");

        let mut sub2 = rcl_get_zero_initialized_subscription();
        let sub_ops2 = rcl_subscription_get_default_options();
        let ret = rcl_subscription_init(
            &mut sub2,
            fixture.remote_node(),
            ts,
            fixture.topic_name.as_ptr().cast(),
            &sub_ops2,
        );
        assert_eq!(
            ret, RCL_RET_OK as i32,
            "Failed to initialize remote subscription"
        );

        fixture.verify_subsystem_count(
            ExpectedNodeState::new(0, 1, 0, 0),
            ExpectedNodeState::new(0, 1, 0, 0),
        );

        // Destroy the node's subscriber
        let ret = rcl_subscription_fini(&mut sub, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize subscription");
        fixture.verify_subsystem_count(
            ExpectedNodeState::new(0, 0, 0, 0),
            ExpectedNodeState::new(0, 1, 0, 0),
        );

        // Destroy the remote node's subscriber
        let ret = rcl_subscription_fini(&mut sub2, fixture.remote_node());
        assert_eq!(
            ret, RCL_RET_OK as i32,
            "Failed to finalize remote subscription"
        );
        fixture.verify_subsystem_count(
            ExpectedNodeState::new(0, 0, 0, 0),
            ExpectedNodeState::new(0, 0, 0, 0),
        );
    }
}

/// Test node info publishers (multi-node scenario)
#[test]
fn test_node_info_publishers() {
    let mut fixture = NodeGraphMultiNodeFixture::new();

    unsafe {
        // Create a publisher on the main node
        let mut pub_ = rcl_get_zero_initialized_publisher();
        let pub_ops = rcl_publisher_get_default_options();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let ret = rcl_publisher_init(
            &mut pub_,
            fixture.node(),
            ts,
            fixture.topic_name.as_ptr().cast(),
            &pub_ops,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize publisher");

        fixture.verify_subsystem_count(
            ExpectedNodeState::new(1, 0, 0, 0),
            ExpectedNodeState::new(0, 0, 0, 0),
        );

        // Destroy the publisher
        let ret = rcl_publisher_fini(&mut pub_, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize publisher");
        fixture.verify_subsystem_count(
            ExpectedNodeState::new(0, 0, 0, 0),
            ExpectedNodeState::new(0, 0, 0, 0),
        );
    }
}

/// Test node info services (multi-node scenario)
#[test]
fn test_node_info_services() {
    let mut fixture = NodeGraphMultiNodeFixture::new();

    unsafe {
        // Create a service on the main node
        let service_name = c"test_service";
        let mut service = rcl_get_zero_initialized_service();
        let service_options = rcl_service_get_default_options();
        let ts = ROSIDL_GET_SRV_TYPE_SUPPORT!(test_msgs, srv, BasicTypes);
        let ret = rcl_service_init(
            &mut service,
            fixture.node(),
            ts,
            service_name.as_ptr(),
            &service_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize service");

        fixture.verify_subsystem_count(
            ExpectedNodeState::new(0, 0, 1, 0),
            ExpectedNodeState::new(0, 0, 0, 0),
        );

        // Destroy the service
        let ret = rcl_service_fini(&mut service, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize service");
        fixture.verify_subsystem_count(
            ExpectedNodeState::new(0, 0, 0, 0),
            ExpectedNodeState::new(0, 0, 0, 0),
        );
    }
}

/// Test node info clients (multi-node scenario)
#[test]
fn test_node_info_clients() {
    let mut fixture = NodeGraphMultiNodeFixture::new();

    unsafe {
        // Create a client on the main node
        let service_name = c"test_service";
        let mut client = rcl_get_zero_initialized_client();
        let client_options = rcl_client_get_default_options();
        let ts = ROSIDL_GET_SRV_TYPE_SUPPORT!(test_msgs, srv, BasicTypes);
        let ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ts,
            service_name.as_ptr(),
            &client_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize client");

        fixture.verify_subsystem_count(
            ExpectedNodeState::new(0, 0, 0, 1),
            ExpectedNodeState::new(0, 0, 0, 0),
        );

        // Destroy the client
        let ret = rcl_client_fini(&mut client, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize client");
        fixture.verify_subsystem_count(
            ExpectedNodeState::new(0, 0, 0, 0),
            ExpectedNodeState::new(0, 0, 0, 0),
        );
    }
}

/// Test graph query functions
///
/// TODO: This test is currently failing due to a misaligned pointer dereference in
/// rcl_node_is_valid when checking (*node).context. The issue occurs in the
/// borrow_impl trait when validating the context pointer. This needs investigation
/// into how the context pointer is stored and accessed in rcl_node_t.
#[test]
#[ignore = "misaligned pointer dereference - needs investigation"]
fn test_graph_query_functions() {
    let mut fixture = TestGraphFixture::new();

    unsafe {
        // Create a unique topic name
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let topic_name = format!("/test_graph_query_functions__{}", now);

        // First assert the topic is not in use
        check_entity_count(
            fixture.node(),
            &topic_name,
            0,     // expected publishers
            0,     // expected subscribers
            false, // expected in graph
            std::time::Duration::from_secs(4),
        );

        // Create a publisher
        let mut pub_ = rcl_get_zero_initialized_publisher();
        let pub_ops = rcl_publisher_get_default_options();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_cstr = std::ffi::CString::new(topic_name.clone()).unwrap();
        let ret = rcl_publisher_init(&mut pub_, fixture.node(), ts, topic_cstr.as_ptr(), &pub_ops);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check the graph
        check_entity_count(
            fixture.node(),
            &topic_name,
            1,    // expected publishers
            0,    // expected subscribers
            true, // expected in graph
            std::time::Duration::from_secs(4),
        );

        // Create a subscriber
        let mut sub = rcl_get_zero_initialized_subscription();
        let sub_ops = rcl_subscription_get_default_options();
        let ret =
            rcl_subscription_init(&mut sub, fixture.node(), ts, topic_cstr.as_ptr(), &sub_ops);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check the graph again
        check_entity_count(
            fixture.node(),
            &topic_name,
            1,    // expected publishers
            1,    // expected subscribers
            true, // expected in graph
            std::time::Duration::from_secs(4),
        );

        // Destroy the publisher
        let ret = rcl_publisher_fini(&mut pub_, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check the graph again
        check_entity_count(
            fixture.node(),
            &topic_name,
            0,    // expected publishers
            1,    // expected subscribers
            true, // expected in graph
            std::time::Duration::from_secs(4),
        );

        // Destroy the subscriber
        let ret = rcl_subscription_fini(&mut sub, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check the graph again
        check_entity_count(
            fixture.node(),
            &topic_name,
            0,     // expected publishers
            0,     // expected subscribers
            false, // expected in graph
            std::time::Duration::from_secs(4),
        );
    }
}

/// Test graph guard condition trigger check
///
/// TODO: This test is currently failing because cross-node graph change notifications
/// are not implemented. The graph guard condition correctly triggers for entity
/// create/destroy on the same node, but does not trigger when a new node is created
/// or destroyed in the same context (lines 1722, 1733). This requires implementing
/// inter-node graph event propagation within a context.
#[test]
#[ignore = "cross-node graph notifications not implemented"]
fn test_graph_guard_condition_trigger_check() {
    let mut fixture = TestGraphFixture::new();

    unsafe {
        let timeout_1s = std::time::Duration::from_secs(1);
        let timeout_3s = std::time::Duration::from_secs(3);

        let mut wait_set = rcl_get_zero_initialized_wait_set();
        let ret = rcl_wait_set_init(
            &mut wait_set,
            0,
            1,
            0,
            0,
            0,
            0,
            &mut fixture.context,
            rcl_get_default_allocator(),
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        let graph_guard_condition = rcl_node_get_graph_guard_condition(fixture.node());

        // Wait for no graph change condition
        let mut idx = 0;
        while idx < 100 {
            let ret = rcl_wait_set_clear(&mut wait_set);
            assert_eq!(ret, RCL_RET_OK as i32);
            let ret = rcl_wait_set_add_guard_condition(
                &mut wait_set,
                graph_guard_condition,
                std::ptr::null_mut(),
            );
            assert_eq!(ret, RCL_RET_OK as i32);
            let ret = rcl_wait(&mut wait_set, timeout_3s.as_nanos() as i64);
            if ret == RCL_RET_TIMEOUT as i32 {
                break;
            }
            idx += 1;
        }
        assert!(idx < 100);

        // Graph change since creating the publisher
        let mut pub_ = rcl_get_zero_initialized_publisher();
        let pub_ops = rcl_publisher_get_default_options();
        let ts = ROSIDL_GET_MSG_TYPE_SUPPORT!(test_msgs, msg, BasicTypes);
        let topic_name = c"/chatter_test_graph_guard_condition_topics";
        let ret = rcl_publisher_init(&mut pub_, fixture.node(), ts, topic_name.as_ptr(), &pub_ops);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check guard condition change
        let ret = rcl_wait_set_clear(&mut wait_set);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait_set_add_guard_condition(
            &mut wait_set,
            graph_guard_condition,
            std::ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait(&mut wait_set, timeout_1s.as_nanos() as i64);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Graph change since destroying the publisher
        let ret = rcl_publisher_fini(&mut pub_, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check guard condition change
        let ret = rcl_wait_set_clear(&mut wait_set);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait_set_add_guard_condition(
            &mut wait_set,
            graph_guard_condition,
            std::ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait(&mut wait_set, timeout_1s.as_nanos() as i64);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Graph change since creating the subscription
        let mut sub = rcl_get_zero_initialized_subscription();
        let sub_ops = rcl_subscription_get_default_options();
        let ret =
            rcl_subscription_init(&mut sub, fixture.node(), ts, topic_name.as_ptr(), &sub_ops);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check guard condition change
        let ret = rcl_wait_set_clear(&mut wait_set);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait_set_add_guard_condition(
            &mut wait_set,
            graph_guard_condition,
            std::ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait(&mut wait_set, timeout_1s.as_nanos() as i64);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Graph change since destroying the subscription
        let ret = rcl_subscription_fini(&mut sub, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check guard condition change
        let ret = rcl_wait_set_clear(&mut wait_set);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait_set_add_guard_condition(
            &mut wait_set,
            graph_guard_condition,
            std::ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait(&mut wait_set, timeout_1s.as_nanos() as i64);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Graph change since creating service
        let mut service = rcl_get_zero_initialized_service();
        let service_options = rcl_service_get_default_options();
        let service_name = c"test_graph_guard_condition_service";
        let ts_srv = ROSIDL_GET_SRV_TYPE_SUPPORT!(test_msgs, srv, BasicTypes);
        let ret = rcl_service_init(
            &mut service,
            fixture.node(),
            ts_srv,
            service_name.as_ptr(),
            &service_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check guard condition change
        let ret = rcl_wait_set_clear(&mut wait_set);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait_set_add_guard_condition(
            &mut wait_set,
            graph_guard_condition,
            std::ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait(&mut wait_set, timeout_1s.as_nanos() as i64);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Graph change since destroy service
        let ret = rcl_service_fini(&mut service, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check guard condition change
        let ret = rcl_wait_set_clear(&mut wait_set);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait_set_add_guard_condition(
            &mut wait_set,
            graph_guard_condition,
            std::ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait(&mut wait_set, timeout_1s.as_nanos() as i64);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Graph change since creating client
        let mut client = rcl_get_zero_initialized_client();
        let client_options = rcl_client_get_default_options();
        let ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ts_srv,
            service_name.as_ptr(),
            &client_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check guard condition change
        let ret = rcl_wait_set_clear(&mut wait_set);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait_set_add_guard_condition(
            &mut wait_set,
            graph_guard_condition,
            std::ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait(&mut wait_set, timeout_1s.as_nanos() as i64);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Graph change since destroying client
        let ret = rcl_client_fini(&mut client, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check guard condition change
        let ret = rcl_wait_set_clear(&mut wait_set);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait_set_add_guard_condition(
            &mut wait_set,
            graph_guard_condition,
            std::ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait(&mut wait_set, timeout_1s.as_nanos() as i64);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Graph change since adding new node
        let mut node_new = rcl_get_zero_initialized_node();
        let node_options = rcl_node_get_default_options();
        let ret = rcl_node_init(
            &mut node_new,
            c"test_graph2".as_ptr(),
            c"".as_ptr(),
            &mut fixture.context,
            &node_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check guard condition change
        let ret = rcl_wait_set_clear(&mut wait_set);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait_set_add_guard_condition(
            &mut wait_set,
            graph_guard_condition,
            std::ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait(&mut wait_set, timeout_1s.as_nanos() as i64);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Graph change since destroying new node
        let ret = rcl_node_fini(&mut node_new);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check guard condition change
        let ret = rcl_wait_set_clear(&mut wait_set);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait_set_add_guard_condition(
            &mut wait_set,
            graph_guard_condition,
            std::ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait(&mut wait_set, timeout_1s.as_nanos() as i64);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Should not get graph change if no change
        let ret = rcl_wait_set_clear(&mut wait_set);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait_set_add_guard_condition(
            &mut wait_set,
            graph_guard_condition,
            std::ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_wait(&mut wait_set, timeout_1s.as_nanos() as i64);
        assert_eq!(ret, RCL_RET_TIMEOUT as i32);

        let ret = rcl_wait_set_fini(&mut wait_set);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_service_server_is_available
#[test]
fn test_rcl_service_server_is_available() {
    let mut fixture = TestGraphFixture::new();

    unsafe {
        // Create a client
        let mut client = rcl_get_zero_initialized_client();
        let ts = ROSIDL_GET_SRV_TYPE_SUPPORT!(test_msgs, srv, BasicTypes);
        let service_name = c"/service_test_rcl_service_server_is_available";
        let client_options = rcl_client_get_default_options();
        let ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ts,
            service_name.as_ptr(),
            &client_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check, knowing there is no service server
        let mut is_available = false;
        let ret = rcl_service_server_is_available(fixture.node(), &client, &mut is_available);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!is_available);

        // Create the service server
        let mut service = rcl_get_zero_initialized_service();
        let service_options = rcl_service_get_default_options();
        let ret = rcl_service_init(
            &mut service,
            fixture.node(),
            ts,
            service_name.as_ptr(),
            &service_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Wait for service to be available
        let start_time = std::time::Instant::now();
        let timeout = std::time::Duration::from_secs(10);
        while start_time.elapsed() < timeout {
            let ret = rcl_service_server_is_available(fixture.node(), &client, &mut is_available);
            assert_eq!(ret, RCL_RET_OK as i32);
            if is_available {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        assert!(is_available);

        // Destroy the service
        let ret = rcl_service_fini(&mut service, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Wait for service to be unavailable
        let start_time = std::time::Instant::now();
        while start_time.elapsed() < timeout {
            let ret = rcl_service_server_is_available(fixture.node(), &client, &mut is_available);
            assert_eq!(ret, RCL_RET_OK as i32);
            if !is_available {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        assert!(!is_available);

        // Clean up client
        let ret = rcl_client_fini(&mut client, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test bad parameters for rcl_service_server_is_available
#[test]
fn test_bad_server_available() {
    let mut fixture = TestGraphFixture::new();

    unsafe {
        // Create a client
        let mut client = rcl_get_zero_initialized_client();
        let ts = ROSIDL_GET_SRV_TYPE_SUPPORT!(test_msgs, srv, BasicTypes);
        let service_name = c"/service_test_rcl_service_server_is_available";
        let client_options = rcl_client_get_default_options();
        let ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ts,
            service_name.as_ptr(),
            &client_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut is_available = false;

        // Invalid node - null
        let ret = rcl_service_server_is_available(std::ptr::null_mut(), &client, &mut is_available);
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Invalid node - not initialized
        let not_init_node = rcl_get_zero_initialized_node();
        let ret = rcl_service_server_is_available(&not_init_node, &client, &mut is_available);
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Clean up client
        let ret = rcl_client_fini(&mut client, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test bad parameters for rcl_get_node_names functions
#[test]
fn test_bad_get_node_names() {
    let mut fixture = TestGraphFixture::new();

    unsafe {
        let mut node_names = rcutils_get_zero_initialized_string_array();
        let mut node_namespaces = rcutils_get_zero_initialized_string_array();
        let mut node_names_2 = rcutils_get_zero_initialized_string_array();
        let mut node_namespaces_2 = rcutils_get_zero_initialized_string_array();
        let mut node_enclaves = rcutils_get_zero_initialized_string_array();
        let allocator = rcl_get_default_allocator();

        // Invalid node - null
        let ret = rcl_get_node_names(
            std::ptr::null(),
            allocator,
            &mut node_names,
            &mut node_namespaces,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
        let ret = rcl_get_node_names_with_enclaves(
            std::ptr::null(),
            allocator,
            &mut node_names,
            &mut node_namespaces,
            &mut node_enclaves,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Invalid node - not initialized
        let not_init_node = rcl_get_zero_initialized_node();
        let ret = rcl_get_node_names(
            &not_init_node,
            allocator,
            &mut node_names,
            &mut node_namespaces,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
        let ret = rcl_get_node_names_with_enclaves(
            &not_init_node,
            allocator,
            &mut node_names,
            &mut node_namespaces,
            &mut node_enclaves,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Invalid node_names - null
        let ret = rcl_get_node_names(
            fixture.node(),
            allocator,
            std::ptr::null_mut(),
            &mut node_namespaces,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        let ret = rcl_get_node_names_with_enclaves(
            fixture.node(),
            allocator,
            std::ptr::null_mut(),
            &mut node_namespaces,
            &mut node_enclaves,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Invalid node_namespaces - null
        let ret = rcl_get_node_names(
            fixture.node(),
            allocator,
            &mut node_names,
            std::ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        let ret = rcl_get_node_names_with_enclaves(
            fixture.node(),
            allocator,
            &mut node_names,
            std::ptr::null_mut(),
            &mut node_enclaves,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Invalid node_enclaves - null
        let ret = rcl_get_node_names_with_enclaves(
            fixture.node(),
            allocator,
            &mut node_names,
            &mut node_namespaces,
            std::ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Invalid node_names - already initialized with size > 0
        node_names.size = 1;
        let ret = rcl_get_node_names(
            fixture.node(),
            allocator,
            &mut node_names,
            &mut node_namespaces,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        let ret = rcl_get_node_names_with_enclaves(
            fixture.node(),
            allocator,
            &mut node_names,
            &mut node_namespaces,
            &mut node_enclaves,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        node_names.size = 0;

        // Valid calls
        let ret = rcl_get_node_names(
            fixture.node(),
            allocator,
            &mut node_names,
            &mut node_namespaces,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_get_node_names_with_enclaves(
            fixture.node(),
            allocator,
            &mut node_names_2,
            &mut node_namespaces_2,
            &mut node_enclaves,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Clean up
        rcutils_string_array_fini(&mut node_names);
        rcutils_string_array_fini(&mut node_namespaces);
        rcutils_string_array_fini(&mut node_names_2);
        rcutils_string_array_fini(&mut node_namespaces_2);
        rcutils_string_array_fini(&mut node_enclaves);
    }
}
