// Ported from Open Source Robotics Foundation code (2016)
// https://github.com/ros2/rcl
// Licensed under the Apache License 2.0
// Copyright 2025 ZettaScale Technology

#![allow(clippy::needless_return)]
#![cfg(feature = "test-msgs")]

// TODO: Implement missing test functions from C++ test_graph.cpp:
//   - test_node_info_subscriptions (multi-node scenario)
//   - test_node_info_publishers (multi-node scenario)
//   - test_node_info_services (multi-node scenario)
//   - test_node_info_clients (multi-node scenario)
//   - test_graph_query_functions (node name queries)
//   - test_graph_guard_condition_trigger_check (graph guard condition)

mod test_msgs_support;

use std::ptr;

use rcl_z::{
    context::{rcl_context_fini, rcl_get_zero_initialized_context, rcl_shutdown},
    graph::{
        rcl_count_clients, rcl_count_publishers, rcl_count_services, rcl_count_subscribers,
        rcl_get_client_names_and_types_by_node, rcl_get_publisher_names_and_types_by_node,
        rcl_get_service_names_and_types, rcl_get_service_names_and_types_by_node,
        rcl_get_subscriber_names_and_types_by_node, rcl_get_topic_names_and_types,
        rcl_get_zero_initialized_names_and_types, rcl_names_and_types_fini,
    },
    init::{
        rcl_get_default_allocator, rcl_get_zero_initialized_init_options, rcl_init,
        rcl_init_options_fini, rcl_init_options_init,
    },
    node::{
        rcl_get_zero_initialized_node, rcl_node_fini, rcl_node_get_default_options, rcl_node_init,
    },
    pubsub::{
        rcl_subscription_fini, rcl_subscription_get_default_options, rcl_subscription_init,
    },
    ros::*,
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
                assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize rosout publisher for node");

                let ret = rcl_logging_rosout_init_publisher_for_node(&mut remote_node);
                assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize rosout publisher for remote node");
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
                assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize rosout publisher for node");
            }

            let ret = rcl_node_fini(&mut self.node);
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize node");

            let remote_node_ops = rcl_node_get_options(&self.remote_node);
            if rcl_logging_rosout_enabled() && !remote_node_ops.is_null() && (*remote_node_ops).enable_rosout {
                let ret = rcl_logging_rosout_fini_publisher_for_node(&mut self.remote_node);
                assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize rosout publisher for remote node");
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
            assert_eq!(num_topics, nat.names.size, "Expected {} topics, got {}", num_topics, nat.names.size);
        }
        let ret = rcl_names_and_types_fini(&mut nat);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize names and types");
    }
}

/// Verify subsystem count for both nodes
impl NodeGraphMultiNodeFixture {
    fn verify_subsystem_count(&mut self, node_state: ExpectedNodeState, remote_node_state: ExpectedNodeState) {
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
#[test]
fn test_get_topic_names_and_types() {
    let mut fixture = TestGraphFixture::new();

    unsafe {
        // Test with null node
        let mut names_and_types = rcl_get_zero_initialized_names_and_types();
        let ret = rcl_get_topic_names_and_types(
            ptr::null(),
            ptr::null_mut(),
            false,
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with zero-initialized node
        let zero_node = rcl_get_zero_initialized_node();
        let ret = rcl_get_topic_names_and_types(
            &zero_node,
            ptr::null_mut(),
            false,
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with finalized node
        let ret = rcl_get_topic_names_and_types(
            fixture.old_node(),
            ptr::null_mut(),
            false,
            &mut names_and_types,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with null names_and_types
        let ret = rcl_get_topic_names_and_types(
            fixture.node(),
            ptr::null_mut(),
            false,
            ptr::null_mut(),
        );
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
        let ret = rcl_wait_for_publishers(ptr::null(), &mut allocator, topic_name.as_ptr(), 1, 1000000000, &mut success);
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with null topic_name
        let ret = rcl_wait_for_publishers(fixture.node(), &mut allocator, ptr::null(), 1, 1000000000, &mut success);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test timeout case (no publishers available)
        let ret = rcl_wait_for_publishers(fixture.node(), &mut allocator, topic_name.as_ptr(), 1, 1000000, &mut success); // 1ms timeout
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
        let ret = rcl_wait_for_subscribers(ptr::null(), &mut allocator, topic_name.as_ptr(), 1, 1000000000, &mut success);
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test with null topic_name
        let ret = rcl_wait_for_subscribers(fixture.node(), &mut allocator, ptr::null(), 1, 1000000000, &mut success);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test timeout case (no subscribers available)
        let ret = rcl_wait_for_subscribers(fixture.node(), &mut allocator, topic_name.as_ptr(), 1, 1000000, &mut success); // 1ms timeout
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
        let ret = rcl_subscription_init(&mut sub, fixture.node(), ts, fixture.topic_name.as_ptr().cast(), &sub_ops);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize subscription");

        let mut sub2 = rcl_get_zero_initialized_subscription();
        let sub_ops2 = rcl_subscription_get_default_options();
        let ret = rcl_subscription_init(&mut sub2, fixture.remote_node(), ts, fixture.topic_name.as_ptr().cast(), &sub_ops2);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize remote subscription");

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
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize remote subscription");
        fixture.verify_subsystem_count(
            ExpectedNodeState::new(0, 0, 0, 0),
            ExpectedNodeState::new(0, 0, 0, 0),
        );
    }
}
