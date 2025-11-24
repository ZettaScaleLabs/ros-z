#![allow(clippy::needless_return)]
#![cfg(feature = "test-msgs")]

// NOTE: This file ports graph-related tests from ROS 2 C++ test_graph.cpp.
// Implemented: Error handling tests and valid-call tests for graph query functions.
//
// TODO: Add error handling tests to the following functions (currently only test valid calls):
//   - test_get_topic_names_and_types
//   - test_rcl_get_publisher_names_and_types_by_node
//   - test_rcl_get_subscriber_names_and_types_by_node
//   - test_rcl_get_service_names_and_types_by_node
//   - test_rcl_get_client_names_and_types_by_node
//
// TODO: Implement missing test functions from C++ test_graph.cpp:
//   - test_rcl_names_and_types_init (requires rcl_names_and_types_init function)
//   - test_rcl_wait_for_publishers (requires rcl_wait_for_publishers function)
//   - test_rcl_wait_for_subscribers (requires rcl_wait_for_subscribers function)
//   - test_node_info_subscriptions (multi-node scenario)
//   - test_node_info_publishers (multi-node scenario)
//   - test_node_info_services (multi-node scenario)
//   - test_node_info_clients (multi-node scenario)
//   - test_graph_query_functions (node name queries)
//   - test_graph_guard_condition_trigger_check (graph guard condition)
//   - test_rcl_service_server_is_available (service availability checking)
//   - test_bad_server_available (invalid args for service availability)
//   - test_bad_get_node_names (invalid args for node name queries)
//
// Note: The Rust implementation returns RCL_RET_INVALID_ARGUMENT for invalid nodes
// instead of RCL_RET_NODE_INVALID as the C++ version does. This is a less granular
// but still correct error handling approach.

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
    ros::*,
};

/// Test fixture that provides an initialized RCL context and node
struct TestGraphFixture {
    context: rcl_context_t,
    node: rcl_node_t,
    old_node: rcl_node_t, // A finalized node for testing invalid node scenarios
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

/// Test rcl_get_topic_names_and_types
/// TODO: Add error handling tests (invalid node, null allocator, null names_and_types, etc.)
#[test]
fn test_get_topic_names_and_types() {
    let mut fixture = TestGraphFixture::new();

    // Get topic names and types
    let mut names_and_types = rcl_get_zero_initialized_names_and_types();
    let ret = unsafe {
        rcl_get_topic_names_and_types(
            fixture.node(),
            ptr::null_mut(), // use default allocator
            false,           // no demangle
            &mut names_and_types,
        )
    };
    assert_eq!(
        ret, RCL_RET_OK as i32,
        "Failed to get topic names and types"
    );

    // Finalize
    let ret = unsafe { rcl_names_and_types_fini(&mut names_and_types) };
    assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize names and types");
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
/// TODO: Add error handling tests (invalid node, null allocator, null node_name, null namespace, null names_and_types)
#[test]
fn test_rcl_get_publisher_names_and_types_by_node() {
    let mut fixture = TestGraphFixture::new();

    let mut names_and_types = rcl_get_zero_initialized_names_and_types();
    let node_name = c"test_graph_node";
    let namespace = c"";

    // Valid call
    let ret = rcl_get_publisher_names_and_types_by_node(
        fixture.node(),
        ptr::null_mut(), // use default allocator
        false,           // no demangle
        node_name.as_ptr(),
        namespace.as_ptr(),
        &mut names_and_types,
    );
    assert_eq!(
        ret, RCL_RET_OK as i32,
        "Failed to get publisher names and types by node"
    );

    // Finalize
    let ret = unsafe { rcl_names_and_types_fini(&mut names_and_types) };
    assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize names and types");
}

/// Test rcl_get_subscriber_names_and_types_by_node
/// TODO: Add error handling tests (invalid node, null allocator, null node_name, null namespace, null names_and_types)
#[test]
fn test_rcl_get_subscriber_names_and_types_by_node() {
    let mut fixture = TestGraphFixture::new();

    let mut names_and_types = rcl_get_zero_initialized_names_and_types();
    let node_name = c"test_graph_node";
    let namespace = c"";

    // Valid call
    let ret = rcl_get_subscriber_names_and_types_by_node(
        fixture.node(),
        ptr::null_mut(), // use default allocator
        false,           // no demangle
        node_name.as_ptr(),
        namespace.as_ptr(),
        &mut names_and_types,
    );
    assert_eq!(
        ret, RCL_RET_OK as i32,
        "Failed to get subscriber names and types by node"
    );

    // Finalize
    let ret = unsafe { rcl_names_and_types_fini(&mut names_and_types) };
    assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize names and types");
}

/// Test rcl_get_service_names_and_types_by_node
/// TODO: Add error handling tests (invalid node, null allocator, null node_name, null namespace, null names_and_types)
#[test]
fn test_rcl_get_service_names_and_types_by_node() {
    let mut fixture = TestGraphFixture::new();

    let mut names_and_types = rcl_get_zero_initialized_names_and_types();
    let node_name = c"test_graph_node";
    let namespace = c"";

    // Valid call
    let ret = unsafe {
        rcl_get_service_names_and_types_by_node(
            fixture.node(),
            ptr::null_mut(), // use default allocator
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut names_and_types,
        )
    };
    assert_eq!(
        ret, RCL_RET_OK as i32,
        "Failed to get service names and types by node"
    );

    // Finalize
    let ret = unsafe { rcl_names_and_types_fini(&mut names_and_types) };
    assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize names and types");
}

/// Test rcl_get_client_names_and_types_by_node
/// TODO: Add error handling tests (invalid node, null allocator, null node_name, null namespace, null names_and_types)
#[test]
fn test_rcl_get_client_names_and_types_by_node() {
    let mut fixture = TestGraphFixture::new();

    let mut names_and_types = rcl_get_zero_initialized_names_and_types();
    let node_name = c"test_graph_node";
    let namespace = c"";

    // Valid call
    let ret = rcl_get_client_names_and_types_by_node(
        fixture.node(),
        ptr::null_mut(), // use default allocator
        node_name.as_ptr(),
        namespace.as_ptr(),
        &mut names_and_types,
    );
    assert_eq!(
        ret, RCL_RET_OK as i32,
        "Failed to get client names and types by node"
    );

    // Finalize
    let ret = unsafe { rcl_names_and_types_fini(&mut names_and_types) };
    assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize names and types");
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
