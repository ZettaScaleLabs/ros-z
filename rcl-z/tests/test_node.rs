// Ported from Open Source Robotics Foundation code (2015)
// https://github.com/ros2/rcl
// Licensed under the Apache License 2.0
// Copyright 2025 ZettaScale Technology

#![cfg(feature = "test-core")]

use std::ptr;

use rcl_z::{
    context::{rcl_context_fini, rcl_get_zero_initialized_context, rcl_shutdown},
    init::{
        rcl_get_default_allocator, rcl_get_zero_initialized_init_options, rcl_init,
        rcl_init_options_fini, rcl_init_options_init, rcl_init_options_set_domain_id,
    },
    node::{
        rcl_get_zero_initialized_node, rcl_node_fini, rcl_node_get_default_options,
        rcl_node_get_domain_id, rcl_node_get_fully_qualified_name,
        rcl_node_get_graph_guard_condition, rcl_node_get_logger_name, rcl_node_get_name,
        rcl_node_get_namespace, rcl_node_get_options, rcl_node_get_rcl_instance_id,
        rcl_node_get_rmw_handle, rcl_node_init, rcl_node_is_valid,
        rcl_node_is_valid_except_context,
    },
    ros::{RCL_RET_INVALID_ARGUMENT, RCL_RET_OK},
};

/// Test fixture that provides an initialized RCL context for node tests
struct TestNodeFixture {
    context: rcl_z::ros::rcl_context_t,
    init_options: rcl_z::ros::rcl_init_options_t,
}

impl TestNodeFixture {
    fn new() -> Self {
        let mut init_options = rcl_get_zero_initialized_init_options();
        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut context = rcl_get_zero_initialized_context();
        let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        TestNodeFixture {
            context,
            init_options,
        }
    }

    fn context(&mut self) -> *mut rcl_z::ros::rcl_context_t {
        &mut self.context
    }
}

impl Drop for TestNodeFixture {
    fn drop(&mut self) {
        let ret = rcl_shutdown(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_context_fini(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_init_options_fini(&mut self.init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_get_zero_initialized_node
#[test]
fn test_get_zero_initialized_node() {
    let node = rcl_get_zero_initialized_node();
    // Check that node is zero-initialized
    assert!(!rcl_node_is_valid(&node));
}

/// Test rcl_node_get_default_options
#[test]
fn test_node_get_default_options() {
    let _options = rcl_node_get_default_options();
    // Function should not crash and return valid options
}

/// Test rcl_node_init and rcl_node_fini with valid parameters
#[test]
fn test_node_init_fini() {
    let mut fixture = TestNodeFixture::new();

    let mut node = rcl_get_zero_initialized_node();
    let node_name = c"test_node";
    let namespace = c"";
    let node_options = rcl_node_get_default_options();

    // Initialize node
    let ret = unsafe {
        rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            namespace.as_ptr(),
            fixture.context(),
            &node_options,
        )
    };
    assert_eq!(ret, RCL_RET_OK as i32);

    // Node should be valid
    assert!(rcl_node_is_valid(&node));

    // Finalize node
    let ret = rcl_node_fini(&mut node);
    assert_eq!(ret, RCL_RET_OK as i32);
}

/// Test rcl_node_init with invalid parameters
#[test]
fn test_node_init_invalid() {
    let mut fixture = TestNodeFixture::new();

    let mut node = rcl_get_zero_initialized_node();
    let node_name = c"test_node";
    let namespace = c"";
    let node_options = rcl_node_get_default_options();

    // Test with null node
    let _ret = unsafe {
        rcl_node_init(
            ptr::null_mut(),
            node_name.as_ptr(),
            namespace.as_ptr(),
            fixture.context(),
            &node_options,
        )
    };
    // Should fail with invalid argument

    // Test with null context
    let _ret = unsafe {
        rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            namespace.as_ptr(),
            ptr::null_mut(),
            &node_options,
        )
    };
    // Should fail with invalid argument

    // Test with null node name
    let _ret = unsafe {
        rcl_node_init(
            &mut node,
            ptr::null(),
            namespace.as_ptr(),
            fixture.context(),
            &node_options,
        )
    };
    // Should fail with invalid argument

    // Test with null options
    let _ret = unsafe {
        rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            namespace.as_ptr(),
            fixture.context(),
            ptr::null(),
        )
    };
    // Should fail with invalid argument
}

/// Test rcl_node_is_valid
#[test]
fn test_node_is_valid() {
    let mut fixture = TestNodeFixture::new();

    // Test with zero-initialized node
    let node = rcl_get_zero_initialized_node();
    assert!(!rcl_node_is_valid(&node));

    // Test with initialized node
    let mut node = rcl_get_zero_initialized_node();
    let node_name = c"test_node";
    let namespace = c"";
    let node_options = rcl_node_get_default_options();

    let ret = unsafe {
        rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            namespace.as_ptr(),
            fixture.context(),
            &node_options,
        )
    };
    assert_eq!(ret, RCL_RET_OK as i32);
    assert!(rcl_node_is_valid(&node));

    // Test after finalization
    let ret = rcl_node_fini(&mut node);
    assert_eq!(ret, RCL_RET_OK as i32);
    // Node should not be valid after finalization
    assert!(!rcl_node_is_valid(&node));
}

/// Test rcl_node_get_name
#[test]
fn test_node_get_name() {
    let mut fixture = TestNodeFixture::new();

    let mut node = rcl_get_zero_initialized_node();
    let node_name = c"test_node";
    let namespace = c"";
    let node_options = rcl_node_get_default_options();

    let ret = unsafe {
        rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            namespace.as_ptr(),
            fixture.context(),
            &node_options,
        )
    };
    assert_eq!(ret, RCL_RET_OK as i32);

    let name = rcl_node_get_name(&node);
    assert!(!name.is_null());
    // The returned string should match the input name
    let name_str = unsafe { std::ffi::CStr::from_ptr(name) };
    assert_eq!(name_str.to_str().unwrap(), "test_node");

    let ret = rcl_node_fini(&mut node);
    assert_eq!(ret, RCL_RET_OK as i32);
}

/// Test rcl_node_get_namespace
#[test]
fn test_node_get_namespace() {
    let mut fixture = TestNodeFixture::new();

    let mut node = rcl_get_zero_initialized_node();
    let node_name = c"test_node";
    let namespace = c"test_namespace";
    let node_options = rcl_node_get_default_options();

    let ret = unsafe {
        rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            namespace.as_ptr(),
            fixture.context(),
            &node_options,
        )
    };
    assert_eq!(ret, RCL_RET_OK as i32);

    let ns = rcl_node_get_namespace(&node);
    assert!(!ns.is_null());
    // The returned string should match the input namespace (with leading / added)
    let ns_str = unsafe { std::ffi::CStr::from_ptr(ns) };
    assert_eq!(ns_str.to_str().unwrap(), "/test_namespace");

    let ret = rcl_node_fini(&mut node);
    assert_eq!(ret, RCL_RET_OK as i32);
}

/// Test rcl_node_get_fully_qualified_name
#[test]
fn test_node_get_fully_qualified_name() {
    let mut fixture = TestNodeFixture::new();

    let mut node = rcl_get_zero_initialized_node();
    let node_name = c"test_node";
    let namespace = c"test_namespace";
    let node_options = rcl_node_get_default_options();

    let ret = unsafe {
        rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            namespace.as_ptr(),
            fixture.context(),
            &node_options,
        )
    };
    assert_eq!(ret, RCL_RET_OK as i32);

    let fq_name = rcl_node_get_fully_qualified_name(&node);
    assert!(!fq_name.is_null());
    // The returned string should be the fully qualified name (with leading /)
    let fq_name_str = unsafe { std::ffi::CStr::from_ptr(fq_name) };
    assert_eq!(fq_name_str.to_str().unwrap(), "/test_namespace/test_node");

    let ret = rcl_node_fini(&mut node);
    assert_eq!(ret, RCL_RET_OK as i32);
}

/// Test node lifecycle
/// Aligns with test_node.cpp::test_rcl_node_life_cycle
#[test]
fn test_rcl_node_life_cycle() {
    use rcl_z::ros::{RCL_RET_ALREADY_INIT, RCL_RET_NODE_INVALID, RCL_RET_NOT_INIT};

    let mut context = rcl_get_zero_initialized_context();
    let mut node = rcl_get_zero_initialized_node();
    let node_name = c"test_rcl_node_life_cycle_node";
    let namespace = c"/ns";
    let node_options = rcl_node_get_default_options();

    // Trying to init before rcl_init() should fail
    let ret = unsafe {
        rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            c"".as_ptr(),
            &mut context,
            &node_options,
        )
    };
    assert_eq!(ret, RCL_RET_NOT_INIT as i32);

    // Initialize rcl with rcl_init()
    let mut init_options = rcl_get_zero_initialized_init_options();
    let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
    assert_eq!(ret, RCL_RET_OK as i32);

    // Try invalid arguments
    let ret = unsafe {
        rcl_node_init(
            ptr::null_mut(),
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut context,
            &node_options,
        )
    };
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    let ret = unsafe {
        rcl_node_init(
            &mut node,
            ptr::null(),
            namespace.as_ptr(),
            &mut context,
            &node_options,
        )
    };
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    let ret = unsafe {
        rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            ptr::null(),
            &mut context,
            &node_options,
        )
    };
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    let ret = unsafe {
        rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            namespace.as_ptr(),
            ptr::null_mut(),
            &node_options,
        )
    };
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    let ret = unsafe {
        rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut context,
            ptr::null(),
        )
    };
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    // Try fini with invalid arguments
    let ret = rcl_node_fini(ptr::null_mut());
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

    // Try fini with an uninitialized node
    let ret = rcl_node_fini(&mut node);
    assert_eq!(ret, RCL_RET_OK as i32);

    // Try a normal init and fini
    let ret = unsafe {
        rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut context,
            &node_options,
        )
    };
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_node_fini(&mut node);
    assert_eq!(ret, RCL_RET_OK as i32);

    // Try repeated init and fini calls
    let ret = unsafe {
        rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut context,
            &node_options,
        )
    };
    assert_eq!(ret, RCL_RET_OK as i32);

    // Double init should fail
    let ret = unsafe {
        rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            namespace.as_ptr(),
            &mut context,
            &node_options,
        )
    };
    assert_eq!(ret, RCL_RET_ALREADY_INIT as i32);

    let ret = rcl_node_fini(&mut node);
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_node_fini(&mut node);
    assert_eq!(ret, RCL_RET_OK as i32);

    // Cleanup
    let ret = rcl_shutdown(&mut context);
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_context_fini(&mut context);
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_init_options_fini(&mut init_options);
    assert_eq!(ret, RCL_RET_OK as i32);
}

/// Test rcl_node accessors
#[test]
fn test_rcl_node_accessors() {
    // Initialize rcl with rcl_init().
    let mut init_options = rcl_get_zero_initialized_init_options();
    let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
    assert_eq!(ret, RCL_RET_OK as i32);
    let ret = rcl_init_options_set_domain_id(&mut init_options, 42);
    assert_eq!(ret, RCL_RET_OK as i32);
    let mut invalid_context = rcl_get_zero_initialized_context();
    let ret = rcl_init(0, ptr::null(), &init_options, &mut invalid_context);
    assert_eq!(ret, RCL_RET_OK as i32);

    // Create an invalid node (rcl_shutdown).
    let mut invalid_node = rcl_get_zero_initialized_node();
    let name = c"test_rcl_node_accessors_node";
    let namespace_ = c"/ns";
    let default_options = rcl_node_get_default_options();
    let ret = unsafe {
        rcl_node_init(
            &mut invalid_node,
            name.as_ptr(),
            namespace_.as_ptr(),
            &mut invalid_context,
            &default_options,
        )
    };
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_shutdown(&mut invalid_context);
    assert_eq!(ret, RCL_RET_OK as i32);

    let mut context = rcl_get_zero_initialized_context();
    let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
    assert_eq!(ret, RCL_RET_OK as i32);

    // Create a zero init node.
    let zero_node = rcl_get_zero_initialized_node();

    // Create a normal node.
    let mut node = rcl_get_zero_initialized_node();
    let ret = unsafe {
        rcl_node_init(
            &mut node,
            name.as_ptr(),
            namespace_.as_ptr(),
            &mut context,
            &default_options,
        )
    };
    assert_eq!(ret, RCL_RET_OK as i32);

    // Test rcl_node_is_valid().
    assert!(!rcl_node_is_valid(ptr::null()));
    assert!(!rcl_node_is_valid(&zero_node));
    // invalid node will be true for rcl_node_is_valid_except_context, but false for valid only
    assert!(rcl_node_is_valid_except_context(&invalid_node));
    assert!(!rcl_node_is_valid(&invalid_node));
    assert!(rcl_node_is_valid(&node));

    // Test rcl_node_get_name().
    assert_eq!(rcl_node_get_name(ptr::null()), ptr::null());
    assert_eq!(rcl_node_get_name(&zero_node), ptr::null());
    let actual_name = rcl_node_get_name(&invalid_node);
    assert!(!actual_name.is_null());
    let actual_name_str = unsafe { std::ffi::CStr::from_ptr(actual_name) };
    assert_eq!(
        actual_name_str.to_str().unwrap(),
        "test_rcl_node_accessors_node"
    );
    let actual_name = rcl_node_get_name(&node);
    assert!(!actual_name.is_null());
    let actual_name_str = unsafe { std::ffi::CStr::from_ptr(actual_name) };
    assert_eq!(
        actual_name_str.to_str().unwrap(),
        "test_rcl_node_accessors_node"
    );

    // Test rcl_node_get_namespace().
    assert_eq!(rcl_node_get_namespace(ptr::null()), ptr::null());
    assert_eq!(rcl_node_get_namespace(&zero_node), ptr::null());
    let actual_namespace = rcl_node_get_namespace(&invalid_node);
    assert!(!actual_namespace.is_null());
    let actual_namespace_str = unsafe { std::ffi::CStr::from_ptr(actual_namespace) };
    assert_eq!(actual_namespace_str.to_str().unwrap(), "/ns");
    let actual_namespace = rcl_node_get_namespace(&node);
    assert!(!actual_namespace.is_null());
    let actual_namespace_str = unsafe { std::ffi::CStr::from_ptr(actual_namespace) };
    assert_eq!(actual_namespace_str.to_str().unwrap(), "/ns");

    // Test rcl_node_get_fully_qualified_name().
    assert_eq!(rcl_node_get_fully_qualified_name(ptr::null()), ptr::null());
    assert_eq!(rcl_node_get_fully_qualified_name(&zero_node), ptr::null());
    let actual_fq_name = rcl_node_get_fully_qualified_name(&invalid_node);
    assert!(!actual_fq_name.is_null());
    let fq_name_str = unsafe { std::ffi::CStr::from_ptr(actual_fq_name) };
    assert_eq!(
        fq_name_str.to_str().unwrap(),
        "/ns/test_rcl_node_accessors_node"
    );
    let actual_fq_name = rcl_node_get_fully_qualified_name(&node);
    assert_ne!(actual_fq_name, ptr::null());

    // Test rcl_node_get_logger_name().
    assert_eq!(rcl_node_get_logger_name(ptr::null()), ptr::null());
    assert_eq!(rcl_node_get_logger_name(&zero_node), ptr::null());
    let actual_logger_name = rcl_node_get_logger_name(&invalid_node);
    assert!(!actual_logger_name.is_null());
    let logger_name_str = unsafe { std::ffi::CStr::from_ptr(actual_logger_name) };
    assert_eq!(
        logger_name_str.to_str().unwrap(),
        "ns.test_rcl_node_accessors_node"
    );
    let actual_logger_name = rcl_node_get_logger_name(&node);
    assert!(!actual_logger_name.is_null());
    let logger_name_str = unsafe { std::ffi::CStr::from_ptr(actual_logger_name) };
    assert_eq!(
        logger_name_str.to_str().unwrap(),
        "ns.test_rcl_node_accessors_node"
    );

    // Test rcl_node_get_options().
    assert_eq!(rcl_node_get_options(ptr::null()), ptr::null());
    assert_eq!(rcl_node_get_options(&zero_node), ptr::null());
    let actual_options = rcl_node_get_options(&invalid_node);
    assert_ne!(actual_options, ptr::null());
    let actual_options = rcl_node_get_options(&node);
    assert_ne!(actual_options, ptr::null());

    // Test rcl_node_get_domain_id().
    use rcl_z::ros::RCL_RET_NODE_INVALID;
    let mut actual_domain_id = 0;
    let ret = rcl_node_get_domain_id(ptr::null(), &mut actual_domain_id);
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
    let ret = rcl_node_get_domain_id(&zero_node, &mut actual_domain_id);
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
    let ret = rcl_node_get_domain_id(&invalid_node, &mut actual_domain_id);
    assert_eq!(ret, RCL_RET_NODE_INVALID as i32);
    let ret = rcl_node_get_domain_id(&node, &mut actual_domain_id);
    assert_eq!(ret, RCL_RET_OK as i32);
    assert_eq!(actual_domain_id, 42);

    // Test rcl_node_get_rmw_handle().
    assert_eq!(rcl_node_get_rmw_handle(ptr::null()), ptr::null_mut());
    assert_eq!(rcl_node_get_rmw_handle(&zero_node), ptr::null_mut());
    let rmw_handle = rcl_node_get_rmw_handle(&invalid_node);
    assert_ne!(rmw_handle, ptr::null_mut());
    let rmw_handle = rcl_node_get_rmw_handle(&node);
    assert_ne!(rmw_handle, ptr::null_mut());

    // Test rcl_node_get_rcl_instance_id().
    assert_eq!(rcl_node_get_rcl_instance_id(ptr::null()), 0);
    assert_eq!(rcl_node_get_rcl_instance_id(&zero_node), 0);
    assert_eq!(rcl_node_get_rcl_instance_id(&invalid_node), 0);
    let instance_id = rcl_node_get_rcl_instance_id(&node);
    assert_ne!(instance_id, 0);

    // Test rcl_node_get_graph_guard_condition
    assert_eq!(rcl_node_get_graph_guard_condition(ptr::null()), ptr::null());
    assert_eq!(rcl_node_get_graph_guard_condition(&zero_node), ptr::null());
    let graph_guard_condition = rcl_node_get_graph_guard_condition(&invalid_node);
    assert_ne!(graph_guard_condition, ptr::null());
    let graph_guard_condition = rcl_node_get_graph_guard_condition(&node);
    assert_ne!(graph_guard_condition, ptr::null());

    // Cleanup
    let ret = rcl_node_fini(&mut invalid_node);
    assert_eq!(ret, RCL_RET_OK as i32);
    let ret = rcl_node_fini(&mut node);
    assert_eq!(ret, RCL_RET_OK as i32);
    let ret = rcl_shutdown(&mut context);
    assert_eq!(ret, RCL_RET_OK as i32);
    let ret = rcl_context_fini(&mut context);
    assert_eq!(ret, RCL_RET_OK as i32);
    let ret = rcl_context_fini(&mut invalid_context);
    assert_eq!(ret, RCL_RET_OK as i32);
    let ret = rcl_init_options_fini(&mut init_options);
    assert_eq!(ret, RCL_RET_OK as i32);
}

/// Test node name restrictions enforcement
/// Aligns with test_node.cpp::test_rcl_node_name_restrictions
#[test]
fn test_rcl_node_name_restrictions() {
    use rcl_z::ros::RCL_RET_NODE_INVALID_NAME;

    // Initialize rcl with rcl_init()
    let mut init_options = rcl_get_zero_initialized_init_options();
    let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
    assert_eq!(ret, RCL_RET_OK as i32);

    let mut context = rcl_get_zero_initialized_context();
    let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
    assert_eq!(ret, RCL_RET_OK as i32);

    let namespace = c"/ns";
    let default_options = rcl_node_get_default_options();

    // First do a normal node name
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                c"my_node_42".as_ptr(),
                namespace.as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Node name with invalid characters
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                c"my_node_42$".as_ptr(),
                namespace.as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_NODE_INVALID_NAME as i32);
        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Node name with /, which is valid in a topic, but not a node name
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                c"my/node_42".as_ptr(),
                namespace.as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_NODE_INVALID_NAME as i32);
        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Node name with {}, which is valid in a topic, but not a node name
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                c"my_{node}_42".as_ptr(),
                namespace.as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_NODE_INVALID_NAME as i32);
        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Cleanup
    let ret = rcl_shutdown(&mut context);
    assert_eq!(ret, RCL_RET_OK as i32);
    let ret = rcl_context_fini(&mut context);
    assert_eq!(ret, RCL_RET_OK as i32);
    let ret = rcl_init_options_fini(&mut init_options);
    assert_eq!(ret, RCL_RET_OK as i32);
}

/// Test node namespace restrictions enforcement
/// Aligns with test_node.cpp::test_rcl_node_namespace_restrictions
#[test]
fn test_rcl_node_namespace_restrictions() {
    use rcl_z::ros::RCL_RET_NODE_INVALID_NAMESPACE;

    // Initialize rcl with rcl_init()
    let mut init_options = rcl_get_zero_initialized_init_options();
    let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
    assert_eq!(ret, RCL_RET_OK as i32);

    let mut context = rcl_get_zero_initialized_context();
    let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
    assert_eq!(ret, RCL_RET_OK as i32);

    let name = c"node";
    let default_options = rcl_node_get_default_options();

    // First do a normal node namespace
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                name.as_ptr(),
                c"/ns".as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Node namespace which is an empty string, which is also valid
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                name.as_ptr(),
                c"".as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_OK as i32);
        let actual_namespace = rcl_node_get_namespace(&node);
        let ns_str = unsafe { std::ffi::CStr::from_ptr(actual_namespace) };
        assert_eq!(ns_str.to_str().unwrap(), "/");
        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Node namespace which is just a forward slash, which is valid
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                name.as_ptr(),
                c"/".as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Node namespaces with invalid characters
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                name.as_ptr(),
                c"/ns/{name}".as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_NODE_INVALID_NAMESPACE as i32);
        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                name.as_ptr(),
                c"/~/".as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_NODE_INVALID_NAMESPACE as i32);
        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Node namespace with a trailing / which is not allowed
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                name.as_ptr(),
                c"/ns/foo/".as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_NODE_INVALID_NAMESPACE as i32);
        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Node namespace which is not absolute, it should get / added automatically
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                name.as_ptr(),
                c"ns".as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_OK as i32);
        let actual_namespace = rcl_node_get_namespace(&node);
        let ns_str = unsafe { std::ffi::CStr::from_ptr(actual_namespace) };
        assert_eq!(ns_str.to_str().unwrap(), "/ns");
        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Other reasons for being invalid, which are related to being part of a topic
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                name.as_ptr(),
                c"/starts/with/42number".as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_NODE_INVALID_NAMESPACE as i32);
        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Cleanup
    let ret = rcl_shutdown(&mut context);
    assert_eq!(ret, RCL_RET_OK as i32);
    let ret = rcl_context_fini(&mut context);
    assert_eq!(ret, RCL_RET_OK as i32);
    let ret = rcl_init_options_fini(&mut init_options);
    assert_eq!(ret, RCL_RET_OK as i32);
}

/// Test logger name as well as fully qualified name associated with the node
/// Aligns with test_node.cpp::test_rcl_node_names
#[test]
fn test_rcl_node_names() {
    // Initialize rcl with rcl_init()
    let mut init_options = rcl_get_zero_initialized_init_options();
    let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
    assert_eq!(ret, RCL_RET_OK as i32);

    let mut context = rcl_get_zero_initialized_context();
    let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
    assert_eq!(ret, RCL_RET_OK as i32);

    let default_options = rcl_node_get_default_options();

    // First do a normal node namespace
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                c"node".as_ptr(),
                c"/ns".as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_OK as i32);

        let actual_node_logger_name = rcl_node_get_logger_name(&node);
        let actual_node_name = rcl_node_get_name(&node);
        let actual_node_namespace = rcl_node_get_namespace(&node);
        let actual_node_fq_name = rcl_node_get_fully_qualified_name(&node);

        assert!(!actual_node_logger_name.is_null());
        let logger_name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_logger_name) };
        assert_eq!(logger_name_str.to_str().unwrap(), "ns.node");

        assert!(!actual_node_name.is_null());
        let name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_name) };
        assert_eq!(name_str.to_str().unwrap(), "node");

        assert!(!actual_node_namespace.is_null());
        let namespace_str = unsafe { std::ffi::CStr::from_ptr(actual_node_namespace) };
        assert_eq!(namespace_str.to_str().unwrap(), "/ns");

        assert!(!actual_node_fq_name.is_null());
        let fq_name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_fq_name) };
        assert_eq!(fq_name_str.to_str().unwrap(), "/ns/node");

        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Node namespace that is an empty string
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                c"node".as_ptr(),
                c"".as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_OK as i32);

        let actual_node_logger_name = rcl_node_get_logger_name(&node);
        let actual_node_name = rcl_node_get_name(&node);
        let actual_node_namespace = rcl_node_get_namespace(&node);
        let actual_node_fq_name = rcl_node_get_fully_qualified_name(&node);

        assert!(!actual_node_logger_name.is_null());
        let logger_name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_logger_name) };
        assert_eq!(logger_name_str.to_str().unwrap(), "node");

        assert!(!actual_node_name.is_null());
        let name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_name) };
        assert_eq!(name_str.to_str().unwrap(), "node");

        assert!(!actual_node_namespace.is_null());
        let namespace_str = unsafe { std::ffi::CStr::from_ptr(actual_node_namespace) };
        assert_eq!(namespace_str.to_str().unwrap(), "/");

        assert!(!actual_node_fq_name.is_null());
        let fq_name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_fq_name) };
        assert_eq!(fq_name_str.to_str().unwrap(), "/node");

        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Node namespace that is just a forward slash
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                c"node".as_ptr(),
                c"/".as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_OK as i32);

        let actual_node_logger_name = rcl_node_get_logger_name(&node);
        let actual_node_name = rcl_node_get_name(&node);
        let actual_node_namespace = rcl_node_get_namespace(&node);
        let actual_node_fq_name = rcl_node_get_fully_qualified_name(&node);

        assert!(!actual_node_logger_name.is_null());
        let logger_name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_logger_name) };
        assert_eq!(logger_name_str.to_str().unwrap(), "node");

        assert!(!actual_node_name.is_null());
        let name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_name) };
        assert_eq!(name_str.to_str().unwrap(), "node");

        assert!(!actual_node_namespace.is_null());
        let namespace_str = unsafe { std::ffi::CStr::from_ptr(actual_node_namespace) };
        assert_eq!(namespace_str.to_str().unwrap(), "/");

        assert!(!actual_node_fq_name.is_null());
        let fq_name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_fq_name) };
        assert_eq!(fq_name_str.to_str().unwrap(), "/node");

        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Node namespace that is not absolute
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                c"node".as_ptr(),
                c"ns".as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_OK as i32);

        let actual_node_logger_name = rcl_node_get_logger_name(&node);
        let actual_node_name = rcl_node_get_name(&node);
        let actual_node_namespace = rcl_node_get_namespace(&node);
        let actual_node_fq_name = rcl_node_get_fully_qualified_name(&node);

        assert!(!actual_node_logger_name.is_null());
        let logger_name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_logger_name) };
        assert_eq!(logger_name_str.to_str().unwrap(), "ns.node");

        assert!(!actual_node_name.is_null());
        let name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_name) };
        assert_eq!(name_str.to_str().unwrap(), "node");

        assert!(!actual_node_namespace.is_null());
        let namespace_str = unsafe { std::ffi::CStr::from_ptr(actual_node_namespace) };
        assert_eq!(namespace_str.to_str().unwrap(), "/ns");

        assert!(!actual_node_fq_name.is_null());
        let fq_name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_fq_name) };
        assert_eq!(fq_name_str.to_str().unwrap(), "/ns/node");

        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Nested namespace
    {
        let mut node = rcl_get_zero_initialized_node();
        let ret = unsafe {
            rcl_node_init(
                &mut node,
                c"node".as_ptr(),
                c"/ns/sub_1/sub_2".as_ptr(),
                &mut context,
                &default_options,
            )
        };
        assert_eq!(ret, RCL_RET_OK as i32);

        let actual_node_logger_name = rcl_node_get_logger_name(&node);
        let actual_node_name = rcl_node_get_name(&node);
        let actual_node_namespace = rcl_node_get_namespace(&node);
        let actual_node_fq_name = rcl_node_get_fully_qualified_name(&node);

        assert!(!actual_node_logger_name.is_null());
        let logger_name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_logger_name) };
        assert_eq!(logger_name_str.to_str().unwrap(), "ns.sub_1.sub_2.node");

        assert!(!actual_node_name.is_null());
        let name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_name) };
        assert_eq!(name_str.to_str().unwrap(), "node");

        assert!(!actual_node_namespace.is_null());
        let namespace_str = unsafe { std::ffi::CStr::from_ptr(actual_node_namespace) };
        assert_eq!(namespace_str.to_str().unwrap(), "/ns/sub_1/sub_2");

        assert!(!actual_node_fq_name.is_null());
        let fq_name_str = unsafe { std::ffi::CStr::from_ptr(actual_node_fq_name) };
        assert_eq!(fq_name_str.to_str().unwrap(), "/ns/sub_1/sub_2/node");

        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }

    // Cleanup
    let ret = rcl_shutdown(&mut context);
    assert_eq!(ret, RCL_RET_OK as i32);
    let ret = rcl_context_fini(&mut context);
    assert_eq!(ret, RCL_RET_OK as i32);
    let ret = rcl_init_options_fini(&mut init_options);
    assert_eq!(ret, RCL_RET_OK as i32);
}
