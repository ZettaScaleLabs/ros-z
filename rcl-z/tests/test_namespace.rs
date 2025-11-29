// Copyright 2025 ZettaScale Technology
// SPDX-License-Identifier: Apache-2.0
//
// Ported from ros2/rcl:
// Copyright 2017 Open Source Robotics Foundation, Inc.

#![cfg(feature = "test-msgs")]

use std::{ptr, thread, time::Duration};

use rcl_z::{
    context::{rcl_context_fini, rcl_get_zero_initialized_context, rcl_shutdown},
    init::{
        rcl_get_default_allocator, rcl_get_zero_initialized_init_options, rcl_init,
        rcl_init_options_fini, rcl_init_options_init,
    },
    node::{
        rcl_get_zero_initialized_node, rcl_node_fini, rcl_node_get_default_options, rcl_node_init,
    },
    ros::*,
    service::{
        rcl_client_fini, rcl_client_get_default_options, rcl_client_init,
        rcl_get_zero_initialized_client, rcl_get_zero_initialized_service, rcl_service_fini,
        rcl_service_get_default_options, rcl_service_init, rcl_service_server_is_available,
    },
};

mod test_msgs_support;

/// Test fixture that provides an initialized RCL context and node
struct TestNamespaceFixture {
    context: rcl_context_t,
    init_options: rcl_init_options_t,
    node: rcl_node_t,
}

impl TestNamespaceFixture {
    fn new() -> Self {
        unsafe {
            // Initialize init options
            let mut init_options = rcl_get_zero_initialized_init_options();
            let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize init options");

            // Initialize context
            let mut context = rcl_get_zero_initialized_context();
            let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
            assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize context");

            // Initialize node
            let mut node = rcl_get_zero_initialized_node();
            let node_name = c"test_namespace_node";
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

            TestNamespaceFixture {
                context,
                init_options,
                node,
            }
        }
    }

    fn node(&mut self) -> *mut rcl_node_t {
        &mut self.node
    }
}

impl Drop for TestNamespaceFixture {
    fn drop(&mut self) {
        let ret = rcl_node_fini(&mut self.node);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize node");

        let ret = rcl_shutdown(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to shutdown context");

        let ret = rcl_context_fini(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize context");

        let ret = rcl_init_options_fini(&mut self.init_options);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize init options");
    }
}

/// Test that clients and services in different namespaces don't match,
/// and that clients in the same namespace do match.
#[test]
fn test_client_server() {
    let mut fixture = TestNamespaceFixture::new();

    unsafe {
        let ts = ROSIDL_GET_SRV_TYPE_SUPPORT!(test_msgs, srv, BasicTypes);
        let service_name = c"/my/namespace/test_namespace_client_server";
        let unmatched_client_name = c"/your/namespace/test_namespace_client_server";
        let matched_client_name = c"/my/namespace/test_namespace_client_server";
        const SUCCESS_TIMEOUT: u64 = 10;
        const FAIL_TIMEOUT: u64 = 1;

        // Create service
        let mut service = rcl_get_zero_initialized_service();
        let service_options = rcl_service_get_default_options();
        let ret = rcl_service_init(
            &mut service,
            fixture.node(),
            ts,
            service_name.as_ptr(),
            &service_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize service");

        // Create unmatched client (different namespace)
        let mut unmatched_client = rcl_get_zero_initialized_client();
        let unmatched_client_options = rcl_client_get_default_options();
        let ret = rcl_client_init(
            &mut unmatched_client,
            fixture.node(),
            ts,
            unmatched_client_name.as_ptr(),
            &unmatched_client_options,
        );
        assert_eq!(
            ret, RCL_RET_OK as i32,
            "Failed to initialize unmatched client"
        );

        // Wait and check that unmatched client does NOT find the service
        let mut is_available = false;
        for _ in 0..FAIL_TIMEOUT {
            let ret = rcl_service_server_is_available(
                fixture.node(),
                &unmatched_client,
                &mut is_available,
            );
            assert_eq!(
                ret, RCL_RET_OK as i32,
                "Failed to check service availability"
            );
            if is_available {
                // This should not happen
                break;
            }
            thread::sleep(Duration::from_secs(1));
        }
        assert!(!is_available, "Unmatched client should not find service");

        // Create matched client (same namespace)
        let mut matched_client = rcl_get_zero_initialized_client();
        let matched_client_options = rcl_client_get_default_options();
        let ret = rcl_client_init(
            &mut matched_client,
            fixture.node(),
            ts,
            matched_client_name.as_ptr(),
            &matched_client_options,
        );
        assert_eq!(
            ret, RCL_RET_OK as i32,
            "Failed to initialize matched client"
        );

        // Wait and check that matched client DOES find the service
        is_available = false;
        for _ in 0..SUCCESS_TIMEOUT {
            let ret =
                rcl_service_server_is_available(fixture.node(), &matched_client, &mut is_available);
            assert_eq!(
                ret, RCL_RET_OK as i32,
                "Failed to check service availability"
            );
            if is_available {
                break;
            }
            thread::sleep(Duration::from_secs(1));
        }
        assert!(is_available, "Matched client should find service");

        // Cleanup
        let ret = rcl_client_fini(&mut matched_client, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize matched client");

        let ret = rcl_client_fini(&mut unmatched_client, fixture.node());
        assert_eq!(
            ret, RCL_RET_OK as i32,
            "Failed to finalize unmatched client"
        );

        let ret = rcl_service_fini(&mut service, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize service");
    }
}
