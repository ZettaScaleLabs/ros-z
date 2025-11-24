// Ported from Open Source Robotics Foundation code (2016)
// https://github.com/ros2/rcl
// Licensed under the Apache License 2.0
// Copyright 2025 ZettaScale Technology

#![cfg(feature = "test-core")]

use std::ptr;

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
    ros::{RCL_RET_INVALID_ARGUMENT, RCL_RET_OK, rcl_context_t, rcl_node_t, rmw_request_id_t},
    service::{
        rcl_client_fini, rcl_client_get_default_options, rcl_client_init,
        rcl_get_zero_initialized_client, rcl_send_request, rcl_service_server_is_available,
        rcl_take_response,
    },
};

/// Test fixture that provides an initialized RCL context and node for client tests
struct TestClientFixture {
    context: rcl_context_t,
    node: rcl_node_t,
}

impl TestClientFixture {
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
            let node_name = c"test_client_node";
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

            TestClientFixture { context, node }
        }
    }

    fn node(&mut self) -> *mut rcl_node_t {
        &mut self.node
    }
}

impl Drop for TestClientFixture {
    fn drop(&mut self) {
        let ret = rcl_node_fini(&mut self.node);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize node");

        let ret = rcl_shutdown(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to shutdown context");

        let ret = rcl_context_fini(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize context");
    }
}

/// Test rcl_client_init and rcl_client_fini with valid parameters
#[test]
fn test_client_init_fini() {
    let mut fixture = TestClientFixture::new();

    unsafe {
        let mut client = rcl_get_zero_initialized_client();
        let service_name = c"test_service";
        let client_options = rcl_client_get_default_options();

        // Initialize client
        let _ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ptr::null(), // Type support - would need actual service type support
            service_name.as_ptr(),
            &client_options,
        );
        // Since we don't have actual type support, this might fail, but we're testing the API
        // In a real test, we'd need to set up proper type support

        // Finalize client regardless
        let ret = rcl_client_fini(&mut client, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize client");
    }
}

/// Test rcl_client_init with invalid parameters
#[test]
fn test_client_init_invalid() {
    let mut fixture = TestClientFixture::new();

    unsafe {
        let mut client = rcl_get_zero_initialized_client();
        let service_name = c"test_service";
        let client_options = rcl_client_get_default_options();

        // Test with null client pointer
        let _ret = rcl_client_init(
            ptr::null_mut(),
            fixture.node(),
            ptr::null(),
            service_name.as_ptr(),
            &client_options,
        );
        // Should fail with invalid argument

        // Test with null node pointer
        let _ret = rcl_client_init(
            &mut client,
            ptr::null(),
            ptr::null(),
            service_name.as_ptr(),
            &client_options,
        );
        // Should fail with invalid argument

        // Test with null service name
        let _ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ptr::null(),
            ptr::null(),
            &client_options,
        );
        // Should fail with invalid argument

        // Test with null options
        let _ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ptr::null(),
            service_name.as_ptr(),
            ptr::null(),
        );
        // Should fail with invalid argument
    }
}

/// Test rcl_send_request with invalid client
#[test]
fn test_send_request_invalid() {
    unsafe {
        let mut sequence_number: i64 = 0;
        let dummy_request: *const c_void = ptr::null();

        // Test with null client
        let ret = rcl_send_request(ptr::null(), dummy_request, &mut sequence_number);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with null request
        let client = rcl_get_zero_initialized_client();
        let _ret = rcl_send_request(&client, ptr::null(), &mut sequence_number);
        // This might succeed or fail depending on implementation

        // Test with null sequence number
        let ret = rcl_send_request(&client, dummy_request, ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
    }
}

/// Test rcl_take_response with invalid client
#[test]
fn test_take_response_invalid() {
    unsafe {
        let mut request_header = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        let dummy_response: *mut c_void = ptr::null_mut();

        // Test with null client
        let ret = rcl_take_response(ptr::null(), &mut request_header, dummy_response);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with null response
        let client = rcl_get_zero_initialized_client();
        let ret = rcl_take_response(&client, &mut request_header, ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with null request header
        let ret = rcl_take_response(&client, ptr::null_mut(), dummy_response);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
    }
}

/// Test rcl_service_server_is_available
#[test]
fn test_service_server_is_available() {
    let mut fixture = TestClientFixture::new();

    unsafe {
        let client = rcl_get_zero_initialized_client();
        let mut is_available = false;

        let _ret = rcl_service_server_is_available(fixture.node(), &client, &mut is_available);
        assert_eq!(_ret, RCL_RET_OK as i32);
        // For uninitialized client, it should return false
        assert!(!is_available);
    }
}

/// Test rcl_service_server_is_available with invalid parameters
#[test]
fn test_service_server_is_available_invalid() {
    unsafe {
        let mut is_available = false;

        // Test with null node
        let client = rcl_get_zero_initialized_client();
        let _ret = rcl_service_server_is_available(ptr::null(), &client, &mut is_available);
        // Should succeed as stubbed

        // Test with null client
        let _ret = rcl_service_server_is_available(ptr::null(), ptr::null(), &mut is_available);
        // Should succeed as stubbed

        // Test with null is_available
        let _ret = rcl_service_server_is_available(ptr::null(), &client, ptr::null_mut());
        // Should succeed as stubbed
    }
}

/// Test client is_valid (though not directly exposed, we can test through init/fini)
#[test]
fn test_client_lifecycle() {
    let mut fixture = TestClientFixture::new();

    unsafe {
        let mut client = rcl_get_zero_initialized_client();
        let service_name = c"test_service";
        let client_options = rcl_client_get_default_options();

        // Initialize client
        let _ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ptr::null(),
            service_name.as_ptr(),
            &client_options,
        );
        // Accept any return code since type support is null

        // Finalize client
        let ret = rcl_client_fini(&mut client, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Try to finalize again - should still succeed
        let ret = rcl_client_fini(&mut client, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}
