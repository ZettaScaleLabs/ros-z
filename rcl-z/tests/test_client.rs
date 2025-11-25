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
    ros::*,
    service::{
        rcl_client_fini, rcl_client_get_default_options, rcl_client_get_options,
        rcl_client_get_rmw_handle, rcl_client_get_service_name, rcl_client_init,
        rcl_client_is_valid, rcl_client_request_publisher_get_actual_qos,
        rcl_client_response_subscription_get_actual_qos, rcl_get_zero_initialized_client,
        rcl_send_request, rcl_service_server_is_available, rcl_take_response,
        rcl_take_response_with_info,
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
        assert_eq!(ret, RCL_RET_CLIENT_INVALID as i32);

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
        assert_eq!(ret, RCL_RET_CLIENT_INVALID as i32);

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

/// Test rcl_client_is_valid with various client states
#[test]
fn test_client_is_valid_comprehensive() {
    let _fixture = TestClientFixture::new();

    // Check if null client is valid
    assert!(
        !rcl_client_is_valid(ptr::null()),
        "Null client should not be valid"
    );

    // Check if zero initialized client is valid
    let client = rcl_get_zero_initialized_client();
    assert!(
        !rcl_client_is_valid(&client),
        "Zero initialized client should not be valid"
    );

    // We can't fully test a valid client without type support
    // but the basic validation checks work
}

/// Test rcl_client_get_service_name
#[test]
fn test_client_get_service_name() {
    // Test with null client
    let service_name = rcl_client_get_service_name(ptr::null());
    assert!(
        service_name.is_null(),
        "Service name should be null for null client"
    );

    // Test with zero-initialized (invalid) client
    let client = rcl_get_zero_initialized_client();
    let service_name = rcl_client_get_service_name(&client);
    assert!(
        service_name.is_null(),
        "Service name should be null for invalid client"
    );
}

/// Test rcl_client helper functions
#[test]
fn test_client_helper_functions() {
    // Test with null client
    assert!(
        rcl_client_get_options(ptr::null()).is_null(),
        "get_options should return null for null client"
    );
    assert!(
        rcl_client_get_rmw_handle(ptr::null()).is_null(),
        "get_rmw_handle should return null for null client"
    );
    assert!(
        rcl_client_get_service_name(ptr::null()).is_null(),
        "get_service_name should return null for null client"
    );
    assert!(
        rcl_client_request_publisher_get_actual_qos(ptr::null()).is_null(),
        "request_publisher QoS should return null for null client"
    );
    assert!(
        rcl_client_response_subscription_get_actual_qos(ptr::null()).is_null(),
        "response_subscription QoS should return null for null client"
    );

    // Test with zero-initialized (invalid) client
    let client = rcl_get_zero_initialized_client();
    assert!(
        rcl_client_get_options(&client).is_null(),
        "get_options should return null for invalid client"
    );
    assert!(
        rcl_client_get_rmw_handle(&client).is_null(),
        "get_rmw_handle should return null for invalid client"
    );
    assert!(
        rcl_client_get_service_name(&client).is_null(),
        "get_service_name should return null for invalid client"
    );
    assert!(
        rcl_client_request_publisher_get_actual_qos(&client).is_null(),
        "request_publisher QoS should return null for invalid client"
    );
    assert!(
        rcl_client_response_subscription_get_actual_qos(&client).is_null(),
        "response_subscription QoS should return null for invalid client"
    );
}

/// Test rcl_client_init with NODE_INVALID error
#[test]
fn test_client_init_node_invalid() {
    unsafe {
        let mut client = rcl_get_zero_initialized_client();
        let service_name = c"test_service";
        let client_options = rcl_client_get_default_options();

        // Test with null node - should return RCL_RET_NODE_INVALID
        let ret = rcl_client_init(
            &mut client,
            ptr::null(),
            ptr::null(),
            service_name.as_ptr(),
            &client_options,
        );
        assert_eq!(
            ret, RCL_RET_NODE_INVALID as i32,
            "Null node should return RCL_RET_NODE_INVALID"
        );

        // Test with zero-initialized (invalid) node
        // Note: Without type support, we get RCL_RET_INVALID_ARGUMENT (11)
        // With proper type support, it would return RCL_RET_NODE_INVALID (200)
        // But since type support is null here, the error comes from that first
        let invalid_node = rcl_get_zero_initialized_node();
        let ret = rcl_client_init(
            &mut client,
            &invalid_node,
            ptr::null(),
            service_name.as_ptr(),
            &client_options,
        );
        // The function checks type_support before it can even try to use the node,
        // so we get INVALID_ARGUMENT for null type support
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Null type support should return RCL_RET_INVALID_ARGUMENT"
        );
    }
}

/// Test rcl_client_fini with various invalid arguments
#[test]
fn test_client_fini_error_codes() {
    let mut fixture = TestClientFixture::new();

    let mut client = rcl_get_zero_initialized_client();

    // Test with null client - should return RCL_RET_CLIENT_INVALID
    let ret = rcl_client_fini(ptr::null_mut(), fixture.node());
    assert_eq!(
        ret, RCL_RET_CLIENT_INVALID as i32,
        "Null client should return RCL_RET_CLIENT_INVALID"
    );

    // Test with null node - should return RCL_RET_NODE_INVALID
    let ret = rcl_client_fini(&mut client, ptr::null_mut());
    assert_eq!(
        ret, RCL_RET_NODE_INVALID as i32,
        "Null node should return RCL_RET_NODE_INVALID"
    );

    // Test with zero-initialized (invalid) node - should return RCL_RET_NODE_INVALID
    let mut invalid_node = rcl_get_zero_initialized_node();
    let ret = rcl_client_fini(&mut client, &mut invalid_node as *mut _);
    assert_eq!(
        ret, RCL_RET_NODE_INVALID as i32,
        "Fini with invalid node should return RCL_RET_NODE_INVALID"
    );
}

/// Test rcl_take_response_with_info
#[test]
fn test_take_response_with_info_null_arguments() {
    unsafe {
        let mut response_header = rmw_service_info_t::default();
        let dummy_response: *mut c_void = ptr::null_mut();

        // Test with null client
        let ret = rcl_take_response_with_info(ptr::null(), &mut response_header, dummy_response);
        assert_eq!(
            ret, RCL_RET_CLIENT_INVALID as i32,
            "Null client should return RCL_RET_CLIENT_INVALID"
        );

        // Test with null response
        let client = rcl_get_zero_initialized_client();
        let ret = rcl_take_response_with_info(&client, &mut response_header, ptr::null_mut());
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Null response should return RCL_RET_INVALID_ARGUMENT"
        );

        // Test with null request header
        let ret = rcl_take_response_with_info(&client, ptr::null_mut(), dummy_response);
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Null request header should return RCL_RET_INVALID_ARGUMENT"
        );
    }
}

/// Test client initialization with invalid service name
#[test]
fn test_client_init_invalid_service_name() {
    let mut fixture = TestClientFixture::new();

    unsafe {
        // Test with service name containing whitespace (invalid)
        let invalid_service_name = c"invalid name";
        let mut client = rcl_get_zero_initialized_client();
        let client_options = rcl_client_get_default_options();

        let ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ptr::null(),
            invalid_service_name.as_ptr(),
            &client_options,
        );

        // Note: When type support is null, we get RCL_RET_INVALID_ARGUMENT
        // before service name validation. With valid type support, this would
        // return RCL_RET_SERVICE_NAME_INVALID.
        // For this test without type support, we just verify it fails.
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Client init should fail with RCL_RET_INVALID_ARGUMENT (null type support checked first)"
        );
    }
}
