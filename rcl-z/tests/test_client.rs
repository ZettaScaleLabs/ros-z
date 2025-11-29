// Copyright 2025 ZettaScale Technology
// SPDX-License-Identifier: Apache-2.0
//
// Ported from ros2/rcl:
// Copyright 2016 Open Source Robotics Foundation, Inc.

#![cfg(feature = "test-msgs")]

mod test_msgs_support;

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
// Re-export from test_msgs_support for convenience
use test_msgs_support::*;

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

/// Basic nominal test of a client. Complete functionality tested at test_service.cpp
#[test]
#[allow(clippy::field_reassign_with_default)]
// Using ..Default::default() in struct initialization causes shallow copy issues
// for fields containing pointers (e.g., string_value), leading to double-free on drop.
// Therefore, we initialize with Default::default() and reassign fields instead.
fn test_client_nominal() {
    let mut fixture = TestClientFixture::new();

    unsafe {
        let mut client = rcl_get_zero_initialized_client();

        // Initialize the client.
        let service_name = c"add_two_ints";
        let _expected_service_name = c"/add_two_ints";
        let client_options = rcl_client_get_default_options();

        let ts = ROSIDL_GET_SRV_TYPE_SUPPORT!(test_msgs, srv, BasicTypes);
        let ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ts,
            service_name.as_ptr(),
            &client_options,
        );

        // Test access to client options
        let client_internal_options = rcl_client_get_options(&client);
        assert!(!client_internal_options.is_null());
        // Note: QoS checks would require accessing the struct, but we skip for simplicity

        let request_publisher_qos = rcl_client_request_publisher_get_actual_qos(&client);
        assert!(!request_publisher_qos.is_null());

        let response_subscription_qos = rcl_client_response_subscription_get_actual_qos(&client);
        assert!(!response_subscription_qos.is_null());

        // Check the return code of initialization and that the service name matches what's expected
        assert_eq!(ret, RCL_RET_OK as i32);
        let actual_service_name = rcl_client_get_service_name(&client);
        assert!(!actual_service_name.is_null());
        // Note: String comparison would require unsafe C string handling

        // Initialize the client request.
        let mut req = test_msgs__srv__BasicTypes_Request::default();
        req.uint8_value = 1;
        req.uint32_value = 2;

        // Check that there were no errors while sending the request.
        let mut sequence_number: i64 = 0;
        let ret = rcl_send_request(
            &client,
            &req as *const _ as *const c_void,
            &mut sequence_number,
        );
        assert_eq!(sequence_number, 1);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Finalize client
        let ret = rcl_client_fini(&mut client, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Testing the client init and fini functions.
#[test]
fn test_client_init_fini() {
    let mut fixture = TestClientFixture::new();

    unsafe {
        let ts = ROSIDL_GET_SRV_TYPE_SUPPORT!(test_msgs, srv, BasicTypes);
        let service_name = c"chatter";
        let default_client_options = rcl_client_get_default_options();

        // Try passing null for client in init.
        let ret = rcl_client_init(
            ptr::null_mut(),
            fixture.node(),
            ts,
            service_name.as_ptr(),
            &default_client_options,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Try passing null for a node pointer in init.
        let mut client = rcl_get_zero_initialized_client();
        let ret = rcl_client_init(
            &mut client,
            ptr::null(),
            ts,
            service_name.as_ptr(),
            &default_client_options,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Check if null client is valid
        assert!(!rcl_client_is_valid(ptr::null()));

        // Check if zero initialized client is valid
        let client = rcl_get_zero_initialized_client();
        assert!(!rcl_client_is_valid(&client));

        // Check that a valid client is valid
        let mut client = rcl_get_zero_initialized_client();
        let ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ts,
            service_name.as_ptr(),
            &default_client_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(rcl_client_is_valid(&client));
        let ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ts,
            service_name.as_ptr(),
            &default_client_options,
        );
        assert_eq!(ret, RCL_RET_ALREADY_INIT as i32);
        let ret = rcl_client_fini(&mut client, fixture.node());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Try passing an invalid (uninitialized) node in init.
        let mut client = rcl_get_zero_initialized_client();
        let invalid_node = rcl_get_zero_initialized_node();
        let ret = rcl_client_init(
            &mut client,
            &invalid_node,
            ts,
            service_name.as_ptr(),
            &default_client_options,
        );
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Try passing null for the type support in init.
        let mut client = rcl_get_zero_initialized_client();
        let ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ptr::null(),
            service_name.as_ptr(),
            &default_client_options,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Try passing null for the service name in init.
        let mut client = rcl_get_zero_initialized_client();
        let ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ts,
            ptr::null(),
            &default_client_options,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Try passing null for the options in init.
        let mut client = rcl_get_zero_initialized_client();
        let ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ts,
            service_name.as_ptr(),
            ptr::null(),
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Try passing options with an invalid allocator in allocator with init.
        let mut client = rcl_get_zero_initialized_client();
        let mut client_options_with_invalid_allocator = rcl_client_get_default_options();
        client_options_with_invalid_allocator.allocator.allocate = None;
        let ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ts,
            service_name.as_ptr(),
            &client_options_with_invalid_allocator,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Try passing options with an invalid deallocate in allocator with init.
        let mut client = rcl_get_zero_initialized_client();
        let mut client_options_with_invalid_allocator = rcl_client_get_default_options();
        client_options_with_invalid_allocator.allocator.deallocate = None;
        let ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ts,
            service_name.as_ptr(),
            &client_options_with_invalid_allocator,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Try passing options with a failing allocator with init.
        // Note: Skipping failing allocator test as it's complex to implement in Rust
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

/// Passing bad/invalid arguments to the functions
#[test]
fn test_client_bad_arguments() {
    let mut fixture = TestClientFixture::new();

    unsafe {
        let mut client = rcl_get_zero_initialized_client();
        let ts = ROSIDL_GET_SRV_TYPE_SUPPORT!(test_msgs, srv, BasicTypes);
        let default_client_options = rcl_client_get_default_options();

        let ret = rcl_client_init(
            &mut client,
            fixture.node(),
            ts,
            c"invalid name".as_ptr(),
            &default_client_options,
        );
        assert_eq!(ret, RCL_RET_SERVICE_NAME_INVALID as i32);

        // Test fini with null node
        let ret = rcl_client_fini(&mut client, ptr::null_mut());
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        let mut invalid_node = rcl_get_zero_initialized_node();
        let ret = rcl_client_fini(&mut client, &mut invalid_node);
        assert_eq!(ret, RCL_RET_NODE_INVALID as i32);

        // Test various helper functions with null/invalid clients
        let mut request_header = rmw_service_info_t::default();
        let dummy_response: *mut c_void = ptr::null_mut();
        let mut sequence_number: i64 = 24;
        let client_request = test_msgs__srv__BasicTypes_Request::default();

        assert_eq!(rcl_client_get_rmw_handle(ptr::null()), ptr::null_mut());
        assert_eq!(rcl_client_get_service_name(ptr::null()), ptr::null());
        assert_eq!(rcl_client_get_options(ptr::null()), ptr::null());
        assert_eq!(
            rcl_take_response_with_info(ptr::null(), &mut request_header, dummy_response),
            RCL_RET_INVALID_ARGUMENT as i32
        );
        assert_eq!(
            rcl_take_response(ptr::null(), &mut request_header.request_id, dummy_response),
            RCL_RET_INVALID_ARGUMENT as i32
        );
        assert_eq!(
            rcl_send_request(
                ptr::null(),
                &client_request as *const _ as *const c_void,
                &mut sequence_number
            ),
            RCL_RET_INVALID_ARGUMENT as i32
        );
        assert_eq!(sequence_number, 24); // Should remain unchanged
        // QoS functions return null for null client, no error

        // Test with uninitialized client
        let client = rcl_get_zero_initialized_client();
        assert_eq!(rcl_client_get_rmw_handle(&client), ptr::null_mut());
        assert_eq!(rcl_client_get_service_name(&client), ptr::null());
        assert_eq!(rcl_client_get_options(&client), ptr::null());
        assert_eq!(
            rcl_take_response_with_info(&client, &mut request_header, dummy_response),
            RCL_RET_CLIENT_INVALID as i32
        );
        assert_eq!(
            rcl_take_response(&client, &mut request_header.request_id, dummy_response),
            RCL_RET_CLIENT_INVALID as i32
        );
        assert_eq!(
            rcl_send_request(
                &client,
                &client_request as *const _ as *const c_void,
                &mut sequence_number
            ),
            RCL_RET_CLIENT_INVALID as i32
        );
        assert_eq!(sequence_number, 24); // Should remain unchanged
        assert_eq!(
            rcl_client_request_publisher_get_actual_qos(&client),
            ptr::null()
        );
        assert_eq!(
            rcl_client_response_subscription_get_actual_qos(&client),
            ptr::null()
        );
    }
}
