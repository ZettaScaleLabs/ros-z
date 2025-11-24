// Ported from Open Source Robotics Foundation code (2019)
// https://github.com/ros2/rcl
// Licensed under the Apache License 2.0
// Copyright 2025 ZettaScale Technology

#![cfg(feature = "test-core")]
#![allow(unused_unsafe)]

use std::ptr;

use rcl_z::{
    context::{
        rcl_context_fini, rcl_context_get_domain_id, rcl_context_get_init_options,
        rcl_context_get_instance_id, rcl_context_get_rmw_context, rcl_context_is_valid,
        rcl_get_zero_initialized_context, rcl_shutdown,
    },
    init::{
        rcl_get_default_allocator, rcl_get_zero_initialized_init_options, rcl_init,
        rcl_init_options_fini, rcl_init_options_init,
    },
    ros::{RCL_RET_INVALID_ARGUMENT, RCL_RET_OK},
};

/// Test the rcl_context_t's normal function.
/// This test aligns with test_context.cpp::TEST(TestContext, nominal)
#[test]
fn test_context_nominal() {
    unsafe {
        // Initialize context with rcl_init
        let mut context = rcl_get_zero_initialized_context();
        let mut init_options = rcl_get_zero_initialized_init_options();
        let mut ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        ret = rcl_init(0, ptr::null(), &init_options, &mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Test rcl_context_get_init_options with nullptr
        let init_options_ptr = rcl_context_get_init_options(ptr::null());
        assert!(init_options_ptr.is_null());

        // Test rcl_context_get_init_options with valid context
        let init_options_ptr = rcl_context_get_init_options(&context);
        assert!(!init_options_ptr.is_null());

        // Test rcl_context_get_instance_id with nullptr
        let instance_id = rcl_context_get_instance_id(ptr::null());
        assert_eq!(instance_id, 0u64);

        // Test rcl_context_get_instance_id with valid context
        let instance_id = rcl_context_get_instance_id(&context);
        assert_ne!(instance_id, 0u64);

        // Test rcl_context_get_domain_id with nullptr for context
        let mut domain_id: usize = 0;
        ret = rcl_context_get_domain_id(ptr::null(), &mut domain_id);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test rcl_context_get_domain_id with nullptr for domain_id
        ret = rcl_context_get_domain_id(&context, ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test rcl_context_get_domain_id with valid arguments
        ret = rcl_context_get_domain_id(&context, &mut domain_id);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Test rcl_context_is_valid with nullptr
        let is_valid = rcl_context_is_valid(ptr::null());
        assert!(!is_valid);

        // Test rcl_context_is_valid with valid context
        let is_valid = rcl_context_is_valid(&context);
        assert!(is_valid);

        // Test rcl_context_get_rmw_context with nullptr
        let rmw_context_ptr = rcl_context_get_rmw_context(ptr::null_mut());
        assert!(rmw_context_ptr.is_null());

        // Test rcl_context_get_rmw_context with valid context
        let rmw_context_ptr = rcl_context_get_rmw_context(&mut context);
        assert!(!rmw_context_ptr.is_null());

        // Cleanup
        ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        ret = rcl_context_fini(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        ret = rcl_init_options_fini(&mut init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test bad fini scenarios.
/// This test aligns with test_context.cpp::TEST(TestContext, bad_fini)
#[test]
fn test_context_bad_fini() {
    unsafe {
        // Test rcl_context_fini with nullptr - should return INVALID_ARGUMENT
        let ret = rcl_context_fini(ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        let mut init_options = rcl_get_zero_initialized_init_options();
        let mut ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Test fini on zero-initialized context - should return OK
        let mut context = rcl_get_zero_initialized_context();
        ret = rcl_context_fini(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Initialize context
        ret = rcl_init(0, ptr::null(), &init_options, &mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Test fini on initialized but not shutdown context - should return INVALID_ARGUMENT
        ret = rcl_context_fini(&mut context);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Shutdown context
        ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Now fini should succeed
        ret = rcl_context_fini(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Cleanup
        ret = rcl_init_options_fini(&mut init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}
