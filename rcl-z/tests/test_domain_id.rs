#![cfg(feature = "test-core")]

use std::ptr;

use rcl_z::{
    context::{
        rcl_context_fini, rcl_context_get_domain_id, rcl_get_zero_initialized_context, rcl_shutdown,
    },
    init::{
        rcl_get_default_allocator, rcl_get_default_domain_id,
        rcl_get_zero_initialized_init_options, rcl_init, rcl_init_options_fini,
        rcl_init_options_init,
    },
    ros::{RCL_DEFAULT_DOMAIN_ID, RCL_RET_ERROR, RCL_RET_INVALID_ARGUMENT, RCL_RET_OK},
};

/// Test rcl_context_get_domain_id with valid context
#[test]
#[allow(unused_unsafe)]
fn test_context_get_domain_id_nominal() {
    unsafe {
        // Initialize context
        let mut init_options = rcl_get_zero_initialized_init_options();
        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut context = rcl_get_zero_initialized_context();
        let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Get domain ID
        let mut domain_id: usize = 999;
        let ret = rcl_context_get_domain_id(&context, &mut domain_id);
        assert_eq!(ret, RCL_RET_OK as i32);
        // Default domain ID should be 0
        assert_eq!(domain_id, 0);

        // Cleanup
        let ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_context_fini(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_init_options_fini(&mut init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_context_get_domain_id with invalid arguments
#[test]
#[allow(unused_unsafe)]
fn test_context_get_domain_id_invalid_arguments() {
    unsafe {
        // Test with null domain_id pointer
        let context = rcl_get_zero_initialized_context();
        let ret = rcl_context_get_domain_id(&context, ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with null context
        let mut domain_id: usize = 0;
        let ret = rcl_context_get_domain_id(ptr::null(), &mut domain_id);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
    }
}

/// Test rcl_get_default_domain_id function with various environment variable values.
/// Aligns with test_domain_id.cpp::test_nominal
#[test]
#[allow(unused_unsafe)]
fn test_get_default_domain_id_nominal() {
    unsafe {
        // Test with ROS_DOMAIN_ID="42"
        std::env::set_var("ROS_DOMAIN_ID", "42");
        let mut domain_id: usize = RCL_DEFAULT_DOMAIN_ID as usize;
        let ret = rcl_get_default_domain_id(&mut domain_id);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(domain_id, 42);

        // Test with ROS_DOMAIN_ID="" (empty string)
        std::env::set_var("ROS_DOMAIN_ID", "");
        domain_id = RCL_DEFAULT_DOMAIN_ID as usize;
        let ret = rcl_get_default_domain_id(&mut domain_id);
        assert_eq!(ret, RCL_RET_OK as i32);
        // Empty string should leave domain_id unchanged
        assert_eq!(domain_id, RCL_DEFAULT_DOMAIN_ID as usize);

        // Test with ROS_DOMAIN_ID="0000" (leading zeros)
        std::env::set_var("ROS_DOMAIN_ID", "0000");
        domain_id = RCL_DEFAULT_DOMAIN_ID as usize;
        let ret = rcl_get_default_domain_id(&mut domain_id);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(domain_id, 0);

        // Test with ROS_DOMAIN_ID="0   not really" (invalid format)
        std::env::set_var("ROS_DOMAIN_ID", "0   not really");
        domain_id = RCL_DEFAULT_DOMAIN_ID as usize;
        let ret = rcl_get_default_domain_id(&mut domain_id);
        assert_eq!(ret, RCL_RET_ERROR as i32);
        // domain_id should remain unchanged on error
        assert_eq!(domain_id, RCL_DEFAULT_DOMAIN_ID as usize);

        // Test with ROS_DOMAIN_ID="998446744073709551615" (overflow value)
        std::env::set_var("ROS_DOMAIN_ID", "998446744073709551615");
        domain_id = RCL_DEFAULT_DOMAIN_ID as usize;
        let ret = rcl_get_default_domain_id(&mut domain_id);
        assert_eq!(ret, RCL_RET_ERROR as i32);
        // domain_id should remain unchanged on error
        assert_eq!(domain_id, RCL_DEFAULT_DOMAIN_ID as usize);

        // Test with nullptr
        let ret = rcl_get_default_domain_id(ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Cleanup: remove the environment variable
        std::env::remove_var("ROS_DOMAIN_ID");
    }
}

/// Test rcl_get_default_domain_id when ROS_DOMAIN_ID is not set.
#[test]
#[allow(unused_unsafe)]
fn test_get_default_domain_id_not_set() {
    unsafe {
        // Ensure ROS_DOMAIN_ID is not set
        std::env::remove_var("ROS_DOMAIN_ID");

        let mut domain_id: usize = 999;
        let ret = rcl_get_default_domain_id(&mut domain_id);
        assert_eq!(ret, RCL_RET_OK as i32);
        // When not set, domain_id should remain unchanged
        assert_eq!(domain_id, 999);
    }
}
