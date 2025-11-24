#![cfg(feature = "test-core")]
#![allow(unused_unsafe)]

use std::ptr;

use rcl_z::{
    context::{
        rcl_context_fini, rcl_context_get_instance_id, rcl_context_is_valid,
        rcl_get_zero_initialized_context, rcl_shutdown,
    },
    init::{
        rcl_get_default_allocator, rcl_get_zero_initialized_init_options, rcl_init,
        rcl_init_options_copy, rcl_init_options_fini, rcl_init_options_get_allocator,
        rcl_init_options_get_domain_id, rcl_init_options_init, rcl_init_options_set_domain_id,
    },
    ros::{
        RCL_RET_ALREADY_INIT, RCL_RET_ALREADY_SHUTDOWN, RCL_RET_INVALID_ARGUMENT,
        RCL_RET_INVALID_ROS_ARGS, RCL_RET_OK,
    },
};

/// Test rcl_init_options_init() and rcl_init_options_fini() functions.
/// Aligns with test_init.cpp::test_rcl_init_options_init
#[test]
fn test_rcl_init_options_init() {
    unsafe {
        let mut init_options = rcl_get_zero_initialized_init_options();

        // Fini a not initialized options - should fail
        let ret = rcl_init_options_fini(&mut init_options);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Expected usage
        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Already init - should fail
        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_ALREADY_INIT as i32);

        // Cleanup
        let ret = rcl_init_options_fini(&mut init_options);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Test with nullptr - should fail
        let ret = rcl_init_options_init(ptr::null_mut(), rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test fini with nullptr - should fail
        let ret = rcl_init_options_fini(ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
    }
}

/// Test calling rcl_init() with invalid arguments fails.
/// Aligns with test_init.cpp::test_rcl_init_invalid_arguments
#[test]
fn test_rcl_init_invalid_arguments() {
    unsafe {
        let mut init_options = rcl_get_zero_initialized_init_options();
        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        // If argc is not 0, but argv is, it should be an invalid argument
        {
            let mut context = rcl_get_zero_initialized_context();
            let ret = rcl_init(42, ptr::null(), &init_options, &mut context);
            assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
            assert!(!rcl_context_is_valid(&context));
        }

        // If argc is not 0, argv is not null but contains one, it should be an invalid argument
        {
            let mut context = rcl_get_zero_initialized_context();
            let null_args: [*const i8; 2] = [b"some-arg\0".as_ptr() as *const i8, ptr::null()];
            let ret = rcl_init(2, null_args.as_ptr(), &init_options, &mut context);
            assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
            assert!(!rcl_context_is_valid(&context));
        }

        // If argc is less than 1, argv is not null, it should be an invalid argument
        {
            let mut context = rcl_get_zero_initialized_context();
            let some_args: [*const i8; 1] = [b"some-arg\0".as_ptr() as *const i8];
            let ret = rcl_init(0, some_args.as_ptr(), &init_options, &mut context);
            assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
            assert!(!rcl_context_is_valid(&context));
        }

        // If an invalid ROS arg is given, init should fail
        {
            let mut context = rcl_get_zero_initialized_context();
            let bad_remap_args: [*const i8; 4] = [
                b"some-arg\0".as_ptr() as *const i8,
                b"--ros-args\0".as_ptr() as *const i8,
                b"-r\0".as_ptr() as *const i8,
                b"name:=\0".as_ptr() as *const i8,
            ];
            let ret = rcl_init(4, bad_remap_args.as_ptr(), &init_options, &mut context);
            assert_eq!(ret, RCL_RET_INVALID_ROS_ARGS as i32);
            assert!(!rcl_context_is_valid(&context));
        }

        // Cleanup
        let ret = rcl_init_options_fini(&mut init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test the rcl_init() and rcl_shutdown() functions.
/// Aligns with test_init.cpp::test_rcl_init_and_shutdown
#[test]
fn test_rcl_init_and_shutdown() {
    unsafe {
        let mut init_options = rcl_get_zero_initialized_init_options();
        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut context = rcl_get_zero_initialized_context();

        // A shutdown before an init should fail
        let ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        assert!(!rcl_context_is_valid(&context));

        // If argc is 0 and argv is nullptr and the allocator is valid, it should succeed
        let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(rcl_context_is_valid(&context));

        // Then shutdown should work
        let ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!rcl_context_is_valid(&context));

        let ret = rcl_context_fini(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Valid argc/argv values and a valid allocator should succeed
        context = rcl_get_zero_initialized_context();
        {
            let args: [*const i8; 2] = [
                b"foo\0".as_ptr() as *const i8,
                b"bar\0".as_ptr() as *const i8,
            ];
            let ret = rcl_init(2, args.as_ptr(), &init_options, &mut context);
            assert_eq!(ret, RCL_RET_OK as i32);
            assert!(rcl_context_is_valid(&context));
        }

        // Then shutdown should work
        let ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!rcl_context_is_valid(&context));

        // Then a repeated shutdown should fail
        let ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_ALREADY_SHUTDOWN as i32);
        assert!(!rcl_context_is_valid(&context));

        let ret = rcl_context_fini(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        // A repeat call to shutdown should not work
        context = rcl_get_zero_initialized_context();
        let ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        assert!(!rcl_context_is_valid(&context));

        // Repeat, but valid, calls to rcl_init() should fail
        {
            let args: [*const i8; 2] = [
                b"foo\0".as_ptr() as *const i8,
                b"bar\0".as_ptr() as *const i8,
            ];
            let ret = rcl_init(2, args.as_ptr(), &init_options, &mut context);
            assert_eq!(ret, RCL_RET_OK as i32);
            assert!(rcl_context_is_valid(&context));

            let ret = rcl_init(2, args.as_ptr(), &init_options, &mut context);
            assert_eq!(ret, RCL_RET_ALREADY_INIT as i32);
            assert!(rcl_context_is_valid(&context));
        }

        // But shutdown should still work
        let ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!rcl_context_is_valid(&context));

        let ret = rcl_context_fini(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Cleanup
        let ret = rcl_init_options_fini(&mut init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test the rcl_get_instance_id() function.
/// Aligns with test_init.cpp::test_rcl_get_instance_id
#[test]
fn test_rcl_get_instance_id() {
    unsafe {
        let mut context = rcl_get_zero_initialized_context();

        // Instance id should be 0 before rcl_init()
        assert_eq!(0u64, rcl_context_get_instance_id(&context));
        assert!(!rcl_context_is_valid(&context));

        // It should still return 0 after an invalid init
        let ret = rcl_init(1, ptr::null(), ptr::null(), &mut context);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        assert_eq!(0u64, rcl_context_get_instance_id(&context));
        assert!(!rcl_context_is_valid(&context));

        // A non-zero instance id should be returned after a valid init
        let mut init_options = rcl_get_zero_initialized_init_options();
        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        {
            let args: [*const i8; 2] = [
                b"foo\0".as_ptr() as *const i8,
                b"bar\0".as_ptr() as *const i8,
            ];
            let ret = rcl_init(2, args.as_ptr(), &init_options, &mut context);
            assert_eq!(ret, RCL_RET_OK as i32);
            assert!(rcl_context_is_valid(&context));
        }

        let first_instance_id = rcl_context_get_instance_id(&context);
        assert_ne!(0u64, first_instance_id);

        // Repeat calls should return the same
        assert_eq!(first_instance_id, rcl_context_get_instance_id(&context));
        assert!(rcl_context_is_valid(&context));

        // Calling after a shutdown should return 0
        let ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(0u64, rcl_context_get_instance_id(&context));
        assert!(!rcl_context_is_valid(&context));

        let ret = rcl_context_fini(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        // It should return a different value after another valid init
        context = rcl_get_zero_initialized_context();
        {
            let args: [*const i8; 2] = [
                b"foo\0".as_ptr() as *const i8,
                b"bar\0".as_ptr() as *const i8,
            ];
            let ret = rcl_init(2, args.as_ptr(), &init_options, &mut context);
            assert_eq!(ret, RCL_RET_OK as i32);
            assert!(rcl_context_is_valid(&context));
        }

        assert_ne!(0u64, rcl_context_get_instance_id(&context));
        assert_ne!(first_instance_id, rcl_context_get_instance_id(&context));
        assert!(rcl_context_is_valid(&context));

        // Shutting down a second time should result in 0 again
        let ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(0u64, rcl_context_get_instance_id(&context));
        assert!(!rcl_context_is_valid(&context));

        let ret = rcl_context_fini(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Cleanup
        let ret = rcl_init_options_fini(&mut init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test init_options accessors.
/// Aligns with test_init.cpp::test_rcl_init_options_access
#[test]
fn test_rcl_init_options_access() {
    unsafe {
        let mut init_options = rcl_get_zero_initialized_init_options();
        let not_ini_init_options = rcl_get_zero_initialized_init_options();

        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        // Test rcl_init_options_get_allocator
        let options_allocator = rcl_init_options_get_allocator(&init_options);
        assert!(!options_allocator.is_null());

        let null_allocator = rcl_init_options_get_allocator(ptr::null());
        assert!(null_allocator.is_null());

        let uninit_allocator = rcl_init_options_get_allocator(&not_ini_init_options);
        assert!(uninit_allocator.is_null());

        // Test rcl_init_options_get_domain_id
        let mut domain_id: usize = 0;

        let ret = rcl_init_options_get_domain_id(ptr::null(), &mut domain_id);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        let ret = rcl_init_options_get_domain_id(&not_ini_init_options, &mut domain_id);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        let ret = rcl_init_options_get_domain_id(&init_options, ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        let ret = rcl_init_options_get_domain_id(ptr::null(), ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test rcl_init_options_set_domain_id
        let ret = rcl_init_options_set_domain_id(ptr::null_mut(), domain_id);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        let ret =
            rcl_init_options_set_domain_id(&not_ini_init_options as *const _ as *mut _, domain_id);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Get default domain ID
        let ret = rcl_init_options_get_domain_id(&init_options, &mut domain_id);
        assert_eq!(ret, RCL_RET_OK as i32);
        // Default domain ID should be a valid value (typically 0 or RCL_DEFAULT_DOMAIN_ID)

        // Set domain ID to 0
        let ret = rcl_init_options_set_domain_id(&mut init_options, 0);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_init_options_get_domain_id(&init_options, &mut domain_id);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(0, domain_id);

        // Test rcl_init_options_copy
        let mut init_options_dst = rcl_get_zero_initialized_init_options();

        // nullptr copy cases
        let ret = rcl_init_options_copy(ptr::null(), &mut init_options_dst);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        let ret = rcl_init_options_copy(&init_options, ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Expected usage copy
        let ret = rcl_init_options_copy(&init_options, &mut init_options_dst);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_init_options_get_domain_id(&init_options_dst, &mut domain_id);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(0, domain_id);

        // Copy to already initialized should fail
        let ret = rcl_init_options_copy(&init_options, &mut init_options_dst);
        assert_eq!(ret, RCL_RET_ALREADY_INIT as i32);

        // Cleanup
        let ret = rcl_init_options_fini(&mut init_options_dst);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_init_options_fini(&mut init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}
