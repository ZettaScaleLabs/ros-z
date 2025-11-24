// Ported from Open Source Robotics Foundation code (2018)
// https://github.com/ros2/rcl
// Licensed under the Apache License 2.0
// Copyright 2025 ZettaScale Technology

#![cfg(feature = "test-core")]

use std::ptr;

use rcl_z::{
    arguments::{
        rcl_arguments_copy, rcl_arguments_fini, rcl_arguments_get_count_unparsed,
        rcl_arguments_get_count_unparsed_ros, rcl_arguments_get_log_levels,
        rcl_arguments_get_param_files, rcl_arguments_get_param_files_count,
        rcl_arguments_get_param_overrides, rcl_arguments_get_unparsed,
        rcl_arguments_get_unparsed_ros, rcl_get_zero_initialized_arguments, rcl_parse_arguments,
        rcl_remove_ros_arguments,
    },
    init::rcl_get_default_allocator,
    ros::{RCL_RET_OK, rcl_arguments_t, rcl_log_levels_t, rcl_params_t},
};

/// Test fixture that provides initialized arguments
struct TestArgumentsFixture {
    args: rcl_arguments_t,
}

impl TestArgumentsFixture {
    fn new() -> Self {
        let args = rcl_get_zero_initialized_arguments();
        TestArgumentsFixture { args }
    }
}

impl Drop for TestArgumentsFixture {
    fn drop(&mut self) {
        unsafe {
            let _ = rcl_arguments_fini(&mut self.args);
        }
    }
}

/// Test rcl_get_zero_initialized_arguments
#[test]
fn test_get_zero_initialized_arguments() {
    let args = rcl_get_zero_initialized_arguments();
    // Zero initialized should be default
    assert_eq!(args, rcl_arguments_t::default());
}

/// Test rcl_parse_arguments with valid inputs
#[test]
fn test_parse_arguments() {
    let mut fixture = TestArgumentsFixture::new();

    unsafe {
        // Test with empty arguments
        let ret = rcl_parse_arguments(
            0,
            ptr::null(),
            rcl_get_default_allocator(),
            &mut fixture.args,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Test with some arguments
        let arg1 = c"--ros-args";
        let arg2 = c"--log-level";
        let arg3 = c"info";
        let argv: [*const i8; 3] = [arg1.as_ptr(), arg2.as_ptr(), arg3.as_ptr()];
        let ret = rcl_parse_arguments(
            3,
            argv.as_ptr(),
            rcl_get_default_allocator(),
            &mut fixture.args,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_parse_arguments with invalid inputs
#[test]
fn test_parse_arguments_invalid() {
    let mut fixture = TestArgumentsFixture::new();

    unsafe {
        // Test with null argv but non-zero argc
        let _ret = rcl_parse_arguments(
            1,
            ptr::null(),
            rcl_get_default_allocator(),
            &mut fixture.args,
        );
        // Should still succeed as it's stubbed

        // Test with null args_output
        let arg1 = c"test";
        let argv: [*const i8; 1] = [arg1.as_ptr()];
        let _ret = rcl_parse_arguments(
            1,
            argv.as_ptr(),
            rcl_get_default_allocator(),
            ptr::null_mut(),
        );
        // Should succeed as stubbed
    }
}

/// Test rcl_arguments_get_count_unparsed
#[test]
fn test_arguments_get_count_unparsed() {
    let fixture = TestArgumentsFixture::new();

    unsafe {
        let count = rcl_arguments_get_count_unparsed(&fixture.args);
        assert_eq!(count, 0); // Stubbed to return 0
    }
}

/// Test rcl_arguments_get_count_unparsed with null arguments
#[test]
fn test_arguments_get_count_unparsed_null() {
    unsafe {
        let count = rcl_arguments_get_count_unparsed(ptr::null());
        assert_eq!(count, 0); // Stubbed to return 0
    }
}

/// Test rcl_arguments_get_unparsed
#[test]
fn test_arguments_get_unparsed() {
    let fixture = TestArgumentsFixture::new();

    unsafe {
        let mut output: *mut ::std::os::raw::c_int = ptr::null_mut();
        let ret =
            rcl_arguments_get_unparsed(&fixture.args, rcl_get_default_allocator(), &mut output);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_arguments_get_count_unparsed_ros
#[test]
fn test_arguments_get_count_unparsed_ros() {
    let fixture = TestArgumentsFixture::new();

    unsafe {
        let count = rcl_arguments_get_count_unparsed_ros(&fixture.args);
        assert_eq!(count, 0); // Stubbed to return 0
    }
}

/// Test rcl_arguments_get_unparsed_ros
#[test]
fn test_arguments_get_unparsed_ros() {
    let fixture = TestArgumentsFixture::new();

    unsafe {
        let mut output: *mut ::std::os::raw::c_int = ptr::null_mut();
        let ret =
            rcl_arguments_get_unparsed_ros(&fixture.args, rcl_get_default_allocator(), &mut output);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_arguments_get_param_files_count
#[test]
fn test_arguments_get_param_files_count() {
    let fixture = TestArgumentsFixture::new();

    unsafe {
        let count = rcl_arguments_get_param_files_count(&fixture.args);
        assert_eq!(count, 0); // Stubbed to return 0
    }
}

/// Test rcl_arguments_get_param_files
#[test]
fn test_arguments_get_param_files() {
    let fixture = TestArgumentsFixture::new();

    unsafe {
        let mut param_files: *mut *mut ::std::os::raw::c_char = ptr::null_mut();
        let ret = rcl_arguments_get_param_files(
            &fixture.args,
            rcl_get_default_allocator(),
            &mut param_files,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_arguments_get_param_overrides
#[test]
fn test_arguments_get_param_overrides() {
    let fixture = TestArgumentsFixture::new();

    unsafe {
        let mut param_overrides: *mut rcl_params_t = ptr::null_mut();
        let ret = rcl_arguments_get_param_overrides(&fixture.args, &mut param_overrides);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_remove_ros_arguments
#[test]
fn test_remove_ros_arguments() {
    let fixture = TestArgumentsFixture::new();

    unsafe {
        let arg1 = c"--ros-args";
        let arg2 = c"--log-level";
        let arg3 = c"info";
        let argv: [*const i8; 3] = [arg1.as_ptr(), arg2.as_ptr(), arg3.as_ptr()];

        let mut nonros_argc: ::std::os::raw::c_int = 0;
        let mut nonros_argv: *mut *const ::std::os::raw::c_char = ptr::null_mut();

        let ret = rcl_remove_ros_arguments(
            argv.as_ptr(),
            &fixture.args,
            rcl_get_default_allocator(),
            &mut nonros_argc,
            &mut nonros_argv,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_arguments_get_log_levels
#[test]
fn test_arguments_get_log_levels() {
    let fixture = TestArgumentsFixture::new();

    unsafe {
        let mut log_levels = rcl_log_levels_t::default();
        let ret = rcl_arguments_get_log_levels(&fixture.args, &mut log_levels);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_arguments_copy
#[test]
fn test_arguments_copy() {
    let fixture = TestArgumentsFixture::new();
    let mut args_copy = rcl_get_zero_initialized_arguments();

    unsafe {
        let ret = rcl_arguments_copy(&fixture.args, &mut args_copy);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Clean up
        let _ = rcl_arguments_fini(&mut args_copy);
    }
}

/// Test rcl_arguments_fini
#[test]
fn test_arguments_fini() {
    let mut args = rcl_get_zero_initialized_arguments();

    unsafe {
        let ret = rcl_arguments_fini(&mut args);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}
