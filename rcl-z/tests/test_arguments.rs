// Ported from Open Source Robotics Foundation code (2018)
// https://github.com/ros2/rcl
// Licensed under the Apache License 2.0
// Copyright 2025 ZettaScale Technology

#![cfg(feature = "test-core")]
#![allow(unsafe_op_in_unsafe_fn)]
#![allow(clippy::needless_range_loop)]

use std::ffi::CString;
use std::ptr;

use rcl_z::{
    arguments::{
        rcl_arguments_copy, rcl_arguments_fini, rcl_arguments_get_count_unparsed,
        rcl_arguments_get_count_unparsed_ros,
        rcl_arguments_get_param_files, rcl_arguments_get_param_files_count,
        rcl_arguments_get_param_overrides, rcl_arguments_get_unparsed,
        rcl_arguments_get_unparsed_ros, rcl_get_zero_initialized_arguments, rcl_parse_arguments,
        rcl_remove_ros_arguments,
    },
    init::rcl_get_default_allocator,
    ros::{rcl_arguments_t, rcl_params_t, RCL_RET_ERROR, RCL_RET_INVALID_ARGUMENT, RCL_RET_OK},
};

/// Test fixture that provides initialized arguments
/// (Currently unused but kept for potential future use)
#[allow(dead_code)]
struct TestArgumentsFixture {
    args: rcl_arguments_t,
}

#[allow(dead_code)]
impl TestArgumentsFixture {
    fn new() -> Self {
        let args = rcl_get_zero_initialized_arguments();
        TestArgumentsFixture { args }
    }
}

#[allow(dead_code)]
impl Drop for TestArgumentsFixture {
    fn drop(&mut self) {
        unsafe {
            let _ = rcl_arguments_fini(&mut self.args);
        }
    }
}

/// Helper function to check if arguments are known ROS args (no unparsed)
unsafe fn are_known_ros_args(argv: &[&str]) -> bool {
    let argc = argv.len() as i32;
    let c_strings: Vec<CString> = argv.iter().map(|s| CString::new(*s).unwrap()).collect();
    let c_ptrs: Vec<*const i8> = c_strings.iter().map(|cs| cs.as_ptr()).collect();

    let mut parsed_args = rcl_get_zero_initialized_arguments();
    let ret = rcl_parse_arguments(
        argc,
        c_ptrs.as_ptr(),
        rcl_get_default_allocator(),
        &mut parsed_args,
    );

    if ret != RCL_RET_OK as i32 {
        let _ = rcl_arguments_fini(&mut parsed_args);
        return false;
    }

    let unparsed_count = rcl_arguments_get_count_unparsed(&parsed_args);
    let unparsed_ros_count = rcl_arguments_get_count_unparsed_ros(&parsed_args);
    let is_valid = unparsed_count == 0 && unparsed_ros_count == 0;

    let _ = rcl_arguments_fini(&mut parsed_args);
    is_valid
}

/// Helper function to check if arguments are valid ROS args (parse without error)
unsafe fn are_valid_ros_args(argv: &[&str]) -> bool {
    let argc = argv.len() as i32;
    let c_strings: Vec<CString> = argv.iter().map(|s| CString::new(*s).unwrap()).collect();
    let c_ptrs: Vec<*const i8> = c_strings.iter().map(|cs| cs.as_ptr()).collect();

    let mut parsed_args = rcl_get_zero_initialized_arguments();
    let ret = rcl_parse_arguments(
        argc,
        c_ptrs.as_ptr(),
        rcl_get_default_allocator(),
        &mut parsed_args,
    );

    let _ = rcl_arguments_fini(&mut parsed_args);
    ret == RCL_RET_OK as i32
}

/// Helper to check unparsed argument indices
unsafe fn expect_unparsed(args: &rcl_arguments_t, expected_indices: &[i32]) {
    let alloc = rcl_get_default_allocator();
    let actual_count = rcl_arguments_get_count_unparsed(args);

    assert_eq!(
        actual_count, expected_indices.len() as i32,
        "Expected {} unparsed args, got {}",
        expected_indices.len(), actual_count
    );

    if actual_count > 0 {
        let mut actual_unparsed: *mut i32 = ptr::null_mut();
        let ret = rcl_arguments_get_unparsed(args, alloc, &mut actual_unparsed);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!actual_unparsed.is_null());

        for i in 0..actual_count as usize {
            assert_eq!(
                *actual_unparsed.add(i), expected_indices[i],
                "Unparsed index {} mismatch", i
            );
        }

        alloc.deallocate.unwrap()(actual_unparsed as *mut _, alloc.state);
    }
}

/// Helper to check unparsed ROS argument indices
unsafe fn expect_unparsed_ros(args: &rcl_arguments_t, expected_indices: &[i32]) {
    let alloc = rcl_get_default_allocator();
    let actual_count = rcl_arguments_get_count_unparsed_ros(args);

    assert_eq!(
        actual_count, expected_indices.len() as i32,
        "Expected {} unparsed ROS args, got {}",
        expected_indices.len(), actual_count
    );

    if actual_count > 0 {
        let mut actual_unparsed_ros: *mut i32 = ptr::null_mut();
        let ret = rcl_arguments_get_unparsed_ros(args, alloc, &mut actual_unparsed_ros);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!actual_unparsed_ros.is_null());

        for i in 0..actual_count as usize {
            assert_eq!(
                *actual_unparsed_ros.add(i), expected_indices[i],
                "Unparsed ROS index {} mismatch", i
            );
        }

        alloc.deallocate.unwrap()(actual_unparsed_ros as *mut _, alloc.state);
    }
}

/// Test rcl_get_zero_initialized_arguments
#[test]
fn test_get_zero_initialized_arguments() {
    let args = rcl_get_zero_initialized_arguments();
    // Zero initialized should be default
    assert_eq!(args, rcl_arguments_t::default());
}

/// Test known vs unknown args
#[test]
fn test_check_known_vs_unknown_args() {
    unsafe {
        // Valid remap rules
        assert!(are_known_ros_args(&["--ros-args", "-r", "__node:=node_name"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "old_name:__node:=node_name"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "old_name:__node:=nodename123"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "__node:=nodename123"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "__ns:=/foo/bar"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "__ns:=/"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "_:=kq"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "nodename:__ns:=/foobar"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "foo:=bar"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "~/foo:=~/bar"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "/foo/bar:=bar"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "foo:=/bar"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "/foo123:=/bar123"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "node:/foo123:=/bar123"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "rostopic:=/foo/bar"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "rosservice:=baz"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "rostopic://rostopic:=rosservice"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "rostopic:///rosservice:=rostopic"]));
        assert!(are_known_ros_args(&["--ros-args", "-r", "rostopic:///foo/bar:=baz"]));

        // Valid param rules
        assert!(are_known_ros_args(&["--ros-args", "-p", "foo:=bar"]));
        assert!(are_known_ros_args(&["--ros-args", "-p", "qos_overrides./foo/bar.publisher.history:=keep_last"]));
        assert!(are_known_ros_args(&["--ros-args", "-p", "foo.bar:=bar"]));
        assert!(are_known_ros_args(&["--ros-args", "-p", "node:foo:=bar"]));
        assert!(are_known_ros_args(&["--ros-args", "-p", "fizz123:=buzz456"]));

        // Valid log level args
        assert!(are_known_ros_args(&["--ros-args", "--log-level", "UNSET"]));
        assert!(are_known_ros_args(&["--ros-args", "--log-level", "DEBUG"]));
        assert!(are_known_ros_args(&["--ros-args", "--log-level", "INFO"]));
        assert!(are_known_ros_args(&["--ros-args", "--log-level", "WARN"]));
        assert!(are_known_ros_args(&["--ros-args", "--log-level", "ERROR"]));
        assert!(are_known_ros_args(&["--ros-args", "--log-level", "FATAL"]));
        assert!(are_known_ros_args(&["--ros-args", "--log-level", "debug"]));
        assert!(are_known_ros_args(&["--ros-args", "--log-level", "Info"]));

        // Valid log config args
        assert!(are_known_ros_args(&["--ros-args", "--log-config-file", "file.config"]));
        assert!(are_known_ros_args(&["--ros-args", "--log-file-name", "filename"]));

        // Valid log enable/disable flags
        assert!(are_known_ros_args(&["--ros-args", "--enable-rosout-logs"]));
        assert!(are_known_ros_args(&["--ros-args", "--disable-rosout-logs"]));
        assert!(are_known_ros_args(&["--ros-args", "--enable-stdout-logs"]));
        assert!(are_known_ros_args(&["--ros-args", "--disable-stdout-logs"]));
        assert!(are_known_ros_args(&["--ros-args", "--enable-external-lib-logs"]));
        assert!(are_known_ros_args(&["--ros-args", "--disable-external-lib-logs"]));

        // Unknown ROS args (should have unparsed_ros)
        assert!(!are_known_ros_args(&["--ros-args", "--custom-ros-arg"]));
        assert!(!are_known_ros_args(&["--ros-args", "__node:=node_name"]));  // Missing -r flag
        assert!(!are_known_ros_args(&["--ros-args", "old_name:__node:=node_name"]));  // Missing -r flag
        assert!(!are_known_ros_args(&["--ros-args", "/foo/bar:=bar"]));  // Missing -r flag
        assert!(!are_known_ros_args(&["--ros-args", "foo:=/bar"]));  // Missing -r flag
        assert!(!are_known_ros_args(&["--ros-args", "file_name.yaml"]));  // Missing --params-file flag
        assert!(!are_known_ros_args(&["--ros-args", "--log", "foo"]));  // Invalid flag
        assert!(!are_known_ros_args(&["--ros-args", "--loglevel", "foo"]));  // Invalid flag
        assert!(!are_known_ros_args(&["--ros-args", "--logfile-name", "filename"]));  // Invalid flag
        assert!(!are_known_ros_args(&["--ros-args", "--log-filename", "filename"]));  // Invalid flag
        assert!(!are_known_ros_args(&["--ros-args", "stdout-logs"]));  // Invalid flag
        assert!(!are_known_ros_args(&["--ros-args", "external-lib-logs"]));  // Invalid flag
    }
}

/// Test valid vs invalid args
#[test]
fn test_check_valid_vs_invalid_args() {
    unsafe {
        // Valid comprehensive example
        assert!(are_valid_ros_args(&[
            "--ros-args", "-p", "foo:=bar", "-r", "__node:=node_name",
            "--log-level", "INFO", "--log-config-file", "file.config", "--log-file-name", "filename"
        ]));

        // ROS args unknown to rcl are not (necessarily) invalid
        assert!(are_valid_ros_args(&["--ros-args", "--custom-ros-arg"]));

        // Invalid remap rules (missing argument)
        assert!(!are_valid_ros_args(&["--ros-args", "-r"]));
        assert!(!are_valid_ros_args(&["--ros-args", "--remap"]));

        // Invalid remap rules (bad syntax)
        assert!(!are_valid_ros_args(&["--ros-args", "-r", ":"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "1"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "~"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", ":="]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "foo:="]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", ":=bar"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "::="]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "1:="]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "~:="]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "__node:="]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "__node:=/foo/bar"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "__ns:="]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "__ns:=foo"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", ":__node:=nodename"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "~:__node:=nodename"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "}foo:=/bar"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "f oo:=/bar"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "foo:=/b ar"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "f{oo:=/bar"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "foo:=/b}ar"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "rostopic://:=rosservice"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-r", "rostopic::=rosservice"]));

        // Invalid param rules (missing argument)
        assert!(!are_valid_ros_args(&["--ros-args", "-p"]));

        // Invalid param rules (bad syntax)
        assert!(!are_valid_ros_args(&["--ros-args", "-p", ":="]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", "foo:="]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", ":=bar"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", ":"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", "1"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", "~"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", "::="]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", "1:="]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", "~:="]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", "__node:="]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", "__node:=/foo/bar"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", "__ns:=foo"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", ":__node:=nodename"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", "~:__node:=nodename"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", "}foo:=/bar"]));
        assert!(!are_valid_ros_args(&["--ros-args", "--param", "}foo:=/bar"]));
        assert!(!are_valid_ros_args(&["--ros-args", "-p", "f oo:=/bar"]));
        assert!(!are_valid_ros_args(&["--ros-args", "--param", "f oo:=/bar"]));

        // Invalid enclave (missing argument)
        assert!(!are_valid_ros_args(&["--ros-args", "-e"]));
        assert!(!are_valid_ros_args(&["--ros-args", "--enclave"]));

        // Invalid params-file (missing argument)
        assert!(!are_valid_ros_args(&["--ros-args", "--params-file"]));

        // Invalid log-config-file (missing argument)
        assert!(!are_valid_ros_args(&["--ros-args", "--log-config-file"]));

        // Invalid log-level (missing argument or bad value)
        assert!(!are_valid_ros_args(&["--ros-args", "--log-level"]));
        assert!(!are_valid_ros_args(&["--ros-args", "--log-level", "foo"]));

        // Invalid log-file-name (missing argument)
        assert!(!are_valid_ros_args(&["--ros-args", "--log-file-name"]));
    }
}

/// Test parsing with no arguments
#[test]
fn test_no_args() {
    unsafe {
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(0, ptr::null(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(rcl_arguments_get_count_unparsed(&parsed_args), 0);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(&parsed_args), 0);
        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test parsing with null argv but non-zero argc
#[test]
fn test_null_args() {
    unsafe {
        let argc = 1;
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, ptr::null(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
    }
}

/// Test parsing with negative argc
#[test]
fn test_negative_args() {
    unsafe {
        let argc = -1;
        let argv = [c"process_name".as_ptr()];
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
    }
}

/// Test null args_output
#[test]
fn test_null_args_output() {
    unsafe {
        let argv = [c"process_name".as_ptr()];
        let argc = argv.len() as i32;
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
    }
}

/// Test parsing with no ROS args
#[test]
fn test_no_ros_args() {
    unsafe {
        let argv = [c"process_name".as_ptr()];
        let argc = argv.len() as i32;
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);
        expect_unparsed(&parsed_args, &[0]);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(&parsed_args), 0);
        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test zero ROS args (just --ros-args flag)
#[test]
fn test_zero_ros_args() {
    unsafe {
        let argv = [c"process_name".as_ptr(), c"--ros-args".as_ptr()];
        let argc = argv.len() as i32;
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);
        expect_unparsed(&parsed_args, &[0]);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(&parsed_args), 0);
        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test zero ROS args with trailing dashes
#[test]
fn test_zero_ros_args_w_trailing_dashes() {
    unsafe {
        let argv = [c"process_name".as_ptr(), c"--ros-args".as_ptr(), c"--".as_ptr()];
        let argc = argv.len() as i32;
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);
        expect_unparsed(&parsed_args, &[0]);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(&parsed_args), 0);
        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test remap arguments
#[test]
fn test_remap() {
    unsafe {
        let argv = [
            c"process_name".as_ptr(),
            c"--ros-args".as_ptr(),
            c"-r".as_ptr(),
            c"/foo/bar:=/fiz/buz".as_ptr(),
            c"--remap".as_ptr(),
            c"foo:=/baz".as_ptr(),
        ];
        let argc = argv.len() as i32;
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);
        expect_unparsed(&parsed_args, &[0]);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(&parsed_args), 0);
        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test one remap with two --ros-args flags
#[test]
fn test_one_remap_two_ros_args() {
    unsafe {
        let argv = [
            c"process_name".as_ptr(),
            c"--ros-args".as_ptr(),
            c"--ros-args".as_ptr(),
            c"-r".as_ptr(),
            c"/foo/bar:=/fiz/buz".as_ptr(),
        ];
        let argc = argv.len() as i32;
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);
        expect_unparsed(&parsed_args, &[0]);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(&parsed_args), 0);
        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test one remap with trailing dashes
#[test]
fn test_one_remap_w_trailing_dashes() {
    unsafe {
        let argv = [
            c"process_name".as_ptr(),
            c"--ros-args".as_ptr(),
            c"-r".as_ptr(),
            c"/foo/bar:=/fiz/buz".as_ptr(),
            c"--".as_ptr(),
        ];
        let argc = argv.len() as i32;
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);
        expect_unparsed(&parsed_args, &[0]);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(&parsed_args), 0);
        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test one remap with two trailing dashes
#[test]
fn test_one_remap_w_two_trailing_dashes() {
    unsafe {
        let argv = [
            c"process_name".as_ptr(),
            c"--ros-args".as_ptr(),
            c"-r".as_ptr(),
            c"/foo/bar:=/fiz/buz".as_ptr(),
            c"--".as_ptr(),
            c"--".as_ptr(),
        ];
        let argc = argv.len() as i32;
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);
        expect_unparsed(&parsed_args, &[0, 5]);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(&parsed_args), 0);
        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test mix of valid and invalid rules
#[test]
fn test_mix_valid_invalid_rules() {
    unsafe {
        let argv = [
            c"process_name".as_ptr(),
            c"--ros-args".as_ptr(),
            c"/foo/bar:=".as_ptr(),  // Invalid (unparsed ROS)
            c"-r".as_ptr(),
            c"bar:=/fiz/buz".as_ptr(),  // Valid
            c"}bar:=fiz".as_ptr(),  // Invalid (unparsed ROS)
            c"--".as_ptr(),
            c"arg".as_ptr(),  // Unparsed
        ];
        let argc = argv.len() as i32;
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);
        expect_unparsed(&parsed_args, &[0, 7]);
        expect_unparsed_ros(&parsed_args, &[2, 5]);
        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test rcl_arguments_copy
#[test]
fn test_copy() {
    unsafe {
        let argv = [
            c"process_name".as_ptr(),
            c"--ros-args".as_ptr(),
            c"/foo/bar:=".as_ptr(),
            c"-r".as_ptr(),
            c"bar:=/fiz/buz".as_ptr(),
            c"-r".as_ptr(),
            c"__ns:=/foo".as_ptr(),
            c"--".as_ptr(),
            c"arg".as_ptr(),
        ];
        let argc = argv.len() as i32;
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut copied_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_arguments_copy(&parsed_args, &mut copied_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Can't copy to non-empty
        let ret = rcl_arguments_copy(&parsed_args, &mut copied_args);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        expect_unparsed(&parsed_args, &[0, 8]);
        expect_unparsed_ros(&parsed_args, &[2]);

        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);

        expect_unparsed(&copied_args, &[0, 8]);
        expect_unparsed_ros(&copied_args, &[2]);

        assert_eq!(rcl_arguments_fini(&mut copied_args), RCL_RET_OK as i32);
    }
}

/// Test copy with no ROS args
#[test]
fn test_copy_no_ros_args() {
    unsafe {
        let argv = [
            c"process_name".as_ptr(),
            c"--ros-args".as_ptr(),
            c"--".as_ptr(),
            c"arg".as_ptr(),
            c"--ros-args".as_ptr(),
        ];
        let argc = argv.len() as i32;
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut copied_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_arguments_copy(&parsed_args, &mut copied_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        expect_unparsed(&parsed_args, &[0, 3]);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(&parsed_args), 0);

        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);

        expect_unparsed(&copied_args, &[0, 3]);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(&copied_args), 0);

        assert_eq!(rcl_arguments_fini(&mut copied_args), RCL_RET_OK as i32);
    }
}

/// Test copy with no args at all
#[test]
fn test_copy_no_args() {
    unsafe {
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(0, ptr::null(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(rcl_arguments_get_count_unparsed(&parsed_args), 0);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(&parsed_args), 0);

        let mut copied_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_arguments_copy(&parsed_args, &mut copied_args);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(rcl_arguments_get_count_unparsed(&copied_args), 0);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(&copied_args), 0);

        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
        assert_eq!(rcl_arguments_fini(&mut copied_args), RCL_RET_OK as i32);
    }
}

/// Test two namespace remaps
#[test]
fn test_two_namespace() {
    unsafe {
        let argv = [
            c"process_name".as_ptr(),
            c"--ros-args".as_ptr(),
            c"-r".as_ptr(),
            c"__ns:=/foo/bar".as_ptr(),
            c"-r".as_ptr(),
            c"__ns:=/fiz/buz".as_ptr(),
        ];
        let argc = argv.len() as i32;
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);
        expect_unparsed(&parsed_args, &[0]);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(&parsed_args), 0);
        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test uninitialized parsed args
#[test]
fn test_uninitialized_parsed_args() {
    unsafe {
        let argv = [c"process_name".as_ptr()];
        let argc = argv.len() as i32;
        let not_null: i32 = 1;
        let mut parsed_args = rcl_arguments_t {
            impl_: &not_null as *const i32 as *mut _,
        };
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
    }
}

/// Test double parse
#[test]
fn test_double_parse() {
    unsafe {
        let argv = [
            c"process_name".as_ptr(),
            c"--ros-args".as_ptr(),
            c"-r".as_ptr(),
            c"__ns:=/foo/bar".as_ptr(),
            c"-r".as_ptr(),
            c"__ns:=/fiz/buz".as_ptr(),
        ];
        let argc = argv.len() as i32;
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Try to parse again (should fail)
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test rcl_arguments_fini with null
#[test]
fn test_fini_null() {
    unsafe {
        let ret = rcl_arguments_fini(ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
    }
}

/// Test rcl_arguments_fini with null impl
#[test]
fn test_fini_impl_null() {
    unsafe {
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        parsed_args.impl_ = ptr::null_mut();
        let ret = rcl_arguments_fini(&mut parsed_args);
        assert_eq!(ret, RCL_RET_ERROR as i32);
    }
}

/// Test rcl_arguments_fini twice
#[test]
fn test_fini_twice() {
    unsafe {
        let argv = [c"process_name".as_ptr()];
        let argc = argv.len() as i32;
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), rcl_get_default_allocator(), &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_ERROR as i32);
    }
}

/// Test bad rcl_remove_ros_arguments parameters
#[test]
fn test_bad_remove_ros_args() {
    unsafe {
        let argv = [c"process_name".as_ptr()];
        let argc = argv.len() as i32;

        let default_allocator = rcl_get_default_allocator();
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), default_allocator, &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut nonros_argv: *mut *const i8 = ptr::null_mut();
        let mut nonros_argc: i32 = 0;

        // Test null argv (should fail because there are unparsed args)
        let ret = rcl_remove_ros_arguments(
            ptr::null(),
            &parsed_args,
            default_allocator,
            &mut nonros_argc,
            &mut nonros_argv,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test null args
        let ret = rcl_remove_ros_arguments(
            argv.as_ptr(),
            ptr::null(),
            default_allocator,
            &mut nonros_argc,
            &mut nonros_argv,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test null nonros_argc
        let ret = rcl_remove_ros_arguments(
            argv.as_ptr(),
            &parsed_args,
            default_allocator,
            ptr::null_mut(),
            &mut nonros_argv,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test null nonros_argv
        let ret = rcl_remove_ros_arguments(
            argv.as_ptr(),
            &parsed_args,
            default_allocator,
            &mut nonros_argc,
            ptr::null_mut(),
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test non-zero initialized output
        let stack_allocated_nonros_argv = [c"--foo".as_ptr(), c"--bar".as_ptr()];
        let mut initialized_nonros_argv: *mut *const i8 = stack_allocated_nonros_argv.as_ptr() as *mut _;
        let mut initialized_nonros_argc = stack_allocated_nonros_argv.len() as i32;

        let ret = rcl_remove_ros_arguments(
            argv.as_ptr(),
            &parsed_args,
            default_allocator,
            &mut initialized_nonros_argc,
            &mut initialized_nonros_argv,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test rcl_remove_ros_arguments
#[test]
fn test_remove_ros_args() {
    unsafe {
        let argv = [
            c"process_name".as_ptr(),
            c"-d".as_ptr(),
            c"--ros-args".as_ptr(),
            c"-r".as_ptr(),
            c"__ns:=/foo/bar".as_ptr(),
            c"-r".as_ptr(),
            c"__ns:=/fiz/buz".as_ptr(),
            c"--".as_ptr(),
            c"--foo=bar".as_ptr(),
            c"--baz".as_ptr(),
            c"--ros-args".as_ptr(),
            c"--ros-args".as_ptr(),
            c"-p".as_ptr(),
            c"bar:=baz".as_ptr(),
            c"--".as_ptr(),
            c"--".as_ptr(),
            c"arg".as_ptr(),
        ];
        let argc = argv.len() as i32;

        let alloc = rcl_get_default_allocator();
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), alloc, &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut nonros_argc = 0;
        let mut nonros_argv: *mut *const i8 = ptr::null_mut();

        let ret = rcl_remove_ros_arguments(
            argv.as_ptr(),
            &parsed_args,
            alloc,
            &mut nonros_argc,
            &mut nonros_argv,
        );

        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(nonros_argc, 6);

        // Verify the non-ROS arguments
        assert_eq!(std::ffi::CStr::from_ptr(*nonros_argv.add(0)).to_str().unwrap(), "process_name");
        assert_eq!(std::ffi::CStr::from_ptr(*nonros_argv.add(1)).to_str().unwrap(), "-d");
        assert_eq!(std::ffi::CStr::from_ptr(*nonros_argv.add(2)).to_str().unwrap(), "--foo=bar");
        assert_eq!(std::ffi::CStr::from_ptr(*nonros_argv.add(3)).to_str().unwrap(), "--baz");
        assert_eq!(std::ffi::CStr::from_ptr(*nonros_argv.add(4)).to_str().unwrap(), "--");
        assert_eq!(std::ffi::CStr::from_ptr(*nonros_argv.add(5)).to_str().unwrap(), "arg");

        alloc.deallocate.unwrap()(nonros_argv as *mut _, alloc.state);
        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test remove_ros_args when only ROS args exist
#[test]
fn test_remove_ros_args_if_ros_only() {
    unsafe {
        let argv = [c"--ros-args".as_ptr(), c"--disable-rosout-logs".as_ptr()];
        let argc = argv.len() as i32;

        let alloc = rcl_get_default_allocator();
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, argv.as_ptr(), alloc, &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut nonros_argc = 0;
        let mut nonros_argv: *mut *const i8 = ptr::null_mut();

        let ret = rcl_remove_ros_arguments(
            argv.as_ptr(),
            &parsed_args,
            alloc,
            &mut nonros_argc,
            &mut nonros_argv,
        );

        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(nonros_argc, 0);
        assert!(nonros_argv.is_null());

        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test remove_ros_args when no args
#[test]
fn test_remove_ros_args_if_no_args() {
    unsafe {
        let argc = 0;

        let alloc = rcl_get_default_allocator();
        let mut parsed_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_parse_arguments(argc, ptr::null(), alloc, &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut nonros_argc = 0;
        let mut nonros_argv: *mut *const i8 = ptr::null_mut();

        let ret = rcl_remove_ros_arguments(
            ptr::null(),
            &parsed_args,
            alloc,
            &mut nonros_argc,
            &mut nonros_argv,
        );

        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(nonros_argc, 0);
        assert!(nonros_argv.is_null());

        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test parameter file count (zero)
#[test]
fn test_param_argument_zero() {
    unsafe {
        let argv = [
            c"process_name".as_ptr(),
            c"--ros-args".as_ptr(),
            c"-r".as_ptr(),
            c"__ns:=/namespace".as_ptr(),
            c"random:=arg".as_ptr(),
        ];
        let argc = argv.len() as i32;

        let alloc = rcl_get_default_allocator();
        let mut parsed_args = rcl_get_zero_initialized_arguments();

        let ret = rcl_parse_arguments(argc, argv.as_ptr(), alloc, &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        let parameter_filecount = rcl_arguments_get_param_files_count(&parsed_args);
        assert_eq!(parameter_filecount, 0);

        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test parameter file count (single)
#[test]
fn test_param_argument_single() {
    unsafe {
        let test_params_path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/resources/test_arguments/test_parameters.1.yaml");
        let test_params_cstr = CString::new(test_params_path).unwrap();

        let argv = [
            c"process_name".as_ptr(),
            c"--ros-args".as_ptr(),
            c"-r".as_ptr(),
            c"__ns:=/namespace".as_ptr(),
            c"random:=arg".as_ptr(),
            c"--params-file".as_ptr(),
            test_params_cstr.as_ptr(),
        ];
        let argc = argv.len() as i32;

        let alloc = rcl_get_default_allocator();
        let mut parsed_args = rcl_get_zero_initialized_arguments();

        let ret = rcl_parse_arguments(argc, argv.as_ptr(), alloc, &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        let parameter_filecount = rcl_arguments_get_param_files_count(&parsed_args);
        assert_eq!(parameter_filecount, 1);

        let mut parameter_files: *mut *mut i8 = ptr::null_mut();
        let ret = rcl_arguments_get_param_files(&parsed_args, alloc, &mut parameter_files);
        assert_eq!(ret, RCL_RET_OK as i32);

        let file_path = std::ffi::CStr::from_ptr(*parameter_files.add(0)).to_str().unwrap();
        assert_eq!(file_path, test_params_path);

        // Cleanup
        for i in 0..parameter_filecount {
            alloc.deallocate.unwrap()(*parameter_files.add(i as usize) as *mut _, alloc.state);
        }
        alloc.deallocate.unwrap()(parameter_files as *mut _, alloc.state);

        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test parameter file count (multiple)
#[test]
fn test_param_argument_multiple() {
    unsafe {
        let test_params_path1 = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/resources/test_arguments/test_parameters.1.yaml");
        let test_params_path2 = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/resources/test_arguments/test_parameters.2.yaml");
        let test_params_cstr1 = CString::new(test_params_path1).unwrap();
        let test_params_cstr2 = CString::new(test_params_path2).unwrap();

        let argv = [
            c"process_name".as_ptr(),
            c"--ros-args".as_ptr(),
            c"--params-file".as_ptr(),
            test_params_cstr1.as_ptr(),
            c"-r".as_ptr(),
            c"__ns:=/namespace".as_ptr(),
            c"random:=arg".as_ptr(),
            c"--params-file".as_ptr(),
            test_params_cstr2.as_ptr(),
        ];
        let argc = argv.len() as i32;

        let alloc = rcl_get_default_allocator();
        let mut parsed_args = rcl_get_zero_initialized_arguments();

        let ret = rcl_parse_arguments(argc, argv.as_ptr(), alloc, &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        let parameter_filecount = rcl_arguments_get_param_files_count(&parsed_args);
        assert_eq!(parameter_filecount, 2);

        let mut parameter_files: *mut *mut i8 = ptr::null_mut();
        let ret = rcl_arguments_get_param_files(&parsed_args, alloc, &mut parameter_files);
        assert_eq!(ret, RCL_RET_OK as i32);

        let file_path1 = std::ffi::CStr::from_ptr(*parameter_files.add(0)).to_str().unwrap();
        let file_path2 = std::ffi::CStr::from_ptr(*parameter_files.add(1)).to_str().unwrap();
        assert_eq!(file_path1, test_params_path1);
        assert_eq!(file_path2, test_params_path2);

        // Cleanup
        for i in 0..parameter_filecount {
            alloc.deallocate.unwrap()(*parameter_files.add(i as usize) as *mut _, alloc.state);
        }
        alloc.deallocate.unwrap()(parameter_files as *mut _, alloc.state);

        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test param arguments copy
#[test]
fn test_param_arguments_copy() {
    unsafe {
        let test_params_path1 = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/resources/test_arguments/test_parameters.1.yaml");
        let test_params_path2 = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/resources/test_arguments/test_parameters.2.yaml");
        let test_params_cstr1 = CString::new(test_params_path1).unwrap();
        let test_params_cstr2 = CString::new(test_params_path2).unwrap();

        let argv = [
            c"process_name".as_ptr(),
            c"--ros-args".as_ptr(),
            c"--params-file".as_ptr(),
            test_params_cstr1.as_ptr(),
            c"-r".as_ptr(),
            c"__ns:=/namespace".as_ptr(),
            c"random:=arg".as_ptr(),
            c"--params-file".as_ptr(),
            test_params_cstr2.as_ptr(),
        ];
        let argc = argv.len() as i32;

        let alloc = rcl_get_default_allocator();
        let mut parsed_args = rcl_get_zero_initialized_arguments();

        let ret = rcl_parse_arguments(argc, argv.as_ptr(), alloc, &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        let parameter_filecount = rcl_arguments_get_param_files_count(&parsed_args);
        assert_eq!(parameter_filecount, 2);

        let mut copied_args = rcl_get_zero_initialized_arguments();
        let ret = rcl_arguments_copy(&parsed_args, &mut copied_args);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(rcl_arguments_get_param_files_count(&copied_args), 2);

        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
        assert_eq!(rcl_arguments_fini(&mut copied_args), RCL_RET_OK as i32);
    }
}

/// Test no param overrides
#[test]
fn test_no_param_overrides() {
    unsafe {
        let argv = [c"process_name".as_ptr()];
        let argc = argv.len() as i32;

        let alloc = rcl_get_default_allocator();
        let mut parsed_args = rcl_get_zero_initialized_arguments();

        let ret = rcl_parse_arguments(argc, argv.as_ptr(), alloc, &mut parsed_args);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Test null output pointer
        let ret = rcl_arguments_get_param_overrides(&parsed_args, ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test null args
        let mut params: *mut rcl_params_t = ptr::null_mut();
        let ret = rcl_arguments_get_param_overrides(ptr::null(), &mut params);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test uninitialized args
        let empty_parsed_arg = rcl_get_zero_initialized_arguments();
        let ret = rcl_arguments_get_param_overrides(&empty_parsed_arg, &mut params);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test non-null output pointer
        let preallocated_params: rcl_params_t = rcl_params_t::default();
        let mut params_ptr = &preallocated_params as *const _ as *mut rcl_params_t;
        let ret = rcl_arguments_get_param_overrides(&parsed_args, &mut params_ptr);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test normal case (should return NULL since no overrides)
        let mut params: *mut rcl_params_t = ptr::null_mut();
        let ret = rcl_arguments_get_param_overrides(&parsed_args, &mut params);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(params.is_null());

        assert_eq!(rcl_arguments_fini(&mut parsed_args), RCL_RET_OK as i32);
    }
}

/// Test bad params get counts
#[test]
fn test_bad_params_get_counts() {
    unsafe {
        let parsed_args = rcl_get_zero_initialized_arguments();

        assert_eq!(rcl_arguments_get_count_unparsed(ptr::null()), -1);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(ptr::null()), -1);
        assert_eq!(rcl_arguments_get_param_files_count(ptr::null()), -1);
        assert_eq!(rcl_arguments_get_count_unparsed(&parsed_args), -1);
        assert_eq!(rcl_arguments_get_count_unparsed_ros(&parsed_args), -1);
        assert_eq!(rcl_arguments_get_param_files_count(&parsed_args), -1);
    }
}

/// Test empty unparsed
#[test]
fn test_empty_unparsed() {
    unsafe {
        let allocator = rcl_get_default_allocator();
        let empty_parsed_args = rcl_get_zero_initialized_arguments();
        let mut actual_unparsed: *mut i32 = ptr::null_mut();
        let mut actual_unparsed_ros: *mut i32 = ptr::null_mut();

        let ret = rcl_arguments_get_unparsed(&empty_parsed_args, allocator, &mut actual_unparsed);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        let ret = rcl_arguments_get_unparsed_ros(&empty_parsed_args, allocator, &mut actual_unparsed_ros);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
    }
}
