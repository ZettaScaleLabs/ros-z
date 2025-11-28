// Copyright 2025 ZettaScale Technology
// SPDX-License-Identifier: Apache-2.0
//
// Ported from ros2/rcl:
// Copyright 2017 Open Source Robotics Foundation, Inc.

#![cfg(feature = "test-core")]

use std::{
    ffi::{CStr, CString},
    ptr,
};

use rcl_z::{init::rcl_get_default_allocator, node::rcl_expand_topic_name, ros::*};

/// Helper to convert C string to Rust string
unsafe fn c_str_to_string(ptr: *const std::os::raw::c_char) -> String {
    if ptr.is_null() {
        return String::new();
    }
    unsafe { CStr::from_ptr(ptr).to_string_lossy().to_string() }
}

/// Helper to deallocate a C string
unsafe fn deallocate_c_string(ptr: *mut std::os::raw::c_char) {
    if !ptr.is_null() {
        unsafe {
            let _ = CString::from_raw(ptr);
        }
    }
}

#[test]
fn test_expand_topic_name_normal() {
    unsafe {
        let allocator = rcl_get_default_allocator();
        let subs = ptr::null(); // Not using substitutions in our implementation

        // {node}/chatter example - Note: our implementation doesn't support substitutions yet
        // So we test basic expansion instead
        let topic = CString::new("chatter").unwrap();
        let node = CString::new("my_node").unwrap();
        let ns = CString::new("/my_ns").unwrap();
        let mut expanded_topic: *mut std::os::raw::c_char = ptr::null_mut();

        let ret = rcl_expand_topic_name(
            topic.as_ptr(),
            node.as_ptr(),
            ns.as_ptr(),
            subs,
            allocator,
            &mut expanded_topic,
        );

        assert_eq!(ret, RCL_RET_OK as i32, "Failed to expand topic name");
        assert!(!expanded_topic.is_null(), "Expanded topic is null");

        let result = c_str_to_string(expanded_topic);
        assert_eq!(
            result, "/my_ns/chatter",
            "Expected /my_ns/chatter, got {}",
            result
        );

        deallocate_c_string(expanded_topic);
    }
}

#[test]
fn test_expand_topic_name_invalid_arguments() {
    unsafe {
        let allocator = rcl_get_default_allocator();
        let subs = ptr::null();

        let topic = CString::new("chatter").unwrap();
        let node = CString::new("my_node").unwrap();
        let ns = CString::new("/my_ns").unwrap();
        let mut expanded_topic: *mut std::os::raw::c_char = ptr::null_mut();

        // Pass null for topic string
        let ret = rcl_expand_topic_name(
            ptr::null(),
            node.as_ptr(),
            ns.as_ptr(),
            subs,
            allocator,
            &mut expanded_topic,
        );
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Expected RCL_RET_INVALID_ARGUMENT for null topic"
        );

        // Pass null for node name
        let ret = rcl_expand_topic_name(
            topic.as_ptr(),
            ptr::null(),
            ns.as_ptr(),
            subs,
            allocator,
            &mut expanded_topic,
        );
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Expected RCL_RET_INVALID_ARGUMENT for null node"
        );

        // Pass null for namespace
        let ret = rcl_expand_topic_name(
            topic.as_ptr(),
            node.as_ptr(),
            ptr::null(),
            subs,
            allocator,
            &mut expanded_topic,
        );
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Expected RCL_RET_INVALID_ARGUMENT for null namespace"
        );

        // Pass null for output pointer
        let ret = rcl_expand_topic_name(
            topic.as_ptr(),
            node.as_ptr(),
            ns.as_ptr(),
            subs,
            allocator,
            ptr::null_mut(),
        );
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Expected RCL_RET_INVALID_ARGUMENT for null output"
        );

        // Pass invalid topic name (with spaces)
        let invalid_topic = CString::new("white space").unwrap();
        let ret = rcl_expand_topic_name(
            invalid_topic.as_ptr(),
            node.as_ptr(),
            ns.as_ptr(),
            subs,
            allocator,
            &mut expanded_topic,
        );
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Expected RCL_RET_INVALID_ARGUMENT for invalid topic"
        );

        // Pass invalid node name (starts with /)
        let invalid_node = CString::new("/invalid_node").unwrap();
        let ret = rcl_expand_topic_name(
            topic.as_ptr(),
            invalid_node.as_ptr(),
            ns.as_ptr(),
            subs,
            allocator,
            &mut expanded_topic,
        );
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Expected RCL_RET_INVALID_ARGUMENT for invalid node name"
        );

        // Pass invalid namespace (with spaces)
        let invalid_ns = CString::new("white space").unwrap();
        let ret = rcl_expand_topic_name(
            topic.as_ptr(),
            node.as_ptr(),
            invalid_ns.as_ptr(),
            subs,
            allocator,
            &mut expanded_topic,
        );
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Expected RCL_RET_INVALID_ARGUMENT for invalid namespace"
        );
    }
}

#[test]
fn test_expand_topic_name_various_valid_topics() {
    unsafe {
        let allocator = rcl_get_default_allocator();
        let subs = ptr::null();

        // Test cases: (input_topic, node_name, namespace, expected_result)
        let test_cases = vec![
            // Absolute topics
            ("/chatter", "my_node", "/my_ns", "/chatter"),
            // Relative topics
            ("chatter", "my_node", "/my_ns", "/my_ns/chatter"),
            ("chatter", "my_node", "/", "/chatter"),
            // Private topics (~)
            ("~", "my_node", "/", "/my_node"),
            ("~", "my_node", "/my_ns", "/my_ns/my_node"),
            ("~/ping", "my_node", "/", "/my_node/ping"),
            ("~/ping", "my_node", "/my_ns", "/my_ns/my_node/ping"),
        ];

        for (topic_str, node_str, ns_str, expected) in test_cases {
            let topic = CString::new(topic_str).unwrap();
            let node = CString::new(node_str).unwrap();
            let ns = CString::new(ns_str).unwrap();
            let mut expanded_topic: *mut std::os::raw::c_char = ptr::null_mut();

            let ret = rcl_expand_topic_name(
                topic.as_ptr(),
                node.as_ptr(),
                ns.as_ptr(),
                subs,
                allocator,
                &mut expanded_topic,
            );

            assert_eq!(
                ret, RCL_RET_OK as i32,
                "Failed to expand topic '{}' with node '{}' and namespace '{}'",
                topic_str, node_str, ns_str
            );

            let result = c_str_to_string(expanded_topic);
            assert_eq!(
                result, expected,
                "Expanding '{}' with node '{}' and namespace '{}': expected '{}', got '{}'",
                topic_str, node_str, ns_str, expected, result
            );

            deallocate_c_string(expanded_topic);
        }
    }
}
