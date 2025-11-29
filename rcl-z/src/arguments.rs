// Ported from Open Source Robotics Foundation code (2018)
// https://github.com/ros2/rcl
// Licensed under the Apache License 2.0
// Copyright 2025 ZettaScale Technology

#![allow(unused)]
#![allow(unsafe_op_in_unsafe_fn)]

use crate::ros::*;
use crate::utils::parse_args;
use std::ffi::{CStr, CString};
use std::ptr;
use std::os::raw::{c_char, c_int};

/// Simple error handling - just log for now
fn rcl_set_error_string(msg: &str) {
    tracing::error!("RCL Error: {}", msg);
}

// ROS argument flags
const RCL_ROS_ARGS_FLAG: &str = "--ros-args";
const RCL_ROS_ARGS_EXPLICIT_END_TOKEN: &str = "--";
const RCL_PARAM_FLAG: &str = "--param";
const RCL_SHORT_PARAM_FLAG: &str = "-p";
const RCL_PARAM_FILE_FLAG: &str = "--params-file";
const RCL_REMAP_FLAG: &str = "--remap";
const RCL_SHORT_REMAP_FLAG: &str = "-r";
const RCL_ENCLAVE_FLAG: &str = "--enclave";
const RCL_SHORT_ENCLAVE_FLAG: &str = "-e";
const RCL_LOG_LEVEL_FLAG: &str = "--log-level";
const RCL_LOG_CONFIG_FILE_FLAG: &str = "--log-config-file";
const RCL_LOG_FILE_NAME_FLAG: &str = "--log-file-name";
const RCL_ENABLE_STDOUT_LOGS_FLAG: &str = "--enable-stdout-logs";
const RCL_DISABLE_STDOUT_LOGS_FLAG: &str = "--disable-stdout-logs";
const RCL_ENABLE_ROSOUT_LOGS_FLAG: &str = "--enable-rosout-logs";
const RCL_DISABLE_ROSOUT_LOGS_FLAG: &str = "--disable-rosout-logs";
const RCL_ENABLE_EXTERNAL_LIB_LOGS_FLAG: &str = "--enable-external-lib-logs";
const RCL_DISABLE_EXTERNAL_LIB_LOGS_FLAG: &str = "--disable-external-lib-logs";

/// Internal implementation of rcl_arguments_t
pub struct ArgumentsImpl {
    /// Allocator used to allocate objects in this struct
    allocator: rcl_allocator_t,

    /// Array of indices to unknown ROS specific arguments
    unparsed_ros_args: Vec<c_int>,

    /// Array of indices to non-ROS arguments
    unparsed_args: Vec<c_int>,

    /// Parameter file paths
    parameter_files: Vec<CString>,

    /// Enclave (if specified)
    enclave: Option<CString>,

    /// External log config file
    external_log_config_file: Option<CString>,

    /// External log file name prefix
    external_log_file_name_prefix: Option<CString>,

    /// Log levels
    log_levels: rcl_log_levels_t,

    /// Log stdout disabled flag
    log_stdout_disabled: bool,

    /// Log rosout disabled flag
    log_rosout_disabled: bool,

    /// Log external lib disabled flag
    log_ext_lib_disabled: bool,

    /// Parsed remap rules (simplified - just track count for now)
    num_remap_rules: usize,

    /// Parameter overrides (simplified - just track count for now)
    num_param_rules: usize,
}

impl ArgumentsImpl {
    fn new(allocator: rcl_allocator_t) -> Self {
        Self {
            allocator,
            unparsed_ros_args: Vec::new(),
            unparsed_args: Vec::new(),
            parameter_files: Vec::new(),
            enclave: None,
            external_log_config_file: None,
            external_log_file_name_prefix: None,
            log_levels: rcl_log_levels_t::default(),
            log_stdout_disabled: false,
            log_rosout_disabled: false,
            log_ext_lib_disabled: false,
            num_remap_rules: 0,
            num_param_rules: 0,
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_arguments() -> rcl_arguments_t {
    tracing::trace!("rcl_get_zero_initialized_arguments");
    rcl_arguments_t::default()
}

/// Helper to convert C string to Rust string
unsafe fn c_str_to_string(ptr: *const c_char) -> Option<String> {
    if ptr.is_null() {
        return None;
    }
    unsafe { CStr::from_ptr(ptr) }.to_str().ok().map(|s| s.to_string())
}

/// Check if a string is a valid remap rule
fn is_valid_remap_rule(arg: &str) -> bool {
    // Basic validation: must contain :=
    if !arg.contains(":=") {
        return false;
    }

    let parts: Vec<&str> = arg.split(":=").collect();
    if parts.len() != 2 {
        return false;
    }

    let lhs = parts[0];
    let rhs = parts[1];

    // Both sides must be non-empty
    if lhs.is_empty() || rhs.is_empty() {
        return false;
    }

    // Check for invalid characters (spaces, braces, etc.)
    if lhs.contains(' ') || rhs.contains(' ') ||
       lhs.contains('{') || rhs.contains('{') ||
       lhs.contains('}') || rhs.contains('}') {
        return false;
    }

    // LHS cannot start with : or ~
    if lhs.starts_with(':') || lhs.starts_with('~') && lhs.len() == 1 {
        return false;
    }

    // Check if LHS starts with a number (invalid)
    if lhs.chars().next().is_some_and(|c| c.is_ascii_digit()) {
        return false;
    }

    // Check for special cases
    if lhs.starts_with("__node") {
        // Node name remap: __node:=name or node:__node:=name
        return is_valid_node_name(rhs);
    } else if lhs.starts_with("__ns") {
        // Namespace remap: __ns:=/namespace
        return is_valid_namespace(rhs);
    }

    // Check for node-specific remaps: node:name:=value or node:__node:=name or nodename:__ns:=/namespace
    let lhs_parts: Vec<&str> = lhs.split(':').collect();
    if lhs_parts.len() > 1 {
        // Check each part
        for (i, part) in lhs_parts.iter().enumerate() {
            if part.is_empty() {
                // Empty part like "rostopic::name:=value" is invalid
                return false;
            }
            if part.starts_with(':') || (part.starts_with('~') && part.len() == 1) {
                return false;
            }
            // Check if part is only slashes (like "//") which is invalid
            if part.chars().all(|c| c == '/') {
                return false;
            }
            // Special remaps (__node, __ns) can only be in specific positions
            if part.starts_with("__node") {
                // __node can be at start or at position 1 (node:__node:=name)
                if i > 1 {
                    return false;
                }
                // If __node is at position 1, verify it's actually __node remap
                if i == 1 {
                    return is_valid_node_name(rhs);
                }
            }
            if part.starts_with("__ns") {
                // __ns can be at start or at position 1 (nodename:__ns:=/namespace)
                if i > 1 {
                    return false;
                }
                // If __ns is present, verify rhs is valid namespace
                return is_valid_namespace(rhs);
            }
        }
    }

    true
}

/// Check if a string is a valid node name
fn is_valid_node_name(name: &str) -> bool {
    if name.is_empty() {
        return false;
    }

    // Node name cannot start with /
    if name.starts_with('/') {
        return false;
    }

    // Node name must be alphanumeric plus underscore
    name.chars().all(|c| c.is_alphanumeric() || c == '_')
}

/// Check if a string is a valid namespace
fn is_valid_namespace(ns: &str) -> bool {
    if ns.is_empty() {
        return false;
    }

    // Namespace must start with /
    if !ns.starts_with('/') {
        return false;
    }

    // Single / is valid (root namespace)
    if ns == "/" {
        return true;
    }

    // Check each part
    for part in ns[1..].split('/') {
        if part.is_empty() {
            continue; // Allow trailing /
        }
        if !part.chars().all(|c| c.is_alphanumeric() || c == '_') {
            return false;
        }
    }

    true
}

/// Check if a string is a valid param rule
fn is_valid_param_rule(arg: &str) -> bool {
    // Basic validation: must contain :=
    if !arg.contains(":=") {
        return false;
    }

    let parts: Vec<&str> = arg.split(":=").collect();
    if parts.len() != 2 {
        return false;
    }

    let lhs = parts[0];
    let rhs = parts[1];

    // Both sides must be non-empty
    if lhs.is_empty() || rhs.is_empty() {
        return false;
    }

    // Check for invalid characters
    if lhs.contains(' ') || lhs.contains('}') || lhs.contains('{') {
        return false;
    }

    // LHS cannot start with : or ~
    if lhs.starts_with(':') || lhs.starts_with('~') && lhs.len() == 1 {
        return false;
    }

    // Check if LHS starts with a number (invalid)
    if lhs.chars().next().is_some_and(|c| c.is_ascii_digit()) {
        return false;
    }

    // Param names cannot use __node or __ns prefixes
    if lhs.starts_with("__node") || lhs.starts_with("__ns") {
        return false;
    }

    // Check for invalid colons in middle (like :__node:)
    let lhs_parts: Vec<&str> = lhs.split(':').collect();
    for part in &lhs_parts {
        if part.is_empty() || part.starts_with(':') || (part.starts_with('~') && part.len() == 1) {
            return false;
        }
        if part.starts_with("__node") || part.starts_with("__ns") {
            return false;
        }
    }

    true
}

/// Check if a string is a valid log level
fn is_valid_log_level(level: &str) -> bool {
    matches!(
        level.to_uppercase().as_str(),
        "UNSET" | "DEBUG" | "INFO" | "WARN" | "ERROR" | "FATAL"
    )
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_parse_arguments(
    argc: c_int,
    argv: *const *const c_char,
    allocator: rcl_allocator_t,
    args_output: *mut rcl_arguments_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_parse_arguments: argc={}", argc);

    // Validate arguments
    if argc < 0 {
        rcl_set_error_string("argc must be non-negative");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if argc > 0 && argv.is_null() {
        rcl_set_error_string("argv is null with non-zero argc");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if args_output.is_null() {
        rcl_set_error_string("args_output is null");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check if args_output is already initialized
    if !unsafe { (*args_output).impl_.is_null() } {
        rcl_set_error_string("args_output already initialized");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check allocator
    if !is_valid_allocator(&allocator) {
        rcl_set_error_string("allocator is invalid");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Parse arguments
    let mut impl_ = ArgumentsImpl::new(allocator);

    // Convert argv to Rust strings
    let mut args = Vec::new();
    for i in 0..argc {
        if let Some(arg) = unsafe { c_str_to_string(*argv.offset(i as isize)) } {
            args.push(arg);
        }
    }

    // Parse ROS arguments
    let mut in_ros_args = false;
    let mut i = 0;

    while i < args.len() {
        let arg = &args[i];

        if arg == RCL_ROS_ARGS_FLAG {
            in_ros_args = true;
            i += 1;
            continue;
        }

        if arg == RCL_ROS_ARGS_EXPLICIT_END_TOKEN {
            if in_ros_args {
                in_ros_args = false;
                i += 1;
                continue;
            } else {
                // Trailing -- after ROS args ended
                impl_.unparsed_args.push(i as c_int);
                i += 1;
                continue;
            }
        }

        if !in_ros_args {
            impl_.unparsed_args.push(i as c_int);
            i += 1;
            continue;
        }

        // We're in ROS args section
        if arg == RCL_SHORT_REMAP_FLAG || arg == RCL_REMAP_FLAG {
            if i + 1 < args.len() {
                let remap_arg = &args[i + 1];
                if is_valid_remap_rule(remap_arg) {
                    impl_.num_remap_rules += 1;
                    i += 2;
                    continue;
                } else {
                    // Invalid remap rule
                    rcl_set_error_string(&format!("Invalid remap rule: {}", remap_arg));
                    return RCL_RET_INVALID_ROS_ARGS as _;
                }
            } else {
                // Missing remap argument
                rcl_set_error_string("Missing argument for remap flag");
                return RCL_RET_INVALID_ROS_ARGS as _;
            }
        } else if arg == RCL_SHORT_PARAM_FLAG || arg == RCL_PARAM_FLAG {
            if i + 1 < args.len() {
                let param_arg = &args[i + 1];
                if is_valid_param_rule(param_arg) {
                    impl_.num_param_rules += 1;
                    i += 2;
                    continue;
                } else {
                    // Invalid param rule
                    rcl_set_error_string(&format!("Invalid param rule: {}", param_arg));
                    return RCL_RET_INVALID_ROS_ARGS as _;
                }
            } else {
                // Missing param argument
                rcl_set_error_string("Missing argument for param flag");
                return RCL_RET_INVALID_ROS_ARGS as _;
            }
        } else if arg == RCL_PARAM_FILE_FLAG {
            if i + 1 < args.len() {
                let file_path = &args[i + 1];
                if let Ok(cstring) = CString::new(file_path.as_str()) {
                    impl_.parameter_files.push(cstring);
                }
                i += 2;
                continue;
            } else {
                // Missing param file argument
                rcl_set_error_string("Missing argument for params-file flag");
                return RCL_RET_INVALID_ROS_ARGS as _;
            }
        } else if arg == RCL_LOG_LEVEL_FLAG {
            if i + 1 < args.len() {
                let level = &args[i + 1];
                if is_valid_log_level(level) {
                    // Store log level (simplified)
                    i += 2;
                    continue;
                } else {
                    // Invalid log level
                    rcl_set_error_string(&format!("Invalid log level: {}", level));
                    return RCL_RET_INVALID_ROS_ARGS as _;
                }
            } else {
                // Missing log level argument
                rcl_set_error_string("Missing argument for log-level flag");
                return RCL_RET_INVALID_ROS_ARGS as _;
            }
        } else if arg == RCL_LOG_CONFIG_FILE_FLAG {
            if i + 1 < args.len() {
                let file = &args[i + 1];
                if let Ok(cstring) = CString::new(file.as_str()) {
                    impl_.external_log_config_file = Some(cstring);
                }
                i += 2;
                continue;
            } else {
                // Missing config file argument
                rcl_set_error_string("Missing argument for log-config-file flag");
                return RCL_RET_INVALID_ROS_ARGS as _;
            }
        } else if arg == RCL_LOG_FILE_NAME_FLAG {
            if i + 1 < args.len() {
                let file = &args[i + 1];
                if let Ok(cstring) = CString::new(file.as_str()) {
                    impl_.external_log_file_name_prefix = Some(cstring);
                }
                i += 2;
                continue;
            } else {
                // Missing file name argument
                rcl_set_error_string("Missing argument for log-file-name flag");
                return RCL_RET_INVALID_ROS_ARGS as _;
            }
        } else if arg == RCL_ENCLAVE_FLAG || arg == RCL_SHORT_ENCLAVE_FLAG {
            if i + 1 < args.len() {
                let enclave = &args[i + 1];
                if let Ok(cstring) = CString::new(enclave.as_str()) {
                    impl_.enclave = Some(cstring);
                }
                i += 2;
                continue;
            } else {
                // Missing enclave argument
                rcl_set_error_string("Missing argument for enclave flag");
                return RCL_RET_INVALID_ROS_ARGS as _;
            }
        } else if arg == RCL_ENABLE_STDOUT_LOGS_FLAG {
            impl_.log_stdout_disabled = false;
            i += 1;
            continue;
        } else if arg == RCL_DISABLE_STDOUT_LOGS_FLAG {
            impl_.log_stdout_disabled = true;
            i += 1;
            continue;
        } else if arg == RCL_ENABLE_ROSOUT_LOGS_FLAG {
            impl_.log_rosout_disabled = false;
            i += 1;
            continue;
        } else if arg == RCL_DISABLE_ROSOUT_LOGS_FLAG {
            impl_.log_rosout_disabled = true;
            i += 1;
            continue;
        } else if arg == RCL_ENABLE_EXTERNAL_LIB_LOGS_FLAG {
            impl_.log_ext_lib_disabled = false;
            i += 1;
            continue;
        } else if arg == RCL_DISABLE_EXTERNAL_LIB_LOGS_FLAG {
            impl_.log_ext_lib_disabled = true;
            i += 1;
            continue;
        } else {
            // Unknown ROS argument
            impl_.unparsed_ros_args.push(i as c_int);
            i += 1;
        }
    }

    // Allocate and store the implementation
    let impl_box = Box::new(impl_);
    unsafe { (*args_output).impl_ = Box::into_raw(impl_box) as *mut rcl_arguments_impl_s; }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_count_unparsed(
    args: *const rcl_arguments_t,
) -> c_int {
    if args.is_null() || unsafe { (*args).impl_.is_null() } {
        rcl_set_error_string("arguments not initialized");
        return -1;
    }

    let impl_ = unsafe { &*((*args).impl_ as *const ArgumentsImpl) };
    impl_.unparsed_args.len() as c_int
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_unparsed(
    args: *const rcl_arguments_t,
    allocator: rcl_allocator_t,
    output_unparsed_indices: *mut *mut c_int,
) -> rcl_ret_t {
    if args.is_null() || unsafe { (*args).impl_.is_null() } {
        rcl_set_error_string("arguments not initialized");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if !is_valid_allocator(&allocator) {
        rcl_set_error_string("allocator is invalid");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if output_unparsed_indices.is_null() {
        rcl_set_error_string("output pointer is null");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let impl_ = unsafe { &*((*args).impl_ as *const ArgumentsImpl) };

    if impl_.unparsed_args.is_empty() {
        unsafe { *output_unparsed_indices = ptr::null_mut(); }
        return RCL_RET_OK as _;
    }

    // Allocate memory for indices
    let size = impl_.unparsed_args.len() * std::mem::size_of::<c_int>();
    let ptr = unsafe { (allocator.allocate.unwrap())(size, allocator.state) };

    if ptr.is_null() {
        rcl_set_error_string("failed to allocate memory");
        return RCL_RET_BAD_ALLOC as _;
    }

    let indices_ptr = ptr as *mut c_int;
    for (i, &idx) in impl_.unparsed_args.iter().enumerate() {
        unsafe { *indices_ptr.add(i) = idx; }
    }

    unsafe { *output_unparsed_indices = indices_ptr; }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_count_unparsed_ros(
    args: *const rcl_arguments_t,
) -> c_int {
    if args.is_null() || (*args).impl_.is_null() {
        rcl_set_error_string("arguments not initialized");
        return -1;
    }

    let impl_ = &*((*args).impl_ as *const ArgumentsImpl);
    impl_.unparsed_ros_args.len() as c_int
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_unparsed_ros(
    args: *const rcl_arguments_t,
    allocator: rcl_allocator_t,
    output_unparsed_ros_indices: *mut *mut c_int,
) -> rcl_ret_t {
    if args.is_null() {
        rcl_set_error_string("arguments is null");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if (*args).impl_.is_null() {
        rcl_set_error_string("arguments not initialized");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if !is_valid_allocator(&allocator) {
        rcl_set_error_string("allocator is invalid");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if output_unparsed_ros_indices.is_null() {
        rcl_set_error_string("output pointer is null");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let impl_ = &*((*args).impl_ as *const ArgumentsImpl);

    if impl_.unparsed_ros_args.is_empty() {
        *output_unparsed_ros_indices = ptr::null_mut();
        return RCL_RET_OK as _;
    }

    // Allocate memory for indices
    let size = impl_.unparsed_ros_args.len() * std::mem::size_of::<c_int>();
    let ptr = (allocator.allocate.unwrap())(size, allocator.state);

    if ptr.is_null() {
        rcl_set_error_string("failed to allocate memory");
        return RCL_RET_BAD_ALLOC as _;
    }

    let indices_ptr = ptr as *mut c_int;
    for (i, &idx) in impl_.unparsed_ros_args.iter().enumerate() {
        *indices_ptr.add(i) = idx;
    }

    *output_unparsed_ros_indices = indices_ptr;
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_param_files_count(
    args: *const rcl_arguments_t,
) -> c_int {
    if args.is_null() || unsafe { (*args).impl_.is_null() } {
        return -1;
    }

    let impl_ = unsafe { &*((*args).impl_ as *const ArgumentsImpl) };
    impl_.parameter_files.len() as c_int
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_param_files(
    arguments: *const rcl_arguments_t,
    allocator: rcl_allocator_t,
    parameter_files: *mut *mut *mut c_char,
) -> rcl_ret_t {
    if arguments.is_null() || (*arguments).impl_.is_null() {
        rcl_set_error_string("arguments not initialized");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if !is_valid_allocator(&allocator) {
        rcl_set_error_string("allocator is invalid");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if parameter_files.is_null() {
        rcl_set_error_string("output pointer is null");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let impl_ = &*((*arguments).impl_ as *const ArgumentsImpl);

    if impl_.parameter_files.is_empty() {
        *parameter_files = ptr::null_mut();
        return RCL_RET_OK as _;
    }

    // Allocate array of string pointers
    let count = impl_.parameter_files.len();
    let size = count * std::mem::size_of::<*mut c_char>();
    let array_ptr = (allocator.allocate.unwrap())(size, allocator.state);

    if array_ptr.is_null() {
        rcl_set_error_string("failed to allocate memory");
        return RCL_RET_BAD_ALLOC as _;
    }

    let strings_array = array_ptr as *mut *mut c_char;

    // Allocate and copy each string
    for (i, cstring) in impl_.parameter_files.iter().enumerate() {
        let str_bytes = cstring.as_bytes_with_nul();
        let str_size = str_bytes.len();
        let str_ptr = (allocator.allocate.unwrap())(str_size, allocator.state);

        if str_ptr.is_null() {
            // Cleanup allocated strings
            for j in 0..i {
                let prev_str = *strings_array.add(j);
                if !prev_str.is_null() {
                    (allocator.deallocate.unwrap())(prev_str as *mut _, allocator.state);
                }
            }
            (allocator.deallocate.unwrap())(array_ptr, allocator.state);
            rcl_set_error_string("failed to allocate string memory");
            return RCL_RET_BAD_ALLOC as _;
        }

        ptr::copy_nonoverlapping(str_bytes.as_ptr(), str_ptr as *mut u8, str_size);
        *strings_array.add(i) = str_ptr as *mut c_char;
    }

    *parameter_files = strings_array;
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_param_overrides(
    arguments: *const rcl_arguments_t,
    parameter_overrides: *mut *mut rcl_params_t,
) -> rcl_ret_t {
    if arguments.is_null() || (*arguments).impl_.is_null() {
        rcl_set_error_string("arguments not initialized");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if parameter_overrides.is_null() {
        rcl_set_error_string("output pointer is null");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if !(*parameter_overrides).is_null() {
        rcl_set_error_string("output pointer is not null");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // For now, just return NULL (no parameter overrides parsed)
    *parameter_overrides = ptr::null_mut();
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_remove_ros_arguments(
    argv: *const *const c_char,
    args: *const rcl_arguments_t,
    allocator: rcl_allocator_t,
    nonros_argc: *mut c_int,
    nonros_argv: *mut *mut *const c_char,
) -> rcl_ret_t {
    if args.is_null() || (*args).impl_.is_null() {
        rcl_set_error_string("arguments not initialized");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if !is_valid_allocator(&allocator) {
        rcl_set_error_string("allocator is invalid");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if nonros_argc.is_null() || nonros_argv.is_null() {
        rcl_set_error_string("output pointers are null");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check that output is zero-initialized
    if *nonros_argc != 0 || !(*nonros_argv).is_null() {
        rcl_set_error_string("output must be zero-initialized");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let impl_ = unsafe { &*((*args).impl_ as *const ArgumentsImpl) };

    // If argv is null, check if there are unparsed args
    if argv.is_null() {
        if !impl_.unparsed_args.is_empty() {
            // Have unparsed args but no argv to get them from - error
            rcl_set_error_string("argv is null but unparsed args exist");
            return RCL_RET_INVALID_ARGUMENT as _;
        } else {
            // No unparsed args and no argv - OK
            *nonros_argc = 0;
            *nonros_argv = ptr::null_mut();
            return RCL_RET_OK as _;
        }
    }

    let count = impl_.unparsed_args.len();
    if count == 0 {
        *nonros_argc = 0;
        *nonros_argv = ptr::null_mut();
        return RCL_RET_OK as _;
    }

    // Allocate array of pointers
    let size = count * std::mem::size_of::<*const c_char>();
    let array_ptr = (allocator.allocate.unwrap())(size, allocator.state);

    if array_ptr.is_null() {
        rcl_set_error_string("failed to allocate memory");
        return RCL_RET_BAD_ALLOC as _;
    }

    let argv_array = array_ptr as *mut *const c_char;

    // Copy pointers from original argv at unparsed indices
    for (i, &idx) in impl_.unparsed_args.iter().enumerate() {
        *argv_array.add(i) = *argv.offset(idx as isize);
    }

    *nonros_argc = count as c_int;
    *nonros_argv = argv_array;

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_log_levels(
    arguments: *const rcl_arguments_t,
    log_levels: *mut rcl_log_levels_t,
) -> rcl_ret_t {
    if arguments.is_null() || (*arguments).impl_.is_null() {
        rcl_set_error_string("arguments not initialized");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if log_levels.is_null() {
        rcl_set_error_string("log_levels is null");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let impl_ = &*((*arguments).impl_ as *const ArgumentsImpl);
    *log_levels = impl_.log_levels;

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_copy(
    args: *const rcl_arguments_t,
    args_out: *mut rcl_arguments_t,
) -> rcl_ret_t {
    if args.is_null() || (*args).impl_.is_null() {
        rcl_set_error_string("source arguments not initialized");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if args_out.is_null() {
        rcl_set_error_string("destination arguments is null");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if !(*args_out).impl_.is_null() {
        rcl_set_error_string("destination arguments already initialized");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let src_impl = &*((*args).impl_ as *const ArgumentsImpl);

    // Create a deep copy
    let mut dst_impl = ArgumentsImpl::new(src_impl.allocator);
    dst_impl.unparsed_ros_args = src_impl.unparsed_ros_args.clone();
    dst_impl.unparsed_args = src_impl.unparsed_args.clone();
    dst_impl.parameter_files = src_impl.parameter_files.clone();
    dst_impl.enclave = src_impl.enclave.clone();
    dst_impl.external_log_config_file = src_impl.external_log_config_file.clone();
    dst_impl.external_log_file_name_prefix = src_impl.external_log_file_name_prefix.clone();
    dst_impl.log_levels = src_impl.log_levels;
    dst_impl.log_stdout_disabled = src_impl.log_stdout_disabled;
    dst_impl.log_rosout_disabled = src_impl.log_rosout_disabled;
    dst_impl.log_ext_lib_disabled = src_impl.log_ext_lib_disabled;
    dst_impl.num_remap_rules = src_impl.num_remap_rules;
    dst_impl.num_param_rules = src_impl.num_param_rules;

    let impl_box = Box::new(dst_impl);
    (*args_out).impl_ = Box::into_raw(impl_box) as *mut rcl_arguments_impl_s;

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_fini(args: *mut rcl_arguments_t) -> rcl_ret_t {
    if args.is_null() {
        rcl_set_error_string("arguments is null");
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if (*args).impl_.is_null() {
        rcl_set_error_string("arguments->impl is null");
        return RCL_RET_ERROR as _;
    }

    // Free the implementation
    let impl_ptr = (*args).impl_ as *mut ArgumentsImpl;
    let _ = Box::from_raw(impl_ptr);

    (*args).impl_ = ptr::null_mut();

    RCL_RET_OK as _
}

fn is_valid_allocator(allocator: &rcl_allocator_t) -> bool {
    allocator.allocate.is_some() && allocator.deallocate.is_some()
}
