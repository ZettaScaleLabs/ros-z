// Test message type support for unit tests
// This module provides access to test_msgs ROS message types

use std::os::raw::c_char;

// Re-export for convenience
pub use rcl_z::ros::rosidl_runtime_c__String;
use rcl_z::ros::{rosidl_message_type_support_t, rosidl_service_type_support_t};

// C-compatible message structures for testing
#[repr(C)]
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct test_msgs__msg__BasicTypes {
    pub int64_value: i64,
}

#[repr(C)]
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct test_msgs__msg__Strings {
    pub string_value: rosidl_runtime_c__String,
}

// Service Request/Response structures for BasicTypes service
#[repr(C)]
#[derive(Debug, Clone)]
#[allow(non_camel_case_types, dead_code)]
pub struct test_msgs__srv__BasicTypes_Request {
    pub bool_value: bool,
    pub byte_value: u8,
    pub char_value: u8,
    pub float32_value: f32,
    pub float64_value: f64,
    pub int8_value: i8,
    pub uint8_value: u8,
    pub int16_value: i16,
    pub uint16_value: u16,
    pub int32_value: i32,
    pub uint32_value: u32,
    pub int64_value: i64,
    pub uint64_value: u64,
    pub string_value: rosidl_runtime_c__String,
}

impl Default for test_msgs__srv__BasicTypes_Request {
    fn default() -> Self {
        unsafe {
            let mut s: Self = std::mem::zeroed();
            test_msgs__srv__BasicTypes_Request__init(&mut s);
            s
        }
    }
}

impl Drop for test_msgs__srv__BasicTypes_Request {
    fn drop(&mut self) {
        unsafe {
            test_msgs__srv__BasicTypes_Request__fini(self);
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone)]
#[allow(non_camel_case_types, dead_code)]
pub struct test_msgs__srv__BasicTypes_Response {
    pub bool_value: bool,
    pub byte_value: u8,
    pub char_value: u8,
    pub float32_value: f32,
    pub float64_value: f64,
    pub int8_value: i8,
    pub uint8_value: u8,
    pub int16_value: i16,
    pub uint16_value: u16,
    pub int32_value: i32,
    pub uint32_value: u32,
    pub int64_value: i64,
    pub uint64_value: u64,
    pub string_value: rosidl_runtime_c__String,
}

impl Default for test_msgs__srv__BasicTypes_Response {
    fn default() -> Self {
        unsafe {
            let mut s: Self = std::mem::zeroed();
            test_msgs__srv__BasicTypes_Response__init(&mut s);
            s
        }
    }
}

impl Drop for test_msgs__srv__BasicTypes_Response {
    fn drop(&mut self) {
        unsafe {
            test_msgs__srv__BasicTypes_Response__fini(self);
        }
    }
}

unsafe extern "C" {
    #[allow(improper_ctypes)]
    #[allow(dead_code)]
    pub fn rosidl_typesupport_fastrtps_c__get_message_type_support_handle__test_msgs__msg__BasicTypes()
    -> *const rosidl_message_type_support_t;

    #[allow(improper_ctypes)]
    #[allow(dead_code)]
    pub fn rosidl_typesupport_fastrtps_c__get_message_type_support_handle__test_msgs__msg__Strings()
    -> *const rosidl_message_type_support_t;

    #[allow(improper_ctypes)]
    #[allow(dead_code)]
    pub fn rosidl_typesupport_fastrtps_c__get_service_type_support_handle__test_msgs__srv__BasicTypes()
    -> *const rosidl_service_type_support_t;

    // Message init/fini functions
    #[allow(dead_code)]
    pub fn test_msgs__srv__BasicTypes_Request__init(msg: *mut test_msgs__srv__BasicTypes_Request) -> bool;
    #[allow(dead_code)]
    pub fn test_msgs__srv__BasicTypes_Request__fini(msg: *mut test_msgs__srv__BasicTypes_Request);
    #[allow(dead_code)]
    pub fn test_msgs__srv__BasicTypes_Response__init(msg: *mut test_msgs__srv__BasicTypes_Response) -> bool;
    #[allow(dead_code)]
    pub fn test_msgs__srv__BasicTypes_Response__fini(msg: *mut test_msgs__srv__BasicTypes_Response);

    // String manipulation functions
    #[allow(dead_code)]
    pub fn rosidl_runtime_c__String__init(str: *mut rcl_z::ros::rosidl_runtime_c__String);
    #[allow(dead_code)]
    pub fn rosidl_runtime_c__String__fini(str: *mut rcl_z::ros::rosidl_runtime_c__String);
    #[allow(dead_code)]
    pub fn rosidl_runtime_c__String__assign(
        str: *mut rcl_z::ros::rosidl_runtime_c__String,
        value: *const c_char,
    ) -> bool;

}

// Macro to get type support similar to C++ ROSIDL_GET_MSG_TYPE_SUPPORT
#[macro_export]
macro_rules! ROSIDL_GET_MSG_TYPE_SUPPORT {
    (test_msgs, msg, BasicTypes) => {
        $crate::test_msgs_support::rosidl_typesupport_fastrtps_c__get_message_type_support_handle__test_msgs__msg__BasicTypes()
    };
    (test_msgs, msg, Strings) => {
        $crate::test_msgs_support::rosidl_typesupport_fastrtps_c__get_message_type_support_handle__test_msgs__msg__Strings()
    };
}

// Macro to get service type support similar to C++ ROSIDL_GET_SRV_TYPE_SUPPORT
#[macro_export]
macro_rules! ROSIDL_GET_SRV_TYPE_SUPPORT {
    (test_msgs, srv, BasicTypes) => {
        $crate::test_msgs_support::rosidl_typesupport_fastrtps_c__get_service_type_support_handle__test_msgs__srv__BasicTypes()
    };
}
