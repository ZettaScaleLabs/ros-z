#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals, unused)]

use std::ffi::c_void;

// Import generated bindings which include:
// - RMW constants (RMW_RET_*, RMW_QOS_POLICY_*, etc.)
// - RMW types (rmw_time_t, rmw_qos_profile_t, rmw_context_t, etc.)
// - rosidl_message_type_support_t
// - rosidl_service_type_support_t
// - rmw_event_type_t
// - rmw_gid_t
// - rmw_topic_endpoint_info_array_t
// - rmw_feature_t
// - rmw_init_options_t
// - and many other RMW/RCL types
include!(concat!(env!("CARGO_MANIFEST_DIR"), "/src/bindings.rs"));

// Note: RMW_RET_ALREADY_INIT does not exist in the RMW spec.
// Use RMW_RET_INVALID_ARGUMENT when context is already initialized.

// QoS policy values (used for compatibility)
pub const RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL: u32 = 1;
pub const RMW_QOS_POLICY_DURABILITY_VOLATILE: u32 = 2;

// Additional RCL return codes not in RMW bindings
pub const RCL_RET_OK: i32 = 0;
pub const RCL_RET_ERROR: i32 = 1;
pub const RCL_RET_BAD_ALLOC: i32 = 10;
pub const RCL_RET_INVALID_ARGUMENT: i32 = 11;
pub const RCL_RET_UNSUPPORTED: i32 = 3;
pub const RCL_RET_NODE_INVALID: i32 = 100;
pub const RCL_RET_NODE_INVALID_NAME: i32 = 101;
pub const RCL_RET_NODE_INVALID_NAMESPACE: i32 = 102;
pub const RCL_RET_NODE_NAME_NON_EXISTENT: i32 = 103;
pub const RCL_RET_PUBLISHER_INVALID: i32 = 200;
pub const RCL_RET_SUBSCRIPTION_INVALID: i32 = 201;
pub const RCL_RET_SUBSCRIPTION_TAKE_FAILED: i32 = 202;
pub const RCL_RET_CLIENT_INVALID: i32 = 300;
pub const RCL_RET_CLIENT_TAKE_FAILED: i32 = 301;
pub const RCL_RET_SERVICE_INVALID: i32 = 400;
pub const RCL_RET_SERVICE_TAKE_FAILED: i32 = 401;
pub const RCL_RET_ALREADY_INIT: i32 = 500;
pub const RCL_RET_NOT_INIT: i32 = 501;

// Type aliases for compatibility
pub type rcl_ret_t = i32;
pub type rcl_allocator_t = rcutils_allocator_t;
pub type rcl_serialized_message_t = rmw_serialized_message_t;

// Note: rmw_node_options_t is in bindings, but client/service options are not
pub type rcl_node_options_t = *const ::std::os::raw::c_void; // Placeholder

// Client and service options (not in bindings)
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rmw_client_options_t {
    pub qos: rmw_qos_profile_t,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rmw_service_options_t {
    pub qos: rmw_qos_profile_t,
}

// QoS compatibility types
#[repr(u32)]
#[derive(Debug, Copy, Clone)]
pub enum rmw_qos_compatibility_type_t {
    RMW_QOS_COMPATIBILITY_OK = 0,
    RMW_QOS_COMPATIBILITY_WARNING = 1,
    RMW_QOS_COMPATIBILITY_ERROR = 2,
}

// Service type support
#[repr(C)]
#[derive(Debug)]
pub struct rosidl_service_type_support_t {
    pub _placeholder: [u8; 0],
}

// Message bounds (for pre-allocation)
#[repr(C)]
#[derive(Debug)]
pub struct rosidl_message_bounds_t {
    pub _placeholder: [u8; 0],
}

// Serialization support
#[repr(C)]
#[derive(Debug)]
pub struct rmw_serialization_support_t {
    pub _placeholder: [u8; 0],
}

// Opaque impl types (placeholders for rmw-z implementations)
#[repr(C)]
pub struct rmw_node_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct rmw_publisher_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct rmw_subscription_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct rmw_client_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct rmw_service_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct rmw_wait_set_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct rmw_guard_condition_impl_t {
    _private: [u8; 0],
}

// Callback types
pub type rmw_subscription_new_message_callback_t =
    Option<unsafe extern "C" fn(user_data: *const c_void, number_of_messages: usize)>;

pub type rmw_service_new_request_callback_t =
    Option<unsafe extern "C" fn(user_data: *const c_void, number_of_requests: usize)>;

pub type rmw_client_new_response_callback_t =
    Option<unsafe extern "C" fn(user_data: *const c_void, number_of_responses: usize)>;

pub type rmw_event_callback_t =
    Option<unsafe extern "C" fn(user_data: *const c_void, number_of_events: usize)>;

// Sequence types for batch operations
#[repr(C)]
#[derive(Debug)]
pub struct rmw_message_sequence_t {
    pub data: *mut *mut c_void,
    pub size: usize,
    pub capacity: usize,
    pub allocator: rcl_allocator_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_message_info_sequence_t {
    pub data: *mut rmw_message_info_t,
    pub size: usize,
    pub capacity: usize,
    pub allocator: rcl_allocator_t,
}

// rcldynamic_message_t placeholder (not fully supported)
#[repr(C)]
#[derive(Debug)]
pub struct rcldynamic_message_t {
    _private: [u8; 0],
}
