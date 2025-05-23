#![allow(unused)]

use crate::ros::*;
use crate::traits::OwnImpl;
use crate::{impl_has_impl_ptr, rclz_try};
use ros_z::context::ZContext;
use std::{
    ffi::{CString, c_char, c_int},
    ops::Deref,
};

pub struct ContextImpl(ZContext);

impl Deref for ContextImpl {
    type Target = ZContext;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl_has_impl_ptr!(rcl_context_t, rcl_context_impl_t, ContextImpl);

struct RclInitOptionsImpl {
    allocator: rcl_allocator_t,
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_init(
    init_options: *mut rcl_init_options_t,
    allocator: rcl_allocator_t,
) -> rcl_ret_t {
    unsafe {
        let mut opts = rcl_init_options_t::default();
        opts.impl_ = Box::into_raw(Box::new(RclInitOptionsImpl { allocator })) as _;
        std::ptr::write(init_options, opts);
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_get_allocator(
    init_options: *const rcl_init_options_t,
) -> *const rcl_allocator_t {
    tracing::trace!("rcl_init_options_get_allocator");
    unsafe {
        let opts_impl = (*init_options).impl_ as *mut RclInitOptionsImpl;
        tracing::trace!("{:?}", (*opts_impl).allocator);
        &(*opts_impl).allocator as _
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_logging_configure_with_output_handler(
    _global_args: *const rcl_arguments_t,
    _allocator: *const rcl_allocator_t,
    _output_handler: rcl_logging_output_handler_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_logging_configure_with_output_handler");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_fini(init_options: *mut rcl_init_options_t) -> rcl_ret_t {
    tracing::trace!("rcl_init_options_fini");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_copy(
    src: *const rcl_init_options_t,
    dst: *mut rcl_init_options_t,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_guard_condition_init(
    guard_condition: *mut rcl_guard_condition_t,
    context: *mut rcl_context_t,
    options: rcl_guard_condition_options_t,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_parse_arguments(
    argc: ::std::os::raw::c_int,
    argv: *const *const ::std::os::raw::c_char,
    allocator: rcl_allocator_t,
    args_output: *mut rcl_arguments_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_parse_arguments");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_arguments_get_count_unparsed_ros(
    args: *const rcl_arguments_t,
) -> ::std::os::raw::c_int {
    tracing::trace!("rcl_arguments_get_count_unparsed_ros");
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_arguments_get_param_overrides(
    arguments: *const rcl_arguments_t,
    parameter_overrides: *mut *mut rcl_params_t,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_init(
    service: *mut rcl_service_t,
    node: *const rcl_node_t,
    type_support: *const rosidl_service_type_support_t,
    service_name: *const ::std::os::raw::c_char,
    options: *const rcl_service_options_t,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_guard_condition_get_options(
    guard_condition: *const rcl_guard_condition_t,
) -> *const rcl_guard_condition_options_t {
    std::ptr::null()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_trigger_guard_condition(
    guard_condition: *mut rcl_guard_condition_t,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_get_rmw_handle(
    publisher: *const rcl_publisher_t,
) -> *mut rmw_publisher_t {
    Box::into_raw(Box::new(())) as _
    // // TODO: Recycle this
    // let mut rmw_pub = rmw_publisher_t::default();
    // let gid = rmw_gid_t
    // rmw_pub.data =
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_gid_for_publisher(
    publisher: *const rmw_publisher_t,
    gid: *mut rmw_gid_t,
) -> rmw_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_event_init(
    event: *mut rcl_event_t,
    publisher: *const rcl_publisher_t,
    event_type: rcl_publisher_event_type_t,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_type_description_service_init(
    service: *mut rcl_service_t,
    node: *const rcl_node_t,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_is_valid(service: *const rcl_service_t) -> bool {
    true
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_event_init(
    event: *mut rcl_event_t,
    subscription: *const rcl_subscription_t,
    event_type: rcl_subscription_event_type_t,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_context_is_valid(context: *const rcl_context_t) -> bool {
    unsafe { !(*context).impl_.is_null() }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init(
    _argc: c_int,
    _argv: *const *const c_char,
    _options: *const rcl_init_options_t,
    context: *mut rcl_context_t,
) -> rcl_ret_t {
    zenoh::init_log_from_env_or("error");
    tracing::trace!("rcl_init");
    rclz_try! {
        context.assign_impl(ContextImpl(ZContext::new()?))?;
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_shutdown(context: *mut rcl_context_t) -> rcl_ret_t {
    tracing::trace!("rcl_shutdown");
    rclz_try! {
        context.own_impl()?.shutdown()?;
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_context_fini(_context: *mut rcl_context_t) -> rcl_ret_t {
    tracing::trace!("rcl_context_fini");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_context() -> rcl_context_t {
    rcl_context_t::default()
}
