use crate::ros::*;
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
    tracing::error!("rcl_init_options_get_allocator");
    unsafe {
        let opts_impl = (*init_options).impl_ as *mut RclInitOptionsImpl;
        tracing::error!("{:?}", (*opts_impl).allocator);
        &(*opts_impl).allocator as _
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_logging_configure_with_output_handler(
    _global_args: *const rcl_arguments_t,
    _allocator: *const rcl_allocator_t,
    _output_handler: rcl_logging_output_handler_t,
) -> rcl_ret_t {
    tracing::error!("rcl_logging_configure_with_output_handler");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_fini(init_options: *mut rcl_init_options_t) -> rcl_ret_t {
    // tracing::error!("rcl_init_options_fini");
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
pub extern "C" fn rcl_node_get_logger_name(
    _node: *const rcl_node_t,
) -> *const ::std::os::raw::c_char {
    Box::into_raw(Box::new(CString::new("no rcl-z logger").unwrap())) as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_parse_arguments(
    argc: ::std::os::raw::c_int,
    argv: *const *const ::std::os::raw::c_char,
    allocator: rcl_allocator_t,
    args_output: *mut rcl_arguments_t,
) -> rcl_ret_t {
    tracing::error!("rcl_parse_arguments");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_arguments_get_count_unparsed_ros(
    args: *const rcl_arguments_t,
) -> ::std::os::raw::c_int {
    tracing::error!("rcl_arguments_get_count_unparsed_ros");
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_fully_qualified_name(
    node: *const rcl_node_t,
) -> *const ::std::os::raw::c_char {
    Box::into_raw(Box::new(CString::new("no rcl-z node name").unwrap())) as _
}

// #[unsafe(no_mangle)]
// pub extern "C"  fn rcl_node_get_default_options() -> rcl_node_options_t {
//     let mut args = rcl_arguments_s::default();
//     args.impl_ = Box::into_raw(Box::new(())) as _;
//     let mut opts = rcl_node_options_t::default();
//     opts.arguments = args;
//     opts
// }

#[unsafe(no_mangle)]
pub extern "C" fn rcl_arguments_get_param_overrides(
    arguments: *const rcl_arguments_t,
    parameter_overrides: *mut *mut rcl_params_t,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_namespace(node: *const rcl_node_t) -> *const ::std::os::raw::c_char {
    Box::into_raw(Box::new(CString::new("no rcl-z node namespace").unwrap())) as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_name(node: *const rcl_node_t) -> *const ::std::os::raw::c_char {
    Box::into_raw(Box::new(CString::new("no rcl-z node namespace").unwrap())) as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_expand_topic_name(
    input_topic_name: *const ::std::os::raw::c_char,
    node_name: *const ::std::os::raw::c_char,
    node_namespace: *const ::std::os::raw::c_char,
    substitutions: *const rcutils_string_map_t,
    allocator: rcl_allocator_t,
    output_topic_name: *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_remap_topic_name(
    local_arguments: *const rcl_arguments_t,
    global_arguments: *const rcl_arguments_t,
    topic_name: *const ::std::os::raw::c_char,
    node_name: *const ::std::os::raw::c_char,
    node_namespace: *const ::std::os::raw::c_char,
    allocator: rcl_allocator_t,
    output_name: *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_remap_service_name(
    local_arguments: *const rcl_arguments_t,
    global_arguments: *const rcl_arguments_t,
    service_name: *const ::std::os::raw::c_char,
    node_name: *const ::std::os::raw::c_char,
    node_namespace: *const ::std::os::raw::c_char,
    allocator: rcl_allocator_t,
    output_name: *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    RCL_RET_OK as _
}
#[unsafe(no_mangle)]
pub extern "C" fn rcl_remap_node_name(
    local_arguments: *const rcl_arguments_t,
    global_arguments: *const rcl_arguments_t,
    node_name: *const ::std::os::raw::c_char,
    allocator: rcl_allocator_t,
    output_name: *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    RCL_RET_OK as _
}
#[unsafe(no_mangle)]
pub extern "C" fn rcl_remap_node_namespace(
    local_arguments: *const rcl_arguments_t,
    global_arguments: *const rcl_arguments_t,
    node_name: *const ::std::os::raw::c_char,
    allocator: rcl_allocator_t,
    output_namespace: *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    RCL_RET_OK as _
}
#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_resolve_name(
    node: *const rcl_node_t,
    input_name: *const ::std::os::raw::c_char,
    allocator: rcl_allocator_t,
    is_service: bool,
    only_expand: bool,
    output_name: *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    let cstr = CString::new("skip_rcl_node_resolve_name").unwrap();
    unsafe {
        *output_name = cstr.into_raw();
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_is_valid(node: *const rcl_node_t) -> bool {
    true
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
pub extern "C" fn rcl_publisher_get_rmw_handle(publisher: *const rcl_publisher_t) -> *mut rmw_publisher_t {
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
    unsafe {
        !(*context).impl_.is_null()
    }
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

    unsafe {
        if !(*context).impl_.is_null() {
            tracing::error!("(*context).impl_ has already initiated.");
            return RCL_RET_ALREADY_INIT as _;
        }
    }

    let context_impl = ContextImpl(ZContext::new().unwrap());

    unsafe {
        (*context).impl_ = Box::into_raw(Box::new(context_impl)) as _;
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_shutdown(context: *mut rcl_context_t) -> rcl_ret_t {
    tracing::trace!("rcl_shutdown");

    unsafe {
        if context.is_null() || (*context).impl_.is_null() {
            return RCL_RET_INVALID_ARGUMENT as _;
        }

        // Recreate the Box to take ownership back
        let boxed: Box<ContextImpl> = Box::from_raw((*context).impl_ as *mut ContextImpl);
        (*boxed).0.shutdown().unwrap();

        // Box drops here automatically
        (*context).impl_ = std::ptr::null_mut();
    }

    RCL_RET_OK as _
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
