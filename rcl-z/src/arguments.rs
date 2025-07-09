#![allow(unused)]
use crate::ros::*;
use crate::utils::parse_args;

struct ArgumentsImpl {

}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_arguments() -> rcl_arguments_t {
    tracing::trace!("rcl_get_zero_initialized_arguments");
    rcl_arguments_t::default()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_parse_arguments(
    argc: ::std::os::raw::c_int,
    argv: *const *const ::std::os::raw::c_char,
    _allocator: rcl_allocator_t,
    _args_output: *mut rcl_arguments_t,
) -> rcl_ret_t {
    tracing::warn!("rcl_parse_arguments is skipped with {:?}", parse_args(argc, argv));
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_count_unparsed(
    args: *const rcl_arguments_t,
) -> ::std::os::raw::c_int {
    tracing::warn!("rcl_arguments_get_count_unparsed is skipped");
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_unparsed(
    args: *const rcl_arguments_t,
    allocator: rcl_allocator_t,
    output_unparsed_indices: *mut *mut ::std::os::raw::c_int,
) -> rcl_ret_t {
    tracing::warn!("rcl_arguments_get_unparsed is skipped");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_count_unparsed_ros(
    args: *const rcl_arguments_t,
) -> ::std::os::raw::c_int {
    tracing::warn!("rcl_arguments_get_count_unparsed_ros is skipped");
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_unparsed_ros(
    args: *const rcl_arguments_t,
    allocator: rcl_allocator_t,
    output_unparsed_ros_indices: *mut *mut ::std::os::raw::c_int,
) -> rcl_ret_t {
    tracing::warn!("rcl_arguments_get_unparsed_ros is skipped");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_param_files_count(
    args: *const rcl_arguments_t,
) -> ::std::os::raw::c_int {
    tracing::warn!("rcl_arguments_get_param_files_count is skipped");
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_param_files(
    arguments: *const rcl_arguments_t,
    allocator: rcl_allocator_t,
    parameter_files: *mut *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    tracing::warn!("rcl_arguments_get_param_files is skipped");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_param_overrides(
    arguments: *const rcl_arguments_t,
    parameter_overrides: *mut *mut rcl_params_t,
) -> rcl_ret_t {
    tracing::warn!("rcl_arguments_get_param_overrides is skipped");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_remove_ros_arguments(
    argv: *const *const ::std::os::raw::c_char,
    args: *const rcl_arguments_t,
    allocator: rcl_allocator_t,
    nonros_argc: *mut ::std::os::raw::c_int,
    nonros_argv: *mut *mut *const ::std::os::raw::c_char,
) -> rcl_ret_t {
    tracing::warn!("rcl_remove_ros_arguments is skipped");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_get_log_levels(
    arguments: *const rcl_arguments_t,
    log_levels: *mut rcl_log_levels_t,
) -> rcl_ret_t {
    tracing::warn!("rcl_arguments_get_log_levels is skipped");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_copy(
    args: *const rcl_arguments_t,
    args_out: *mut rcl_arguments_t,
) -> rcl_ret_t {
    unimplemented!()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_arguments_fini(
    args: *mut rcl_arguments_t,
) -> rcl_ret_t {
    tracing::warn!("rcl_arguments_fini is skipped");
    RCL_RET_OK as _
}
