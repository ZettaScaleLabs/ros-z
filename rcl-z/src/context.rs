use std::{ffi::{c_char, c_int}, ops::Deref};
use crate::ros::*;
use ros_z::context::ZContext;

pub struct ContextImpl(ZContext);

impl Deref for ContextImpl {
    type Target = ZContext;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_init(
    _init_options: *mut rcl_init_options_t,
    _allocator: rcl_allocator_t,
) -> rcl_ret_t {
    RCL_RET_OK as _
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
        (*boxed)
            .0
            .shutdown()
            .unwrap();

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
