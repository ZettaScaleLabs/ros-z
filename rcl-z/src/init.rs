use ros_z::Builder;
use ros_z::context::ZContextBuilder;

use crate::context::ContextImpl;
use crate::traits::{BorrowImpl, OwnImpl};
use crate::{impl_has_impl_ptr, rclz_try, ros::*};
use std::ffi::{c_char, c_int};

#[derive(Debug, Default, Clone, Copy)]
pub struct InitOptionsImpl {
    domain_id: usize,
    allocator: rcl_allocator_t,
}

impl_has_impl_ptr!(rcl_init_options_t, rcl_init_options_impl_t, InitOptionsImpl);

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_init(
    init_options: *mut rcl_init_options_t,
    allocator: rcl_allocator_t,
) -> rcl_ret_t {
    let mut opts_impl = InitOptionsImpl::default();
    opts_impl.allocator = allocator;
    rclz_try! {
        init_options.assign_impl(opts_impl)?;
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_get_domain_id(
    init_options: *const rcl_init_options_t,
    domain_id: *mut usize,
) -> rcl_ret_t {
    tracing::trace!("rcl_init_options_get_domain_id");
    rclz_try! {
        unsafe {
            *domain_id = init_options
                .borrow_impl()?
                .domain_id;
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_set_domain_id(
    init_options: *mut rcl_init_options_t,
    domain_id: usize,
) -> rcl_ret_t {
    tracing::trace!("rcl_init_options_set_domain_id");
    rclz_try! {
        init_options
            .borrow_mut_impl()?
            .domain_id = domain_id;
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_fini(init_options: *mut rcl_init_options_t) -> rcl_ret_t {
    // FIXME: tracing is not usable at the exit stage
    // tracing::trace!("rcl_init_options_fini");

    rclz_try! {
        std::mem::drop(init_options.own_impl()?);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_copy(
    src: *const rcl_init_options_t,
    dst: *mut rcl_init_options_t,
) -> rcl_ret_t {
    rclz_try! {
        dst.assign_impl(src.borrow_impl()?.clone())?;
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_get_allocator(
    init_options: *const rcl_init_options_t,
) -> *const rcl_allocator_t {
    tracing::trace!("rcl_init_options_get_allocator");

    match init_options.borrow_impl() {
        Ok(x) => &x.allocator,
        Err(e) => {
            tracing::error!("rcl_init_options_get_allocator failed with {e}");
            std::ptr::null()
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init(
    argc: c_int,
    argv: *const *const c_char,
    options: *const rcl_init_options_t,
    context: *mut rcl_context_t,
) -> rcl_ret_t {
    zenoh::init_log_from_env_or("error");
    tracing::error!(
        "rcl_init with args: {:?}",
        crate::utils::parse_args(argc, argv)
    );
    let ctx = ZContextBuilder::default()
        .with_domain_id(options.borrow_impl().unwrap().domain_id)
        .build()
        .unwrap();
    let ctx_impl = ContextImpl::new(ctx);
    context.assign_impl(ctx_impl).unwrap();
    RCL_RET_OK as _
}
