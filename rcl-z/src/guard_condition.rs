use std::sync::Arc;

use crate::{
    impl_has_impl_ptr,
    ros::*,
    traits::{BorrowImpl, OwnImpl, Waitable},
    utils::Notifier,
};

#[derive(Debug, Default)]
pub struct GuardConditionImpl {
    notifier: Option<Arc<Notifier>>,
    triggered: bool,
}

impl Waitable for GuardConditionImpl {
    fn is_ready(&self) -> bool {
        self.triggered
    }
}

impl GuardConditionImpl {
    fn trigger(&mut self) -> Result<(), ()> {
        let notifier = self
            .notifier
            .as_ref()
            .ok_or(())?;
        std::mem::drop(notifier.mutex.lock());
        self.triggered = true;
        notifier.cv.notify_all();
        Ok(())
    }

    pub fn reset(&mut self) {
        self.triggered = false;
    }
}

impl_has_impl_ptr!(
    rcl_guard_condition_t,
    rcl_guard_condition_impl_t,
    GuardConditionImpl
);

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_guard_condition() -> rcl_guard_condition_t {
    tracing::trace!("rcl_get_zero_initialized_guard_condition");
    // Return a zero-initialized guard condition (impl_ should be null)
    unsafe { std::mem::zeroed() }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_guard_condition_init(
    guard_condition: *mut rcl_guard_condition_t,
    context: *mut rcl_context_t,
    _options: rcl_guard_condition_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_guard_condition_init");

    // Check for null arguments
    if guard_condition.is_null() || context.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        // Check if context is valid
        if (*context).impl_.is_null() {
            return RCL_RET_NOT_INIT as _;
        }

        // Check if guard_condition is already initialized
        if !(*guard_condition).impl_.is_null() {
            return RCL_RET_ALREADY_INIT as _;
        }

        // Create the GuardConditionImpl
        let notifier = match context.borrow_impl() {
            Ok(ctx) => Some(ctx.share_notifier()),
            Err(_) => return RCL_RET_ERROR as _,
        };

        let impl_ = Box::new(GuardConditionImpl {
            notifier,
            triggered: false,
        });
        (*guard_condition).impl_ = Box::into_raw(impl_) as *mut _;
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_guard_condition_get_options(
    _guard_condition: *const rcl_guard_condition_t,
) -> *const rcl_guard_condition_options_t {
    tracing::trace!("rcl_guard_condition_get_options");
    std::ptr::null()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_guard_condition_init_from_rmw(
    _guard_condition: *mut rcl_guard_condition_t,
    _rmw_guard_condition: *const rmw_guard_condition_t,
    _context: *mut rcl_context_t,
    _options: rcl_guard_condition_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_guard_condition_init_from_rmw");
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_guard_condition_fini(
    guard_condition: *mut rcl_guard_condition_t,
) -> rcl_ret_t {
    // FIXME: TLS issue
    // tracing::trace!("rcl_guard_condition_fini");

    if guard_condition.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // It's OK to call fini on an uninitialized guard condition
    if let Ok(impl_) = guard_condition.own_impl() {
        std::mem::drop(impl_);
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_trigger_guard_condition(
    guard_condition: *mut rcl_guard_condition_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_trigger_guard_condition");

    if guard_condition.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    match guard_condition.borrow_mut_impl() {
        Ok(impl_) => {
            if impl_.trigger().is_err() {
                return RCL_RET_INVALID_ARGUMENT as _;
            }
        }
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    }
    RCL_RET_OK as _
}
