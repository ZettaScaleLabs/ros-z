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
    fn trigger(&mut self) {
        let notifier = self
            .notifier
            .as_ref()
            .expect("GuardConditionImpl has no Notifier");
        let _ = notifier.mutex.lock();
        self.triggered = true;
        notifier.cv.notify_all();
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

impl rcl_guard_condition_t {
    fn new() -> Self {
        let gc_impl = GuardConditionImpl::default();
        let mut gc = Self::default();

        gc.impl_ = Box::into_raw(Box::new(gc_impl)) as _;
        gc
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_guard_condition() -> rcl_guard_condition_t {
    tracing::trace!("rcl_get_zero_initialized_guard_condition");
    rcl_guard_condition_t::new()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_guard_condition_init(
    guard_condition: *mut rcl_guard_condition_t,
    context: *mut rcl_context_t,
    _options: rcl_guard_condition_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_guard_condition_init");
    unsafe {
        let x = &mut *((*guard_condition).impl_ as *mut GuardConditionImpl);
        x.notifier = Some(context.borrow_impl().unwrap().share_notifier());
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
    std::mem::drop(guard_condition.own_impl().unwrap());
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_trigger_guard_condition(
    guard_condition: *mut rcl_guard_condition_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_trigger_guard_condition");
    guard_condition.borrow_mut_impl().unwrap().trigger();
    RCL_RET_OK as _
}
