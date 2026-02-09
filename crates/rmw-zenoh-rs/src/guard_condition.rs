use crate::rmw_impl_has_data_ptr;
use crate::ros::*;
use crate::traits::*;
use crate::utils::Notifier;
use std::sync::Arc;

/// Guard condition implementation for RMW
#[derive(Debug, Default)]
pub struct GuardConditionImpl {
    pub(crate) notifier: Option<Arc<Notifier>>,
    pub(crate) triggered: bool,
}

impl GuardConditionImpl {
    pub(crate) fn trigger(&mut self) -> Result<(), ()> {
        let notifier = self.notifier.as_ref().ok_or(())?;
        self.triggered = true;
        notifier.notify_all();
        Ok(())
    }

    pub fn reset(&mut self) {
        self.triggered = false;
    }
}

impl crate::traits::Waitable for GuardConditionImpl {
    fn is_ready(&self) -> bool {
        self.triggered
    }
}

rmw_impl_has_data_ptr!(
    rmw_guard_condition_t,
    rmw_guard_condition_impl_t,
    GuardConditionImpl
);

// RMW Guard Condition Functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_guard_condition(
    context: *mut rmw_context_t,
) -> *mut rmw_guard_condition_t {
    if context.is_null() {
        return std::ptr::null_mut();
    }

    let context_impl = match context.borrow_impl() {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };

    let notifier = Some(context_impl.share_notifier());
    let gc_impl = GuardConditionImpl {
        notifier,
        triggered: false,
    };
    let gc = Box::new(rmw_guard_condition_t {
        implementation_identifier: crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        context,
    });

    let gc_ptr = Box::into_raw(gc);
    gc_ptr.assign_data(gc_impl).unwrap_or(());

    gc_ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_guard_condition(
    guard_condition: *mut rmw_guard_condition_t,
) -> rmw_ret_t {
    if guard_condition.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Drop the implementation data
    let _ = guard_condition.own_data();

    drop(unsafe { Box::from_raw(guard_condition) });
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_trigger_guard_condition(
    guard_condition: *const rmw_guard_condition_t,
) -> rmw_ret_t {
    if guard_condition.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    if let Ok(gc_impl) = (guard_condition as *mut rmw_guard_condition_t).borrow_mut_data() {
        let _ = gc_impl.trigger();
    }

    RMW_RET_OK as _
}
