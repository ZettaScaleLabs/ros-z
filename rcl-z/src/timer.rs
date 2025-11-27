#![allow(unused)]

use std::ffi::c_longlong;
use std::time::SystemTimeError;
use std::time::UNIX_EPOCH;

use crate::impl_has_impl_ptr;
use crate::ros::*;
use crate::traits::BorrowImpl;
use crate::traits::OwnImpl;
use crate::traits::Waitable;
use std::time::{Duration, SystemTime};

#[derive(Debug)]
pub struct TimerImpl {
    period: Duration,
    last_call_time: SystemTime,
    next_call_time: SystemTime,
    callback: rcl_timer_callback_t,
}

// Internal data structure for ROS clock
#[derive(Debug, Default)]
struct RosClockData {
    ros_time_override_value: i64,
    ros_time_override_enabled: bool,
}

fn signed_duration_to_nanos(dur: Result<Duration, SystemTimeError>) -> rcl_time_point_value_t {
    match dur {
        Ok(d) => d.as_nanos() as _,
        Err(e) => -(e.duration().as_nanos() as i64),
    }
}

fn system_time_to_timestamp(time: SystemTime) -> i64 {
    time.duration_since(UNIX_EPOCH).unwrap().as_nanos() as _
}

impl TimerImpl {
    pub fn get_time_until_next_call(&self) -> Result<Duration, SystemTimeError> {
        self.next_call_time.duration_since(SystemTime::now())
    }

    pub fn update(&mut self, now: SystemTime) {
        self.last_call_time = now;
        if self.period.is_zero() {
            self.next_call_time = now;
        } else {
            self.next_call_time += self.period;
            if let Ok(ahead) = now.duration_since(self.next_call_time) {
                self.next_call_time += self.period
                    * (1 + (ahead.as_nanos() as f64 / self.period.as_nanos() as f64) as u32);
            }
        }
    }
}

impl Default for TimerImpl {
    fn default() -> Self {
        let now = SystemTime::now();
        Self {
            period: std::time::Duration::ZERO,
            last_call_time: now,
            next_call_time: now,
            callback: None,
        }
    }
}

impl Waitable for TimerImpl {
    fn is_ready(&self) -> bool {
        self.next_call_time <= SystemTime::now()
    }
}

impl_has_impl_ptr!(rcl_timer_t, rcl_timer_impl_t, TimerImpl);

impl rcl_timer_t {
    fn new() -> Self {
        let timer_impl = TimerImpl::default();
        Self {
            impl_: Box::into_raw(Box::new(timer_impl)) as _,
        }
    }
}

// NOTE: This is not presented in rclcpp
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_ros_clock_init(
    clock: *mut rcl_clock_t,
    allocator: *mut rcl_allocator_t,
) -> rcl_ret_t {
    if clock.is_null() || allocator.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        (*clock).type_ = rcl_clock_type_e::RCL_ROS_TIME;
        (*clock).jump_callbacks = std::ptr::null_mut();
        (*clock).num_jump_callbacks = 0;
        (*clock).get_now = None;
        (*clock).allocator = *allocator;

        // Initialize ROS time override data
        let data = Box::new(RosClockData::default());
        (*clock).data = Box::into_raw(data) as *mut _;
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_clock_init(
    clock_type: rcl_clock_type_e,
    clock: *mut rcl_clock_t,
    allocator: *mut rcl_allocator_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_clock_init");

    if clock.is_null() || allocator.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // UNINITIALIZED type is valid - just set it
    unsafe {
        (*clock).type_ = clock_type;
        (*clock).jump_callbacks = std::ptr::null_mut();
        (*clock).num_jump_callbacks = 0;
        (*clock).get_now = None;
        (*clock).data = std::ptr::null_mut();
        (*clock).allocator = *allocator;

        // Initialize ROS time data for ROS_TIME type
        if clock_type == rcl_clock_type_e::RCL_ROS_TIME {
            let data = Box::new(RosClockData::default());
            (*clock).data = Box::into_raw(data) as *mut _;
        }
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_timer() -> rcl_timer_t {
    tracing::trace!("rcl_get_zero_initialized_timer");
    rcl_timer_t::new()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_timer_init2(
    timer: *mut rcl_timer_t,
    clock: *mut rcl_clock_t,
    context: *mut rcl_context_t,
    period: i64,
    callback: rcl_timer_callback_t,
    allocator: rcl_allocator_t,
    autostart: bool,
) -> rcl_ret_t {
    tracing::trace!("rcl_timer_init2");

    // Check for null arguments
    if timer.is_null() || clock.is_null() || context.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check for negative period
    if period < 0 {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check for invalid allocator
    if allocator.allocate.is_none() || allocator.deallocate.is_none() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check for uninitialized clock
    unsafe {
        if (*clock).type_ == rcl_clock_type_e::RCL_CLOCK_UNINITIALIZED {
            return RCL_RET_ERROR as _;
        }
    }

    let now = SystemTime::now();
    match timer.borrow_mut_impl() {
        Ok(x) => {
            x.period = Duration::from_nanos(period as _);
            x.last_call_time = now;
            x.next_call_time = now + x.period;
            x.callback = callback;
            RCL_RET_OK as _
        }
        Err(_) => RCL_RET_ERROR as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_timer_fini(timer: *mut rcl_timer_t) -> rcl_ret_t {
    tracing::trace!("rcl_timer_fini");

    if timer.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if let Ok(impl_) = timer.own_impl() {
        drop(impl_);
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_clock_fini(clock: *mut rcl_clock_t) -> rcl_ret_t {
    tracing::trace!("rcl_clock_fini");

    if clock.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        let clock_ref = &mut *clock;

        // Cannot finalize UNINITIALIZED clock
        if clock_ref.type_ == rcl_clock_type_e::RCL_CLOCK_UNINITIALIZED {
            return RCL_RET_INVALID_ARGUMENT as _;
        }

        // Clean up ROS clock data
        if clock_ref.type_ == rcl_clock_type_e::RCL_ROS_TIME && !clock_ref.data.is_null() {
            let _ = Box::from_raw(clock_ref.data as *mut RosClockData);
            clock_ref.data = std::ptr::null_mut();
        }

        // Clean up jump callbacks
        if !clock_ref.jump_callbacks.is_null() && clock_ref.num_jump_callbacks > 0 {
            let callbacks = Vec::from_raw_parts(
                clock_ref.jump_callbacks,
                clock_ref.num_jump_callbacks,
                clock_ref.num_jump_callbacks,
            );
            drop(callbacks);
            clock_ref.jump_callbacks = std::ptr::null_mut();
            clock_ref.num_jump_callbacks = 0;
        }
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_timer_is_ready(
    timer: *const rcl_timer_t,
    is_ready: *mut bool,
) -> rcl_ret_t {
    tracing::trace!("rcl_timer_is_ready");
    unsafe {
        (*is_ready) = timer.borrow_impl().unwrap().is_ready();
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_timer_call(timer: *mut rcl_timer_t) -> rcl_ret_t {
    tracing::trace!("rcl_timer_call");
    let x = timer.borrow_mut_impl().unwrap();
    let now = SystemTime::now();
    let since_last_call = signed_duration_to_nanos(x.last_call_time.elapsed());
    x.update(now);
    if let Some(cb) = x.callback {
        unsafe {
            cb(timer, since_last_call);
        }
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_timer_call_with_info(
    timer: *mut rcl_timer_t,
    call_info: *mut rcl_timer_call_info_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_timer_call_with_info");
    let x = timer.borrow_mut_impl().unwrap();
    let now = SystemTime::now();
    let since_last_call = signed_duration_to_nanos(x.last_call_time.elapsed());
    unsafe {
        (*call_info).expected_call_time = system_time_to_timestamp(x.next_call_time);
        (*call_info).actual_call_time = system_time_to_timestamp(now);
    }
    x.update(now);
    if let Some(cb) = x.callback {
        unsafe {
            cb(timer, since_last_call);
        }
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_timer_get_time_until_next_call(
    timer: *const rcl_timer_t,
    time_until_next_call: *mut i64,
) -> rcl_ret_t {
    tracing::trace!("rcl_timer_get_time_until_next_call");

    let time = signed_duration_to_nanos(timer.borrow_impl().unwrap().get_time_until_next_call());
    unsafe {
        *time_until_next_call = time;
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_clock_get_now(
    clock: *mut rcl_clock_t,
    time_point_value: *mut rcl_time_point_value_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_clock_get_now");

    if clock.is_null() || time_point_value.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        let clock_ref = &*clock;

        // Check if this is a ROS clock with override enabled
        if clock_ref.type_ == rcl_clock_type_e::RCL_ROS_TIME && !clock_ref.data.is_null() {
            let data = &*(clock_ref.data as *const RosClockData);
            if data.ros_time_override_enabled {
                *time_point_value = data.ros_time_override_value;
                return RCL_RET_OK as _;
            }
        }

        // Check if clock has get_now function pointer
        if clock_ref.get_now.is_some() && !clock_ref.data.is_null() {
            return clock_ref.get_now.unwrap()(clock_ref.data, time_point_value);
        }

        // Otherwise return system time
        *time_point_value = system_time_to_timestamp(SystemTime::now());
    }
    RCL_RET_OK as _
}

// ROS time override functions
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_set_ros_time_override(
    clock: *mut rcl_clock_t,
    time_value: rcl_time_point_value_t,
) -> rcl_ret_t {
    if clock.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        let clock_ref = &mut *clock;

        if clock_ref.type_ != rcl_clock_type_e::RCL_ROS_TIME {
            return RCL_RET_ERROR as _;
        }

        if clock_ref.data.is_null() {
            return RCL_RET_ERROR as _;
        }

        let data = &mut *(clock_ref.data as *mut RosClockData);
        let old_value = data.ros_time_override_value;
        data.ros_time_override_value = time_value;

        // Trigger jump callbacks if enabled and value changed
        if data.ros_time_override_enabled && old_value != time_value {
            trigger_jump_callbacks(clock, old_value, time_value, rcl_clock_change_e::RCL_ROS_TIME_NO_CHANGE);
        }
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_is_enabled_ros_time_override(
    clock: *mut rcl_clock_t,
    is_enabled: *mut bool,
) -> rcl_ret_t {
    if clock.is_null() || is_enabled.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        let clock_ref = &*clock;

        if clock_ref.type_ != rcl_clock_type_e::RCL_ROS_TIME {
            return RCL_RET_ERROR as _;
        }

        if clock_ref.data.is_null() {
            return RCL_RET_ERROR as _;
        }

        let data = &*(clock_ref.data as *const RosClockData);
        *is_enabled = data.ros_time_override_enabled;
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_enable_ros_time_override(clock: *mut rcl_clock_t) -> rcl_ret_t {
    if clock.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        let clock_ref = &mut *clock;

        if clock_ref.type_ != rcl_clock_type_e::RCL_ROS_TIME {
            return RCL_RET_ERROR as _;
        }

        // Check if clock has been finalized or not properly initialized
        if clock_ref.data.is_null() {
            return RCL_RET_ERROR as _;
        }

        let data = &mut *(clock_ref.data as *mut RosClockData);

        // Only trigger callbacks if transitioning from disabled to enabled
        if !data.ros_time_override_enabled {
            data.ros_time_override_enabled = true;
            trigger_jump_callbacks(clock, 0, 0, rcl_clock_change_e::RCL_ROS_TIME_ACTIVATED);
        }
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_disable_ros_time_override(clock: *mut rcl_clock_t) -> rcl_ret_t {
    if clock.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        let clock_ref = &mut *clock;

        if clock_ref.type_ != rcl_clock_type_e::RCL_ROS_TIME {
            return RCL_RET_ERROR as _;
        }

        if clock_ref.data.is_null() {
            return RCL_RET_ERROR as _;
        }

        let data = &mut *(clock_ref.data as *mut RosClockData);

        // Only trigger callbacks if transitioning from enabled to disabled
        if data.ros_time_override_enabled {
            data.ros_time_override_enabled = false;
            trigger_jump_callbacks(clock, 0, 0, rcl_clock_change_e::RCL_ROS_TIME_DEACTIVATED);
        }
    }

    RCL_RET_OK as _
}

// Clock initialization functions for specific types
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_steady_clock_init(
    clock: *mut rcl_clock_t,
    allocator: *mut rcl_allocator_t,
) -> rcl_ret_t {
    rcl_clock_init(rcl_clock_type_e::RCL_STEADY_TIME, clock, allocator)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_system_clock_init(
    clock: *mut rcl_clock_t,
    allocator: *mut rcl_allocator_t,
) -> rcl_ret_t {
    rcl_clock_init(rcl_clock_type_e::RCL_SYSTEM_TIME, clock, allocator)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_ros_clock_fini(clock: *mut rcl_clock_t) -> rcl_ret_t {
    if clock.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        let clock_ref = &*clock;
        if clock_ref.type_ != rcl_clock_type_e::RCL_ROS_TIME {
            return RCL_RET_ERROR as _;
        }
    }

    rcl_clock_fini(clock)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_steady_clock_fini(clock: *mut rcl_clock_t) -> rcl_ret_t {
    if clock.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        let clock_ref = &*clock;
        if clock_ref.type_ != rcl_clock_type_e::RCL_STEADY_TIME {
            return RCL_RET_ERROR as _;
        }
    }

    rcl_clock_fini(clock)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_system_clock_fini(clock: *mut rcl_clock_t) -> rcl_ret_t {
    if clock.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        let clock_ref = &*clock;
        if clock_ref.type_ != rcl_clock_type_e::RCL_SYSTEM_TIME {
            return RCL_RET_ERROR as _;
        }
    }

    rcl_clock_fini(clock)
}

// Clock validation functions
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_clock_valid(clock: *mut rcl_clock_t) -> bool {
    if clock.is_null() {
        return false;
    }

    unsafe {
        let clock_ref = &*clock;
        // A clock is valid if it has a recognized type
        matches!(
            clock_ref.type_,
            rcl_clock_type_e::RCL_ROS_TIME
                | rcl_clock_type_e::RCL_SYSTEM_TIME
                | rcl_clock_type_e::RCL_STEADY_TIME
        )
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_clock_time_started(clock: *mut rcl_clock_t) -> bool {
    if clock.is_null() {
        return false;
    }

    let mut now: rcl_time_point_value_t = 0;
    unsafe {
        if rcl_clock_get_now(clock, &mut now) != RCL_RET_OK as i32 {
            return false;
        }
        now != 0
    }
}

// Time difference function
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_difference_times(
    start: *const rcl_time_point_t,
    finish: *const rcl_time_point_t,
    delta: *mut rcl_duration_t,
) -> rcl_ret_t {
    if start.is_null() || finish.is_null() || delta.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        let start_ref = &*start;
        let finish_ref = &*finish;

        // Check that clock types match
        if start_ref.clock_type != finish_ref.clock_type {
            return RCL_RET_ERROR as _;
        }

        // Calculate difference
        let diff = finish_ref.nanoseconds - start_ref.nanoseconds;
        (*delta).nanoseconds = diff;
    }

    RCL_RET_OK as _
}

// Jump callback management
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_clock_add_jump_callback(
    clock: *mut rcl_clock_t,
    threshold: rcl_jump_threshold_t,
    callback: rcl_jump_callback_t,
    user_data: *mut std::os::raw::c_void,
) -> rcl_ret_t {
    if clock.is_null() || callback.is_none() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Validate threshold
    if threshold.min_forward.nanoseconds < 0 {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if threshold.min_backward.nanoseconds > 0 {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        let clock_ref = &mut *clock;

        // Check if clock type supports jump callbacks
        if clock_ref.type_ == rcl_clock_type_e::RCL_CLOCK_UNINITIALIZED {
            return RCL_RET_INVALID_ARGUMENT as _;
        }

        // Check if this callback already exists
        if !clock_ref.jump_callbacks.is_null() {
            let callbacks = std::slice::from_raw_parts(
                clock_ref.jump_callbacks,
                clock_ref.num_jump_callbacks,
            );
            for cb in callbacks {
                if cb.callback == callback && cb.user_data == user_data {
                    return RCL_RET_ERROR as _;
                }
            }
        }

        // Create new callback info
        let info = rcl_jump_callback_info_t {
            callback,
            threshold,
            user_data,
        };

        // Reallocate callback array
        let new_size = clock_ref.num_jump_callbacks + 1;
        let layout = std::alloc::Layout::array::<rcl_jump_callback_info_t>(new_size).unwrap();

        let new_ptr = if clock_ref.jump_callbacks.is_null() {
            std::alloc::alloc(layout) as *mut rcl_jump_callback_info_t
        } else {
            let old_layout = std::alloc::Layout::array::<rcl_jump_callback_info_t>(clock_ref.num_jump_callbacks).unwrap();
            std::alloc::realloc(clock_ref.jump_callbacks as *mut u8, old_layout, layout.size()) as *mut rcl_jump_callback_info_t
        };

        if new_ptr.is_null() {
            return RCL_RET_BAD_ALLOC as _;
        }

        // Add new callback
        std::ptr::write(new_ptr.add(clock_ref.num_jump_callbacks), info);
        clock_ref.jump_callbacks = new_ptr;
        clock_ref.num_jump_callbacks = new_size;
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_clock_remove_jump_callback(
    clock: *mut rcl_clock_t,
    callback: rcl_jump_callback_t,
    user_data: *mut std::os::raw::c_void,
) -> rcl_ret_t {
    if clock.is_null() || callback.is_none() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        let clock_ref = &mut *clock;

        if clock_ref.jump_callbacks.is_null() || clock_ref.num_jump_callbacks == 0 {
            return RCL_RET_ERROR as _;
        }

        // Find callback index
        let mut found_index = None;
        let callbacks = std::slice::from_raw_parts(
            clock_ref.jump_callbacks,
            clock_ref.num_jump_callbacks,
        );
        for (i, cb) in callbacks.iter().enumerate() {
            if cb.callback == callback && cb.user_data == user_data {
                found_index = Some(i);
                break;
            }
        }

        let index = match found_index {
            Some(i) => i,
            None => return RCL_RET_ERROR as _,
        };

        // Remove callback by copying remaining callbacks
        if clock_ref.num_jump_callbacks == 1 {
            // Last callback - deallocate
            let layout = std::alloc::Layout::array::<rcl_jump_callback_info_t>(1).unwrap();
            std::alloc::dealloc(clock_ref.jump_callbacks as *mut u8, layout);
            clock_ref.jump_callbacks = std::ptr::null_mut();
            clock_ref.num_jump_callbacks = 0;
        } else {
            // Copy callbacks excluding the removed one
            let new_size = clock_ref.num_jump_callbacks - 1;
            let new_layout = std::alloc::Layout::array::<rcl_jump_callback_info_t>(new_size).unwrap();
            let new_ptr = std::alloc::alloc(new_layout) as *mut rcl_jump_callback_info_t;

            if new_ptr.is_null() {
                return RCL_RET_BAD_ALLOC as _;
            }

            // Copy callbacks before removed index
            if index > 0 {
                std::ptr::copy_nonoverlapping(
                    clock_ref.jump_callbacks,
                    new_ptr,
                    index,
                );
            }

            // Copy callbacks after removed index
            if index < new_size {
                std::ptr::copy_nonoverlapping(
                    clock_ref.jump_callbacks.add(index + 1),
                    new_ptr.add(index),
                    new_size - index,
                );
            }

            // Deallocate old array
            let old_layout = std::alloc::Layout::array::<rcl_jump_callback_info_t>(clock_ref.num_jump_callbacks).unwrap();
            std::alloc::dealloc(clock_ref.jump_callbacks as *mut u8, old_layout);

            clock_ref.jump_callbacks = new_ptr;
            clock_ref.num_jump_callbacks = new_size;
        }
    }

    RCL_RET_OK as _
}

// Helper function to trigger jump callbacks
unsafe fn trigger_jump_callbacks(
    clock: *mut rcl_clock_t,
    old_time: i64,
    new_time: i64,
    clock_change: rcl_clock_change_e,
) {
    unsafe {
        let clock_ref = &*clock;

        if clock_ref.jump_callbacks.is_null() || clock_ref.num_jump_callbacks == 0 {
            return;
        }

        let delta = new_time - old_time;
        let time_jump = rcl_time_jump_t {
            clock_change,
            delta: rcl_duration_t { nanoseconds: delta },
        };

        let callbacks = std::slice::from_raw_parts(
            clock_ref.jump_callbacks,
            clock_ref.num_jump_callbacks,
        );

        for cb_info in callbacks {
            // Check if callback should be triggered
            let should_trigger = if clock_change != rcl_clock_change_e::RCL_ROS_TIME_NO_CHANGE {
                // Clock change event
                cb_info.threshold.on_clock_change
            } else if delta > 0 {
                // Forward jump - only trigger if threshold is set (> 0) and exceeded
                cb_info.threshold.min_forward.nanoseconds > 0
                    && delta >= cb_info.threshold.min_forward.nanoseconds
            } else if delta < 0 {
                // Backward jump - only trigger if threshold is set (< 0) and exceeded
                cb_info.threshold.min_backward.nanoseconds < 0
                    && delta <= cb_info.threshold.min_backward.nanoseconds
            } else {
                false
            };

            if should_trigger {
                if let Some(callback) = cb_info.callback {
                    // Call pre-jump callback
                    callback(&time_jump, true, cb_info.user_data);
                    // Call post-jump callback
                    callback(&time_jump, false, cb_info.user_data);
                }
            }
        }
    }
}
