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
    _allocator: *mut rcl_allocator_t,
) -> rcl_ret_t {
    unsafe {
        (*clock).type_ = rcl_clock_type_e::RCL_ROS_TIME;
        (*clock).jump_callbacks = std::ptr::null_mut();
        (*clock).num_jump_callbacks = 0;
        (*clock).get_now = None;
        (*clock).data = std::ptr::null_mut();
        (*clock).allocator = rcl_allocator_t::default();
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_clock_init(
    clock_type: rcl_clock_type_t,
    clock: *mut rcl_clock_t,
    _allocator: *mut rcl_allocator_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_clock_init");
    tracing::warn!("rcl_clock_init is skipped.");
    // match clock_type {
    //     rcl_clock_type_e::RCL_ROS_TIME => {
    //         let ret = rcl_ros_clock_init(clock, _allocator);
    //         if ret != RCL_RET_OK as i32 {
    //             tracing::error!("rcl_clock_init failed with {ret}");
    //         }
    //     }
    //     _ => {
    //         todo!()
    //     }
    // }
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
    let now = SystemTime::now();
    let x = timer.borrow_mut_impl().unwrap();
    x.period = Duration::from_nanos(period as _);
    x.last_call_time = now;
    x.next_call_time = now + x.period;
    x.callback = callback;
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_timer_fini(timer: *mut rcl_timer_t) -> rcl_ret_t {
    tracing::trace!("rcl_timer_fini");
    drop(timer.own_impl().unwrap());
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_clock_fini(clock: *mut rcl_clock_t) -> rcl_ret_t {
    tracing::trace!("rcl_clock_fini");
    tracing::warn!("rcl_clock_fini is skipped");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_timer_is_ready(timer: *const rcl_timer_t, is_ready: *mut bool) -> rcl_ret_t {
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
    _clock: *mut rcl_clock_t,
    time_point_value: *mut rcl_time_point_value_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_clock_get_now");
    unsafe {
        *time_point_value = system_time_to_timestamp(SystemTime::now());
    }
    RCL_RET_OK as _
}
