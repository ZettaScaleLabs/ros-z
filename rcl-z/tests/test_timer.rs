#![cfg(feature = "test-core")]
#![allow(unused_unsafe)]

use std::{
    ptr,
    sync::atomic::{AtomicU8, Ordering},
};

use rcl_z::{
    context::{rcl_context_fini, rcl_get_zero_initialized_context, rcl_shutdown},
    init::{
        rcl_get_default_allocator, rcl_get_zero_initialized_init_options, rcl_init,
        rcl_init_options_fini, rcl_init_options_init,
    },
    ros::{
        RCL_RET_ERROR, RCL_RET_INVALID_ARGUMENT, RCL_RET_OK, rcl_allocator_t, rcl_clock_t,
        rcl_clock_type_e, rcl_timer_call_info_t, rcl_timer_t,
    },
    timer::{
        rcl_clock_fini, rcl_clock_get_now, rcl_clock_init, rcl_get_zero_initialized_timer,
        rcl_ros_clock_init, rcl_timer_call, rcl_timer_call_with_info, rcl_timer_fini,
        rcl_timer_get_time_until_next_call, rcl_timer_init2, rcl_timer_is_ready,
    },
};

// Helper macros for time conversion
const NANOS_PER_MS: i64 = 1_000_000;
const NANOS_PER_SEC: i64 = 1_000_000_000;

static TIMES_CALLED: AtomicU8 = AtomicU8::new(0);

unsafe extern "C" fn callback_function(_timer: *mut rcl_timer_t, _last_call: i64) {
    TIMES_CALLED.fetch_add(1, Ordering::SeqCst);
}

#[allow(dead_code)]
unsafe extern "C" fn callback_function_changed(_timer: *mut rcl_timer_t, _last_call: i64) {
    TIMES_CALLED.fetch_sub(1, Ordering::SeqCst);
}

/// Test timer init with invalid arguments
#[test]
fn test_timer_init_with_invalid_arguments() {
    unsafe {
        // Initialize context
        let mut init_options = rcl_get_zero_initialized_init_options();
        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut context = rcl_get_zero_initialized_context();
        let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut clock = rcl_clock_t::default();
        let mut allocator = rcl_get_default_allocator();
        let ret = rcl_clock_init(
            rcl_clock_type_e::RCL_STEADY_TIME,
            &mut clock,
            &mut allocator as *mut _,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        let _timer = rcl_get_zero_initialized_timer();

        // Test with null timer
        let ret = rcl_timer_init2(
            ptr::null_mut(),
            &mut clock,
            &mut context,
            50 * NANOS_PER_MS,
            None,
            allocator,
            true,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with null clock
        let mut timer = rcl_get_zero_initialized_timer();
        let ret = rcl_timer_init2(
            &mut timer,
            ptr::null_mut(),
            &mut context,
            50 * NANOS_PER_MS,
            None,
            allocator,
            true,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with null context
        let ret = rcl_timer_init2(
            &mut timer,
            &mut clock,
            ptr::null_mut(),
            50 * NANOS_PER_MS,
            None,
            allocator,
            true,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with negative period
        let ret = rcl_timer_init2(
            &mut timer,
            &mut clock,
            &mut context,
            -1,
            None,
            allocator,
            true,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with invalid allocator
        let invalid_allocator = rcl_allocator_t::default();
        let ret = rcl_timer_init2(
            &mut timer,
            &mut clock,
            &mut context,
            50 * NANOS_PER_MS,
            None,
            invalid_allocator,
            true,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Cleanup
        let ret = rcl_clock_fini(&mut clock);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_context_fini(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_init_options_fini(&mut init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test timer with invalid clock type
#[test]
fn test_timer_with_invalid_clock() {
    unsafe {
        // Initialize context
        let mut init_options = rcl_get_zero_initialized_init_options();
        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut context = rcl_get_zero_initialized_context();
        let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut clock = rcl_clock_t::default();
        let mut allocator = rcl_get_default_allocator();
        let ret = rcl_clock_init(
            rcl_clock_type_e::RCL_CLOCK_UNINITIALIZED,
            &mut clock,
            &mut allocator as *mut _,
        );
        assert_eq!(ret, RCL_RET_ERROR as i32);

        // Cleanup
        let ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_context_fini(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_init_options_fini(&mut init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test timer initialization and basic operations
#[test]
fn test_timer_init_and_call() {
    unsafe {
        // Initialize context
        let mut init_options = rcl_get_zero_initialized_init_options();
        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut context = rcl_get_zero_initialized_context();
        let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut clock = rcl_clock_t::default();
        let mut allocator = rcl_get_default_allocator();
        let ret = rcl_clock_init(
            rcl_clock_type_e::RCL_ROS_TIME,
            &mut clock,
            &mut allocator as *mut _,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Reset counter
        TIMES_CALLED.store(0, Ordering::SeqCst);

        // Create timer
        let mut timer = rcl_get_zero_initialized_timer();
        let ret = rcl_timer_init2(
            &mut timer,
            &mut clock,
            &mut context,
            NANOS_PER_SEC,
            Some(callback_function),
            allocator,
            true,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check if timer is ready (should not be ready immediately)
        let mut is_ready = true;
        let ret = rcl_timer_is_ready(&timer, &mut is_ready);
        assert_eq!(ret, RCL_RET_OK as i32);
        // Timer with 1 second period should not be ready immediately
        // (though this depends on timing)

        // Get time until next call
        let mut time_until_next = 0i64;
        let ret = rcl_timer_get_time_until_next_call(&timer, &mut time_until_next);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Call the timer
        let ret = rcl_timer_call(&mut timer);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(TIMES_CALLED.load(Ordering::SeqCst), 1);

        // Call with info
        let mut call_info = rcl_timer_call_info_t::default();
        let ret = rcl_timer_call_with_info(&mut timer, &mut call_info);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(TIMES_CALLED.load(Ordering::SeqCst), 2);
        assert!(call_info.expected_call_time > 0);
        assert!(call_info.actual_call_time > 0);

        // Cleanup
        let ret = rcl_timer_fini(&mut timer);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_clock_fini(&mut clock);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_context_fini(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_init_options_fini(&mut init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test timer with zero period
#[test]
fn test_timer_with_zero_period() {
    unsafe {
        // Initialize context
        let mut init_options = rcl_get_zero_initialized_init_options();
        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut context = rcl_get_zero_initialized_context();
        let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut clock = rcl_clock_t::default();
        let mut allocator = rcl_get_default_allocator();
        let ret = rcl_clock_init(
            rcl_clock_type_e::RCL_ROS_TIME,
            &mut clock,
            &mut allocator as *mut _,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Create timer with zero period
        let mut timer = rcl_get_zero_initialized_timer();
        let ret = rcl_timer_init2(
            &mut timer,
            &mut clock,
            &mut context,
            0,
            None,
            allocator,
            true,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        // Cleanup
        let ret = rcl_timer_fini(&mut timer);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_clock_fini(&mut clock);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_shutdown(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_context_fini(&mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_init_options_fini(&mut init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test clock get_now
#[test]
fn test_clock_get_now() {
    unsafe {
        let mut clock = rcl_clock_t::default();
        let mut allocator = rcl_get_default_allocator();
        let ret = rcl_ros_clock_init(&mut clock, &mut allocator as *mut _);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut now = 0i64;
        let ret = rcl_clock_get_now(&mut clock, &mut now);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(now > 0);

        let ret = rcl_clock_fini(&mut clock);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

// TODO: Port additional timer tests that require:
// - rcl_timer_clock() function
// - rcl_timer_cancel() function
// - rcl_timer_is_canceled() function
// - rcl_timer_reset() function
// - rcl_timer_get_period() function
// - rcl_timer_exchange_callback() function
// - ROS time override functionality
// - Jump callback functionality
