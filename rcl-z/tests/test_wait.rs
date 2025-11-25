// Ported from Open Source Robotics Foundation code (2016)
// https://github.com/ros2/rcl
// Licensed under the Apache License 2.0
// Copyright 2025 ZettaScale Technology

#![cfg(feature = "test-core")]

use std::ptr;

use rcl_z::{
    context::{rcl_context_fini, rcl_get_zero_initialized_context, rcl_shutdown},
    guard_condition::{
        rcl_get_zero_initialized_guard_condition, rcl_guard_condition_fini,
        rcl_guard_condition_init, rcl_trigger_guard_condition,
    },
    init::{
        rcl_get_default_allocator, rcl_get_zero_initialized_init_options, rcl_init,
        rcl_init_options_fini, rcl_init_options_init,
    },
    ros::*,
    timer::{
        rcl_clock_fini, rcl_clock_init, rcl_get_zero_initialized_timer, rcl_timer_fini, rcl_timer_init2,
    },
    wait_set::{
        rcl_get_zero_initialized_wait_set, rcl_wait, rcl_wait_set_add_guard_condition,
        rcl_wait_set_add_timer, rcl_wait_set_fini, rcl_wait_set_get_allocator,
        rcl_wait_set_init, rcl_wait_set_is_valid, rcl_wait_set_resize,
    },
};

/// Helper function to get default guard condition options
fn rcl_guard_condition_get_default_options() -> rcl_guard_condition_options_t {
    rcl_guard_condition_options_t {
        allocator: rcl_get_default_allocator(),
    }
}

/// Test fixture that provides an initialized RCL context for wait set tests
struct TestWaitSetFixture {
    context: rcl_context_t,
    init_options: rcl_init_options_t,
}

impl TestWaitSetFixture {
    fn new() -> Self {
        let mut init_options = rcl_get_zero_initialized_init_options();
        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut context = rcl_get_zero_initialized_context();
        let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
        assert_eq!(ret, RCL_RET_OK as i32);

        TestWaitSetFixture {
            context,
            init_options,
        }
    }

    fn context(&mut self) -> *mut rcl_context_t {
        &mut self.context
    }
}

impl Drop for TestWaitSetFixture {
    fn drop(&mut self) {
        let ret = rcl_shutdown(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_context_fini(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_init_options_fini(&mut self.init_options);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

#[test]
fn test_wait_set_is_valid() {
    // null pointers are invalid
    assert!(!rcl_wait_set_is_valid(ptr::null()));

    // uninitialized wait set is invalid
    let wait_set = rcl_get_zero_initialized_wait_set();
    assert!(!rcl_wait_set_is_valid(&wait_set));

    let mut fixture = TestWaitSetFixture::new();

    // initialized wait set is valid
    let mut wait_set = rcl_get_zero_initialized_wait_set();
    let ret = unsafe { rcl_wait_set_init(
        &mut wait_set,
        1,
        1,
        1,
        1,
        1,
        0,
        fixture.context(),
        rcl_get_default_allocator(),
    ) };
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_wait_set_fini(&mut wait_set);
    assert_eq!(ret, RCL_RET_OK as i32);
}

#[test]
fn test_wait_set_valid_arguments() {
    let mut wait_set = rcl_get_zero_initialized_wait_set();
    let ret = unsafe { rcl_wait_set_init(
        &mut wait_set,
        0,
        0,
        0,
        0,
        0,
        0,
        ptr::null_mut(),
        rcl_get_default_allocator(),
    ) };
    assert_eq!(ret, 11);

    let mut fixture = TestWaitSetFixture::new();

    let ret = unsafe { rcl_wait_set_init(
        &mut wait_set,
        0,
        0,
        0,
        0,
        0,
        0,
        fixture.context(),
        rcl_get_default_allocator(),
    ) };
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_wait(&mut wait_set, 1_000_000_000);
    assert_eq!(ret, RCL_RET_WAIT_SET_EMPTY as i32);

    let ret = rcl_wait_set_fini(&mut wait_set);
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_wait(ptr::null_mut(), 1_000_000_000);
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    let ret = rcl_wait(&mut wait_set, 1_000_000_000);
    assert_eq!(ret, RCL_RET_WAIT_SET_INVALID as i32);

    let mut not_init_context = rcl_get_zero_initialized_context();
    let ret = unsafe { rcl_wait_set_init(
        &mut wait_set,
        1,
        1,
        1,
        1,
        1,
        0,
        &mut not_init_context,
        rcl_get_default_allocator(),
    ) };
    assert_eq!(ret, RCL_RET_NOT_INIT as i32);

    let ret = unsafe { rcl_wait_set_init(
        ptr::null_mut(),
        1,
        1,
        1,
        1,
        1,
        0,
        fixture.context(),
        rcl_get_default_allocator(),
    ) };
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    let ret = unsafe { rcl_wait_set_init(
        &mut wait_set,
        1,
        1,
        1,
        1,
        1,
        0,
        ptr::null_mut(),
        rcl_get_default_allocator(),
    ) };
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    let ret = unsafe { rcl_wait_set_init(
        &mut wait_set,
        1,
        1,
        1,
        1,
        1,
        0,
        fixture.context(),
        rcl_get_default_allocator(),
    ) };
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = unsafe { rcl_wait_set_init(
        &mut wait_set,
        1,
        1,
        1,
        1,
        1,
        0,
        fixture.context(),
        rcl_get_default_allocator(),
    ) };
    assert_eq!(ret, RCL_RET_ALREADY_INIT as i32);

    let ret = rcl_wait_set_fini(&mut wait_set);
    assert_eq!(ret, RCL_RET_OK as i32);
}

#[test]
fn test_wait_set_get_allocator() {
    let mut fixture = TestWaitSetFixture::new();

    let mut allocator_returned = rcl_allocator_t::default();
    let mut wait_set = rcl_get_zero_initialized_wait_set();

    let ret = rcl_wait_set_get_allocator(ptr::null_mut(), &mut allocator_returned);
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    let ret = rcl_wait_set_get_allocator(&wait_set, ptr::null_mut());
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    let ret = rcl_wait_set_get_allocator(&wait_set, &mut allocator_returned);
    assert_eq!(ret, RCL_RET_WAIT_SET_INVALID as i32);

    let ret = unsafe { rcl_wait_set_init(
        &mut wait_set,
        1,
        1,
        1,
        1,
        1,
        0,
        fixture.context(),
        rcl_get_default_allocator(),
    ) };
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_wait_set_get_allocator(&wait_set, ptr::null_mut());
    assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

    let ret = rcl_wait_set_get_allocator(&wait_set, &mut allocator_returned);
    assert_eq!(ret, RCL_RET_OK as i32);
    // Note: In C++ they check rcutils_allocator_is_valid, but in Rust we can't easily check that

    let ret = rcl_wait_set_fini(&mut wait_set);
    assert_eq!(ret, RCL_RET_OK as i32);
}

#[test]
fn test_excess_capacity() {
    let mut fixture = TestWaitSetFixture::new();

    let mut wait_set = rcl_get_zero_initialized_wait_set();
    let ret = unsafe { rcl_wait_set_init(
        &mut wait_set,
        42,
        42,
        42,
        42,
        42,
        0,
        fixture.context(),
        rcl_get_default_allocator(),
    ) };
    assert_eq!(ret, RCL_RET_OK as i32);

    let timeout = 1;
    let ret = rcl_wait(&mut wait_set, timeout);
    assert_eq!(ret, RCL_RET_TIMEOUT as i32);

    let ret = rcl_wait_set_fini(&mut wait_set);
    assert_eq!(ret, RCL_RET_OK as i32);
}

#[test]
fn test_finite_timeout() {
    let mut fixture = TestWaitSetFixture::new();

    let mut wait_set = rcl_get_zero_initialized_wait_set();
    let ret = unsafe { rcl_wait_set_init(
        &mut wait_set,
        0,
        0,
        1,
        0,
        0,
        0,
        fixture.context(),
        rcl_get_default_allocator(),
    ) };
    assert_eq!(ret, RCL_RET_OK as i32);

    let timeout = 10_000_000; // 10ms in nanoseconds
    let before = std::time::Instant::now();
    let ret = rcl_wait(&mut wait_set, timeout);
    let elapsed = before.elapsed().as_nanos() as i64;

    assert_eq!(ret, RCL_RET_TIMEOUT as i32);
    // Check that the timeout was approximately correct (with some tolerance)
    assert!(elapsed >= timeout && elapsed < timeout + 50_000_000); // 50ms tolerance

    let ret = rcl_wait_set_fini(&mut wait_set);
    assert_eq!(ret, RCL_RET_OK as i32);
}

#[test]
fn test_guard_condition_trigger() {
    let mut fixture = TestWaitSetFixture::new();

    let mut wait_set = rcl_get_zero_initialized_wait_set();
    let ret = unsafe { rcl_wait_set_init(
        &mut wait_set,
        0,
        1,
        0,
        0,
        0,
        0,
        fixture.context(),
        rcl_get_default_allocator(),
    ) };
    assert_eq!(ret, RCL_RET_OK as i32);

    let mut guard_cond = rcl_get_zero_initialized_guard_condition();
    let ret = unsafe {
        rcl_guard_condition_init(
            &mut guard_cond,
            fixture.context(),
            rcl_guard_condition_get_default_options(),
        )
    };
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_wait_set_add_guard_condition(&mut wait_set, &guard_cond, ptr::null_mut());
    assert_eq!(ret, RCL_RET_OK as i32);

    // Trigger the guard condition
    let ret = rcl_trigger_guard_condition(&mut guard_cond);
    assert_eq!(ret, RCL_RET_OK as i32);

    // Wait should return immediately since guard condition was triggered
    let before = std::time::Instant::now();
    let ret = rcl_wait(&mut wait_set, 1_000_000_000); // 1 second timeout
    let elapsed = before.elapsed().as_millis();

    assert_eq!(ret, RCL_RET_OK as i32);
    assert!(elapsed < 100); // Should return almost immediately

    let ret = rcl_guard_condition_fini(&mut guard_cond);
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_wait_set_fini(&mut wait_set);
    assert_eq!(ret, RCL_RET_OK as i32);
}

#[test]
fn test_wait_set_resize_to_zero() {
    let mut fixture = TestWaitSetFixture::new();

    let mut wait_set = rcl_get_zero_initialized_wait_set();
    let ret = unsafe { rcl_wait_set_init(
        &mut wait_set,
        1,
        1,
        1,
        1,
        1,
        0,
        fixture.context(),
        rcl_get_default_allocator(),
    ) };
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_wait_set_resize(&mut wait_set, 0, 0, 0, 0, 0, 0);
    assert_eq!(ret, RCL_RET_OK as i32);

    assert_eq!(wait_set.size_of_subscriptions, 0);
    assert_eq!(wait_set.size_of_guard_conditions, 0);
    assert_eq!(wait_set.size_of_clients, 0);
    assert_eq!(wait_set.size_of_services, 0);
    assert_eq!(wait_set.size_of_timers, 0);

    let ret = rcl_wait_set_fini(&mut wait_set);
    assert_eq!(ret, RCL_RET_OK as i32);
}

#[test]
fn test_timer_with_wait() {
    let mut fixture = TestWaitSetFixture::new();

    let mut wait_set = rcl_get_zero_initialized_wait_set();
    let ret = unsafe { rcl_wait_set_init(
        &mut wait_set,
        0,
        1,
        1,
        0,
        0,
        0,
        fixture.context(),
        rcl_get_default_allocator(),
    ) };
    assert_eq!(ret, RCL_RET_OK as i32);

    // Add a dummy guard condition to avoid empty wait set
    let mut guard_cond = rcl_get_zero_initialized_guard_condition();
    let ret = unsafe {
        rcl_guard_condition_init(
            &mut guard_cond,
            fixture.context(),
            rcl_guard_condition_get_default_options(),
        )
    };
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_wait_set_add_guard_condition(&mut wait_set, &guard_cond, ptr::null_mut());
    assert_eq!(ret, RCL_RET_OK as i32);

    // Create a clock and timer
    let mut clock = rcl_clock_t::default();
    let mut allocator = rcl_get_default_allocator();
    let ret = rcl_clock_init(rcl_clock_type_e::RCL_STEADY_TIME, &mut clock, &mut allocator as *mut _);
    assert_eq!(ret, RCL_RET_OK as i32);

    let mut timer = rcl_get_zero_initialized_timer();
    let ret = rcl_timer_init2(
        &mut timer,
        &mut clock,
        fixture.context(),
        10_000_000, // 10ms
        None,
        rcl_get_default_allocator(),
        true,
    );
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_wait_set_add_timer(&mut wait_set, &timer, ptr::null_mut());
    assert_eq!(ret, RCL_RET_OK as i32);

    // Wait with a negative timeout (should be overridden by timer)
    let before = std::time::Instant::now();
    let ret = rcl_wait(&mut wait_set, -1);
    let elapsed = before.elapsed().as_millis();

    assert_eq!(ret, RCL_RET_OK as i32);
    assert!(elapsed < 100); // Timer should trigger within reasonable time

    let ret = rcl_timer_fini(&mut timer);
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_clock_fini(&mut clock);
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_guard_condition_fini(&mut guard_cond);
    assert_eq!(ret, RCL_RET_OK as i32);

    let ret = rcl_wait_set_fini(&mut wait_set);
    assert_eq!(ret, RCL_RET_OK as i32);
}
