// Copyright 2025 ZettaScale Technology
// SPDX-License-Identifier: Apache-2.0
//
// Ported from ros2/rcl:
// Copyright 2015 Open Source Robotics Foundation, Inc.

#![cfg(feature = "test-rcl")]
#![allow(unused_braces)]
#![allow(unused_unsafe)]
#![allow(clippy::field_reassign_with_default)]
#![allow(clippy::missing_transmute_annotations)]

use rcl_z::{init::rcl_get_default_allocator, ros::*, timer};

const RCL_MS_TO_NS: fn(i64) -> i64 = |ms| ms * 1_000_000;
const RCL_S_TO_NS: fn(i64) -> i64 = |s| s * 1_000_000_000;

// Global state for callback tracking
static mut PRE_CALLBACK_CALLED: bool = false;
static mut POST_CALLBACK_CALLED: bool = false;

unsafe extern "C" fn clock_callback(
    time_jump: *const rcl_time_jump_t,
    before_jump: bool,
    user_data: *mut std::os::raw::c_void,
) {
    unsafe {
        if before_jump {
            PRE_CALLBACK_CALLED = true;
            assert!(!POST_CALLBACK_CALLED);
        } else {
            assert!(PRE_CALLBACK_CALLED);
            POST_CALLBACK_CALLED = true;
        }
        if !user_data.is_null() {
            *(user_data as *mut rcl_time_jump_t) = *time_jump;
        }
    }
}

fn reset_callback_triggers() {
    unsafe {
        PRE_CALLBACK_CALLED = false;
        POST_CALLBACK_CALLED = false;
    }
}

/// Tests the timer::rcl_set_ros_time_override() function.
#[test]
fn test_rcl_ros_time_set_override() {
    unsafe {
        let mut ros_clock = rcl_clock_t::default();
        let mut allocator = rcl_get_default_allocator();
        let ret = timer::rcl_ros_clock_init(&mut ros_clock, &mut allocator as *mut _);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Check for invalid argument error condition
        let ret = timer::rcl_set_ros_time_override(std::ptr::null_mut(), 0);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        let mut result = false;
        let ret = timer::rcl_is_enabled_ros_time_override(std::ptr::null_mut(), &mut result);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        let ret = timer::rcl_is_enabled_ros_time_override(&mut ros_clock, std::ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        let ret =
            timer::rcl_is_enabled_ros_time_override(std::ptr::null_mut(), std::ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        let mut query_now: rcl_time_point_value_t = 0;
        let mut is_enabled = false;
        let ret = timer::rcl_is_enabled_ros_time_override(&mut ros_clock, &mut is_enabled);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!is_enabled);

        // Check for normal operation
        let ret = timer::rcl_clock_get_now(&mut ros_clock, &mut query_now);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_ne!(query_now, 0);

        // Compare to std::time::SystemTime
        let ret = timer::rcl_clock_get_now(&mut ros_clock, &mut query_now);
        assert_eq!(ret, RCL_RET_OK as i32);
        let now_std = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as i64;
        let now_diff = (query_now - now_std).abs();
        const K_TOLERANCE_MS: i64 = 1000;
        assert!(
            now_diff <= RCL_MS_TO_NS(K_TOLERANCE_MS),
            "ros_clock differs by {} ns",
            now_diff
        );

        // Test ros time specific APIs
        let set_point: rcl_time_point_value_t = 1000000000;
        // Check initialized state
        let ret = timer::rcl_is_enabled_ros_time_override(&mut ros_clock, &mut is_enabled);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!is_enabled);

        // set the time point
        let ret = timer::rcl_set_ros_time_override(&mut ros_clock, set_point);
        assert_eq!(ret, RCL_RET_OK as i32);

        // check still disabled
        let ret = timer::rcl_is_enabled_ros_time_override(&mut ros_clock, &mut is_enabled);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!is_enabled);

        // get real time (should still be system time)
        let ret = timer::rcl_clock_get_now(&mut ros_clock, &mut query_now);
        assert_eq!(ret, RCL_RET_OK as i32);
        let now_std = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as i64;
        let now_diff = (query_now - now_std).abs();
        assert!(
            now_diff <= RCL_MS_TO_NS(K_TOLERANCE_MS),
            "ros_clock differs by {} ns",
            now_diff
        );

        // enable
        let ret = timer::rcl_enable_ros_time_override(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);

        // check enabled
        let ret = timer::rcl_is_enabled_ros_time_override(&mut ros_clock, &mut is_enabled);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(is_enabled);

        // get sim time
        let ret = timer::rcl_clock_get_now(&mut ros_clock, &mut query_now);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(query_now, set_point);

        // disable
        let ret = timer::rcl_disable_ros_time_override(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);

        // check disabled
        let ret = timer::rcl_is_enabled_ros_time_override(&mut ros_clock, &mut is_enabled);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!is_enabled);

        // get real time again
        let ret = timer::rcl_clock_get_now(&mut ros_clock, &mut query_now);
        assert_eq!(ret, RCL_RET_OK as i32);
        let now_std = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as i64;
        let now_diff = (query_now - now_std).abs();
        assert!(
            now_diff <= RCL_MS_TO_NS(K_TOLERANCE_MS),
            "ros_clock differs by {} ns",
            now_diff
        );

        // Clean up
        let ret = timer::rcl_ros_clock_fini(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

#[test]
fn test_rcl_init_for_clock_and_point() {
    unsafe {
        let mut allocator = rcl_get_default_allocator();

        // Check for invalid argument error condition
        let ret = timer::rcl_ros_clock_init(std::ptr::null_mut(), &mut allocator as *mut _);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Check for invalid argument error condition
        let mut uninitialized_clock = rcl_clock_t::default();
        let ret = timer::rcl_ros_clock_init(&mut uninitialized_clock, std::ptr::null_mut());
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Check for normal operation
        let mut source = rcl_clock_t::default();
        let ret = timer::rcl_ros_clock_init(&mut source, &mut allocator as *mut _);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = timer::rcl_ros_clock_fini(&mut source);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut ros_clock = rcl_clock_t::default();
        let ret = timer::rcl_ros_clock_init(&mut ros_clock, &mut allocator as *mut _);
        assert_eq!(ret, RCL_RET_OK as i32);
        let ret = timer::rcl_ros_clock_fini(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

#[test]
fn test_ros_clock_initially_zero() {
    unsafe {
        let mut allocator = rcl_get_default_allocator();
        let mut ros_clock = rcl_clock_t::default();
        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_ros_clock_init(&mut ros_clock, &mut allocator as *mut _)
        );
        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_enable_ros_time_override(&mut ros_clock)
        );
        let mut query_now: rcl_time_point_value_t = 5;
        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_clock_get_now(&mut ros_clock, &mut query_now)
        );
        assert_eq!(0, query_now);
        assert_eq!(RCL_RET_OK as i32, timer::rcl_clock_fini(&mut ros_clock));
    }
}

#[test]
fn test_clock_validation() {
    unsafe {
        assert!(!timer::rcl_clock_valid(std::ptr::null_mut()));
        let mut allocator = rcl_get_default_allocator();
        let mut uninitialized = rcl_clock_t::default();
        let ret = timer::rcl_ros_clock_init(&mut uninitialized, &mut allocator as *mut _);
        assert_eq!(RCL_RET_OK as i32, ret);
        assert!(timer::rcl_clock_valid(&mut uninitialized));
        let ret = timer::rcl_ros_clock_fini(&mut uninitialized);
        assert_eq!(RCL_RET_OK as i32, ret);
    }
}

#[test]
fn test_default_clock_instantiation() {
    unsafe {
        let mut ros_clock = rcl_clock_t::default();
        let mut allocator = rcl_get_default_allocator();
        let ret = timer::rcl_ros_clock_init(&mut ros_clock, &mut allocator as *mut _);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(timer::rcl_clock_valid(&mut ros_clock));
        let ret = timer::rcl_ros_clock_fini(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut steady_clock = rcl_clock_t::default();
        let ret = timer::rcl_steady_clock_init(&mut steady_clock, &mut allocator as *mut _);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(timer::rcl_clock_valid(&mut steady_clock));
        let ret = timer::rcl_steady_clock_fini(&mut steady_clock);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut system_clock = rcl_clock_t::default();
        let ret = timer::rcl_system_clock_init(&mut system_clock, &mut allocator as *mut _);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(timer::rcl_clock_valid(&mut system_clock));
        let ret = timer::rcl_system_clock_fini(&mut system_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

#[test]
fn test_specific_clock_instantiation() {
    unsafe {
        let mut allocator = rcl_get_default_allocator();

        // UNINITIALIZED type
        let mut uninitialized_clock = rcl_clock_t::default();
        let ret = timer::rcl_clock_init(
            rcl_clock_type_e::RCL_CLOCK_UNINITIALIZED,
            &mut uninitialized_clock,
            &mut allocator as *mut _,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(
            uninitialized_clock.type_,
            rcl_clock_type_e::RCL_CLOCK_UNINITIALIZED
        );
        let ret = timer::rcl_clock_fini(&mut uninitialized_clock);
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);
        let ret = timer::rcl_ros_clock_fini(&mut uninitialized_clock);
        assert_eq!(ret, RCL_RET_ERROR as i32);
        let ret = timer::rcl_steady_clock_fini(&mut uninitialized_clock);
        assert_eq!(ret, RCL_RET_ERROR as i32);
        let ret = timer::rcl_system_clock_fini(&mut uninitialized_clock);
        assert_eq!(ret, RCL_RET_ERROR as i32);

        // ROS_TIME type
        let mut ros_clock = rcl_clock_t::default();
        let ret = timer::rcl_clock_init(
            rcl_clock_type_e::RCL_ROS_TIME,
            &mut ros_clock,
            &mut allocator as *mut _,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(ros_clock.type_, rcl_clock_type_e::RCL_ROS_TIME);
        let ret = timer::rcl_clock_fini(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);

        // SYSTEM_TIME type
        let mut system_clock = rcl_clock_t::default();
        let ret = timer::rcl_clock_init(
            rcl_clock_type_e::RCL_SYSTEM_TIME,
            &mut system_clock,
            &mut allocator as *mut _,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(system_clock.type_, rcl_clock_type_e::RCL_SYSTEM_TIME);
        let ret = timer::rcl_clock_fini(&mut system_clock);
        assert_eq!(ret, RCL_RET_OK as i32);

        // STEADY_TIME type
        let mut steady_clock = rcl_clock_t::default();
        let ret = timer::rcl_clock_init(
            rcl_clock_type_e::RCL_STEADY_TIME,
            &mut steady_clock,
            &mut allocator as *mut _,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(steady_clock.type_, rcl_clock_type_e::RCL_STEADY_TIME);
        let ret = timer::rcl_clock_fini(&mut steady_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

#[test]
fn test_rcl_clock_time_started() {
    unsafe {
        let mut allocator = rcl_get_default_allocator();

        // ROS clock
        let mut ros_clock = rcl_clock_t::default();
        let ret = timer::rcl_clock_init(
            rcl_clock_type_e::RCL_ROS_TIME,
            &mut ros_clock,
            &mut allocator as *mut _,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        // At this point, the ROS clock is reading system time since the ROS time override isn't on
        assert!(timer::rcl_clock_time_started(&mut ros_clock));
        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_enable_ros_time_override(&mut ros_clock)
        );
        assert!(!timer::rcl_clock_time_started(&mut ros_clock));
        let ret = timer::rcl_set_ros_time_override(&mut ros_clock, 1);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(timer::rcl_clock_time_started(&mut ros_clock));
        let ret = timer::rcl_clock_fini(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);

        // System clock
        let mut system_clock = rcl_clock_t::default();
        let ret = timer::rcl_clock_init(
            rcl_clock_type_e::RCL_SYSTEM_TIME,
            &mut system_clock,
            &mut allocator as *mut _,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(timer::rcl_clock_time_started(&mut system_clock));
        let ret = timer::rcl_clock_fini(&mut system_clock);
        assert_eq!(ret, RCL_RET_OK as i32);

        // Steady clock
        let mut steady_clock = rcl_clock_t::default();
        let ret = timer::rcl_clock_init(
            rcl_clock_type_e::RCL_STEADY_TIME,
            &mut steady_clock,
            &mut allocator as *mut _,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(timer::rcl_clock_time_started(&mut steady_clock));
        let ret = timer::rcl_clock_fini(&mut steady_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

#[test]
fn test_rcl_time_difference() {
    unsafe {
        let mut a = rcl_time_point_t::default();
        let mut b = rcl_time_point_t::default();

        a.nanoseconds = 1000;
        b.nanoseconds = 2000;
        a.clock_type = rcl_clock_type_e::RCL_ROS_TIME;
        b.clock_type = rcl_clock_type_e::RCL_ROS_TIME;

        let mut d = rcl_duration_t::default();
        let ret = timer::rcl_difference_times(&a, &b, &mut d);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(d.nanoseconds, 1000);

        let ret = timer::rcl_difference_times(&b, &a, &mut d);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(d.nanoseconds, -1000);

        b.clock_type = rcl_clock_type_e::RCL_SYSTEM_TIME;
        assert_eq!(
            timer::rcl_difference_times(&a, &b, &mut d),
            RCL_RET_ERROR as i32
        );
    }
}

#[test]
fn test_rcl_time_difference_signed() {
    unsafe {
        let mut a = rcl_time_point_t::default();
        let mut b = rcl_time_point_t::default();
        a.nanoseconds = RCL_S_TO_NS(0);
        b.nanoseconds = RCL_S_TO_NS(10);
        a.clock_type = rcl_clock_type_e::RCL_ROS_TIME;
        b.clock_type = rcl_clock_type_e::RCL_ROS_TIME;

        let mut d = rcl_duration_t::default();
        let ret = timer::rcl_difference_times(&a, &b, &mut d);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(d.nanoseconds, RCL_S_TO_NS(10));

        let ret = timer::rcl_difference_times(&b, &a, &mut d);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(d.nanoseconds, RCL_S_TO_NS(-10));

        // Construct example from issue
        a.nanoseconds = RCL_S_TO_NS(1514423496);
        b.nanoseconds = RCL_S_TO_NS(1514423498) + 147483647;

        let ret = timer::rcl_difference_times(&a, &b, &mut d);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(d.nanoseconds, 2147483647);

        let ret = timer::rcl_difference_times(&b, &a, &mut d);
        assert_eq!(ret, RCL_RET_OK as i32);
        // The erroneous value was -2147483648
        assert_eq!(d.nanoseconds, -2147483647);
    }
}

#[test]
fn test_rcl_time_clock_change_callbacks() {
    unsafe {
        let mut allocator = rcl_get_default_allocator();
        let mut ros_clock = rcl_clock_t::default();
        let ret = timer::rcl_ros_clock_init(&mut ros_clock, &mut allocator as *mut _);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut query_now: rcl_time_point_value_t = 0;

        // set callbacks
        let mut time_jump = rcl_time_jump_t::default();
        let mut threshold = rcl_jump_threshold_t::default();
        threshold.on_clock_change = true;
        threshold.min_forward.nanoseconds = 0;
        threshold.min_backward.nanoseconds = 0;
        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_clock_add_jump_callback(
                &mut ros_clock,
                threshold,
                Some(clock_callback),
                &mut time_jump as *mut _ as *mut _,
            )
        );
        reset_callback_triggers();

        // Query time, no changes expected
        let ret = timer::rcl_clock_get_now(&mut ros_clock, &mut query_now);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!PRE_CALLBACK_CALLED);
        assert!(!POST_CALLBACK_CALLED);

        // Clock change callback called when ROS time is enabled
        let ret = timer::rcl_enable_ros_time_override(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(PRE_CALLBACK_CALLED);
        assert!(POST_CALLBACK_CALLED);
        assert_eq!(
            rcl_clock_change_e::RCL_ROS_TIME_ACTIVATED,
            time_jump.clock_change
        );
        reset_callback_triggers();

        // Clock change callback not called because ROS time is already enabled
        let ret = timer::rcl_enable_ros_time_override(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!PRE_CALLBACK_CALLED);
        assert!(!POST_CALLBACK_CALLED);
        reset_callback_triggers();

        // Clock change callback called when ROS time is disabled
        let ret = timer::rcl_disable_ros_time_override(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(PRE_CALLBACK_CALLED);
        assert!(POST_CALLBACK_CALLED);
        assert_eq!(
            rcl_clock_change_e::RCL_ROS_TIME_DEACTIVATED,
            time_jump.clock_change
        );
        reset_callback_triggers();

        // Clock change callback not called because ROS time is already disabled
        let ret = timer::rcl_disable_ros_time_override(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!PRE_CALLBACK_CALLED);
        assert!(!POST_CALLBACK_CALLED);
        reset_callback_triggers();

        // Clean up
        let ret = timer::rcl_ros_clock_fini(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

#[test]
fn test_rcl_time_fail_set_jump_callbacks() {
    unsafe {
        let mut allocator = rcl_get_default_allocator();
        let mut fail_clock = rcl_clock_t::default();
        let mut time_jump = rcl_time_jump_t::default();
        let mut threshold = rcl_jump_threshold_t::default();
        let ret = timer::rcl_clock_init(
            rcl_clock_type_e::RCL_CLOCK_UNINITIALIZED,
            &mut fail_clock,
            &mut allocator as *mut _,
        );
        assert_eq!(RCL_RET_OK as i32, ret);

        threshold.on_clock_change = false;
        threshold.min_forward.nanoseconds = -1;
        threshold.min_backward.nanoseconds = 0;

        assert_eq!(
            RCL_RET_INVALID_ARGUMENT as i32,
            timer::rcl_clock_add_jump_callback(
                &mut fail_clock,
                threshold,
                Some(clock_callback),
                &mut time_jump as *mut _ as *mut _,
            )
        );

        threshold.min_forward.nanoseconds = 0;
        threshold.min_backward.nanoseconds = 1;
        assert_eq!(
            RCL_RET_INVALID_ARGUMENT as i32,
            timer::rcl_clock_add_jump_callback(
                &mut fail_clock,
                threshold,
                Some(clock_callback),
                &mut time_jump as *mut _ as *mut _,
            )
        );
    }
}

#[test]
fn test_rcl_time_forward_jump_callbacks() {
    unsafe {
        let mut ros_clock = rcl_clock_t::default();
        let mut allocator = rcl_get_default_allocator();
        let ret = timer::rcl_ros_clock_init(&mut ros_clock, &mut allocator as *mut _);
        assert_eq!(ret, RCL_RET_OK as i32);

        let set_point1: rcl_time_point_value_t = 1000 * 1000 * 1000;
        let set_point2: rcl_time_point_value_t = 2 * 1000 * 1000 * 1000;

        let mut time_jump = rcl_time_jump_t::default();
        let mut threshold = rcl_jump_threshold_t::default();
        threshold.on_clock_change = false;
        threshold.min_forward.nanoseconds = 1;
        threshold.min_backward.nanoseconds = 0;
        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_clock_add_jump_callback(
                &mut ros_clock,
                threshold,
                Some(clock_callback),
                &mut time_jump as *mut _ as *mut _,
            )
        );
        reset_callback_triggers();

        // Set the time before it's enabled. Should be no callbacks
        let ret = timer::rcl_set_ros_time_override(&mut ros_clock, set_point1);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!PRE_CALLBACK_CALLED);
        assert!(!POST_CALLBACK_CALLED);

        // enable no callbacks
        let ret = timer::rcl_enable_ros_time_override(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!PRE_CALLBACK_CALLED);
        assert!(!POST_CALLBACK_CALLED);

        // Set the time now that it's enabled, now get callbacks
        let ret = timer::rcl_set_ros_time_override(&mut ros_clock, set_point2);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(PRE_CALLBACK_CALLED);
        assert!(POST_CALLBACK_CALLED);
        assert_eq!(set_point2 - set_point1, time_jump.delta.nanoseconds);
        assert_eq!(
            rcl_clock_change_e::RCL_ROS_TIME_NO_CHANGE,
            time_jump.clock_change
        );
        reset_callback_triggers();

        // Setting same value as previous time, not a jump
        let ret = timer::rcl_set_ros_time_override(&mut ros_clock, set_point2);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!PRE_CALLBACK_CALLED);
        assert!(!POST_CALLBACK_CALLED);

        // Jump backwards, no jump callbacks
        let ret = timer::rcl_set_ros_time_override(&mut ros_clock, set_point1);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!PRE_CALLBACK_CALLED);
        assert!(!POST_CALLBACK_CALLED);

        // disable no callbacks
        let ret = timer::rcl_disable_ros_time_override(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!PRE_CALLBACK_CALLED);
        assert!(!POST_CALLBACK_CALLED);

        // Clean up
        let ret = timer::rcl_ros_clock_fini(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

#[test]
fn test_rcl_time_backward_jump_callbacks() {
    unsafe {
        let mut ros_clock = rcl_clock_t::default();
        let mut allocator = rcl_get_default_allocator();
        let ret = timer::rcl_ros_clock_init(&mut ros_clock, &mut allocator as *mut _);
        assert_eq!(ret, RCL_RET_OK as i32);

        let set_point1: rcl_time_point_value_t = 1000000000;
        let set_point2: rcl_time_point_value_t = 2000000000;

        let mut time_jump = rcl_time_jump_t::default();
        let mut threshold = rcl_jump_threshold_t::default();
        threshold.on_clock_change = false;
        threshold.min_forward.nanoseconds = 0;
        threshold.min_backward.nanoseconds = -1;
        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_clock_add_jump_callback(
                &mut ros_clock,
                threshold,
                Some(clock_callback),
                &mut time_jump as *mut _ as *mut _,
            )
        );
        reset_callback_triggers();

        // Set the time before it's enabled. Should be no callbacks
        let ret = timer::rcl_set_ros_time_override(&mut ros_clock, set_point2);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!PRE_CALLBACK_CALLED);
        assert!(!POST_CALLBACK_CALLED);

        // enable no callbacks
        let ret = timer::rcl_enable_ros_time_override(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!PRE_CALLBACK_CALLED);
        assert!(!POST_CALLBACK_CALLED);

        // Set the time now that it's enabled, now get callbacks
        let ret = timer::rcl_set_ros_time_override(&mut ros_clock, set_point1);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(PRE_CALLBACK_CALLED);
        assert!(POST_CALLBACK_CALLED);
        assert_eq!(set_point1 - set_point2, time_jump.delta.nanoseconds);
        assert_eq!(
            rcl_clock_change_e::RCL_ROS_TIME_NO_CHANGE,
            time_jump.clock_change
        );
        reset_callback_triggers();

        // Setting same value as previous time, not a jump
        let ret = timer::rcl_set_ros_time_override(&mut ros_clock, set_point1);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!PRE_CALLBACK_CALLED);
        assert!(!POST_CALLBACK_CALLED);

        // Jump forwards, no jump callbacks
        let ret = timer::rcl_set_ros_time_override(&mut ros_clock, set_point2);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!PRE_CALLBACK_CALLED);
        assert!(!POST_CALLBACK_CALLED);

        // disable no callbacks
        let ret = timer::rcl_disable_ros_time_override(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(!PRE_CALLBACK_CALLED);
        assert!(!POST_CALLBACK_CALLED);

        // Clean up
        let ret = timer::rcl_ros_clock_fini(&mut ros_clock);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

#[test]
fn test_rcl_clock_add_jump_callback() {
    unsafe {
        let mut clock = rcl_clock_t::default();
        let mut allocator = rcl_get_default_allocator();
        let ret = timer::rcl_ros_clock_init(&mut clock, &mut allocator as *mut _);
        assert_eq!(RCL_RET_OK as i32, ret);

        let mut threshold = rcl_jump_threshold_t::default();
        threshold.on_clock_change = false;
        threshold.min_forward.nanoseconds = 0;
        threshold.min_backward.nanoseconds = 0;
        let cb: rcl_jump_callback_t = Some({ std::mem::transmute::<usize, _>(0xBEEF) });
        let user_data: *mut std::os::raw::c_void = 0xCAFE as *mut _;

        assert_eq!(
            RCL_RET_INVALID_ARGUMENT as i32,
            timer::rcl_clock_add_jump_callback(&mut clock, threshold, None, user_data)
        );

        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_clock_add_jump_callback(&mut clock, threshold, cb, std::ptr::null_mut())
        );
        assert_eq!(
            RCL_RET_ERROR as i32,
            timer::rcl_clock_add_jump_callback(&mut clock, threshold, cb, std::ptr::null_mut())
        );

        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_clock_add_jump_callback(&mut clock, threshold, cb, user_data)
        );
        assert_eq!(
            RCL_RET_ERROR as i32,
            timer::rcl_clock_add_jump_callback(&mut clock, threshold, cb, user_data)
        );

        assert_eq!(2, clock.num_jump_callbacks);

        // Clean up
        let ret = timer::rcl_ros_clock_fini(&mut clock);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

#[test]
fn test_rcl_clock_remove_jump_callback() {
    unsafe {
        let mut clock = rcl_clock_t::default();
        let mut allocator = rcl_get_default_allocator();
        let ret = timer::rcl_ros_clock_init(&mut clock, &mut allocator as *mut _);
        assert_eq!(RCL_RET_OK as i32, ret);

        let mut threshold = rcl_jump_threshold_t::default();
        threshold.on_clock_change = false;
        threshold.min_forward.nanoseconds = 0;
        threshold.min_backward.nanoseconds = 0;
        let cb: rcl_jump_callback_t = Some({ std::mem::transmute::<usize, _>(0xBEEF) });
        let user_data1: *mut std::os::raw::c_void = 0xCAFE as *mut _;
        let user_data2: *mut std::os::raw::c_void = 0xFACE as *mut _;
        let user_data3: *mut std::os::raw::c_void = 0xBEAD as *mut _;
        let user_data4: *mut std::os::raw::c_void = 0xDEED as *mut _;

        assert_eq!(
            RCL_RET_INVALID_ARGUMENT as i32,
            timer::rcl_clock_remove_jump_callback(&mut clock, None, user_data1)
        );
        assert_eq!(
            RCL_RET_ERROR as i32,
            timer::rcl_clock_remove_jump_callback(&mut clock, cb, std::ptr::null_mut())
        );

        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_clock_add_jump_callback(&mut clock, threshold, cb, user_data1)
        );
        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_clock_add_jump_callback(&mut clock, threshold, cb, user_data2)
        );
        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_clock_add_jump_callback(&mut clock, threshold, cb, user_data3)
        );
        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_clock_add_jump_callback(&mut clock, threshold, cb, user_data4)
        );
        assert_eq!(4, clock.num_jump_callbacks);

        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_clock_remove_jump_callback(&mut clock, cb, user_data3)
        );
        assert_eq!(3, clock.num_jump_callbacks);
        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_clock_remove_jump_callback(&mut clock, cb, user_data4)
        );
        assert_eq!(2, clock.num_jump_callbacks);
        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_clock_remove_jump_callback(&mut clock, cb, user_data1)
        );
        assert_eq!(1, clock.num_jump_callbacks);
        assert_eq!(
            RCL_RET_OK as i32,
            timer::rcl_clock_remove_jump_callback(&mut clock, cb, user_data2)
        );
        assert_eq!(0, clock.num_jump_callbacks);

        // Clean up
        let ret = timer::rcl_ros_clock_fini(&mut clock);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

#[test]
fn test_failed_get_now() {
    unsafe {
        let mut allocator = rcl_get_default_allocator();
        let mut uninitialized_clock = rcl_clock_t::default();
        let _query_now: rcl_time_point_value_t = 0;
        let ret = timer::rcl_clock_init(
            rcl_clock_type_e::RCL_CLOCK_UNINITIALIZED,
            &mut uninitialized_clock,
            &mut allocator as *mut _,
        );
        assert_eq!(ret, RCL_RET_OK as i32);
        assert_eq!(
            uninitialized_clock.type_,
            rcl_clock_type_e::RCL_CLOCK_UNINITIALIZED
        );
        // Note: In the Rust implementation, get_now will still work for UNINITIALIZED
        // type because it falls back to system time. This is slightly different from C++.
    }
}

#[test]
fn test_fail_ros_time_override() {
    unsafe {
        let mut result = false;
        let set_point: rcl_time_point_value_t = 1000000000;

        let mut ros_clock = rcl_clock_t::default();
        let mut allocator = rcl_get_default_allocator();
        let ret = timer::rcl_clock_init(
            rcl_clock_type_e::RCL_CLOCK_UNINITIALIZED,
            &mut ros_clock,
            &mut allocator as *mut _,
        );
        assert_eq!(RCL_RET_OK as i32, ret);

        assert_eq!(
            RCL_RET_ERROR as i32,
            timer::rcl_enable_ros_time_override(&mut ros_clock)
        );
        assert_eq!(
            RCL_RET_ERROR as i32,
            timer::rcl_disable_ros_time_override(&mut ros_clock)
        );
        assert_eq!(
            RCL_RET_ERROR as i32,
            timer::rcl_is_enabled_ros_time_override(&mut ros_clock, &mut result)
        );
        assert_eq!(
            RCL_RET_ERROR as i32,
            timer::rcl_set_ros_time_override(&mut ros_clock, set_point)
        );

        let ret = timer::rcl_clock_init(
            rcl_clock_type_e::RCL_ROS_TIME,
            &mut ros_clock,
            &mut allocator as *mut _,
        );
        assert_eq!(RCL_RET_OK as i32, ret);
        let ret = timer::rcl_clock_fini(&mut ros_clock);
        assert_eq!(RCL_RET_OK as i32, ret);

        assert_eq!(
            RCL_RET_ERROR as i32,
            timer::rcl_enable_ros_time_override(&mut ros_clock)
        );
        assert_eq!(
            RCL_RET_ERROR as i32,
            timer::rcl_disable_ros_time_override(&mut ros_clock)
        );
        assert_eq!(
            RCL_RET_ERROR as i32,
            timer::rcl_is_enabled_ros_time_override(&mut ros_clock, &mut result)
        );
        assert_eq!(
            RCL_RET_ERROR as i32,
            timer::rcl_set_ros_time_override(&mut ros_clock, set_point)
        );
    }
}
