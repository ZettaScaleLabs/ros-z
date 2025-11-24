// Ported from Open Source Robotics Foundation code (2015)
// https://github.com/ros2/rcl
// Licensed under the Apache License 2.0
// Copyright 2025 ZettaScale Technology

#![cfg(feature = "test-core")]

// TODO: Implement comprehensive time/clock tests
// These tests require:
// - rcl_set_ros_time_override()
// - rcl_is_enabled_ros_time_override()
// - rcl_enable_ros_time_override()
// - rcl_disable_ros_time_override()
// - Jump callbacks
// - Clock initialization tests

use rcl_z::{
    init::rcl_get_default_allocator,
    ros::RCL_RET_OK,
    timer::{rcl_clock_get_now, rcl_ros_clock_init},
};

/// Basic test that clock_get_now works
#[test]
fn test_clock_get_now_basic() {
    unsafe {
        let mut clock = rcl_z::ros::rcl_clock_t::default();
        let mut allocator = rcl_get_default_allocator();
        let ret = rcl_ros_clock_init(&mut clock, &mut allocator as *mut _);
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut now = 0i64;
        let ret = rcl_clock_get_now(&mut clock, &mut now);
        assert_eq!(ret, RCL_RET_OK as i32);
        assert!(now > 0);
    }
}

// TODO: Port additional time tests including:
// - ROS time override functionality
// - Time jump callbacks
// - Clock type validation
