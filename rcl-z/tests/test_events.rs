// Copyright 2025 ZettaScale Technology
// SPDX-License-Identifier: Apache-2.0
//
// Ported from ros2/rcl:
// Copyright 2019 Open Source Robotics Foundation, Inc.

#[cfg(feature = "test-msgs")]
mod test_msgs_support;

use std::ptr;

// Status structs for event testing
#[repr(C)]
#[derive(Debug, Default, Clone)]
pub struct rmw_offered_deadline_missed_status_t {
    pub total_count: i32,
    pub total_count_change: i32,
}

#[repr(C)]
#[derive(Debug, Default, Clone)]
pub struct rmw_requested_deadline_missed_status_t {
    pub total_count: i32,
    pub total_count_change: i32,
}

#[repr(C)]
#[derive(Debug, Default, Clone)]
pub struct rmw_liveliness_changed_status_t {
    pub alive_count: i32,
    pub not_alive_count: i32,
    pub alive_count_change: i32,
    pub not_alive_count_change: i32,
}

#[repr(C)]
#[derive(Debug, Default, Clone)]
pub struct rmw_liveliness_lost_status_t {
    pub total_count: i32,
    pub total_count_change: i32,
}

#[repr(C)]
#[derive(Debug, Default, Clone)]
pub struct rmw_requested_qos_incompatible_event_status_t {
    pub total_count: i32,
    pub total_count_change: i32,
    pub last_policy_kind: i32,
}

#[repr(C)]
#[derive(Debug, Default, Clone)]
pub struct rmw_offered_qos_incompatible_event_status_t {
    pub total_count: i32,
    pub total_count_change: i32,
    pub last_policy_kind: i32,
}

#[repr(C)]
#[derive(Debug, Default, Clone)]
pub struct rmw_message_lost_status_t {
    pub total_count: u32,
    pub total_count_change: u32,
}

#[repr(C)]
#[derive(Debug, Default, Clone)]
pub struct rmw_matched_status_t {
    pub total_count: i32,
    pub total_count_change: i32,
    pub current_count: i32,
    pub current_count_change: i32,
}

use rcl_z::{
    event::{
        rcl_event_get_rmw_handle, rcl_event_is_valid, rcl_publisher_event_init,
        rcl_subscription_event_init, rcl_take_event,
    },
    ros::*,
};

/// Test bad event init
#[test]
fn test_bad_event_ini() {
    unsafe {
        // Test with null pointers - should return INVALID_ARGUMENT
        let mut publisher_event = rcl_get_zero_initialized_event();
        let ret = rcl_publisher_event_init(
            &mut publisher_event,
            ptr::null(),
            rcl_publisher_event_type_t::RCL_PUBLISHER_OFFERED_DEADLINE_MISSED,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        let mut subscription_event = rcl_get_zero_initialized_event();
        let ret = rcl_subscription_event_init(
            &mut subscription_event,
            ptr::null(),
            rcl_subscription_event_type_t::RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED,
        );
        assert_eq!(ret, RCL_RET_INVALID_ARGUMENT as i32);

        // Test with valid pointers and valid event types - should return UNSUPPORTED
        let mut publisher_event3 = rcl_get_zero_initialized_event();
        let ret = rcl_publisher_event_init(
            &mut publisher_event3,
            &publisher_event as *const _ as *const rcl_publisher_t, // dummy
            rcl_publisher_event_type_t::RCL_PUBLISHER_OFFERED_DEADLINE_MISSED,
        );
        assert_eq!(ret, RCL_RET_UNSUPPORTED as i32);

        let mut subscription_event3 = rcl_get_zero_initialized_event();
        let ret = rcl_subscription_event_init(
            &mut subscription_event3,
            &subscription_event as *const _ as *const rcl_subscription_t, // dummy
            rcl_subscription_event_type_t::RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED,
        );
        assert_eq!(ret, RCL_RET_UNSUPPORTED as i32);
    }
}

/// Test pubsub events (will return UNSUPPORTED since unsupported)
#[test]
fn test_pubsub_no_deadline_missed() {
    unsafe {
        // Try to init events with dummy pointers - should return UNSUPPORTED
        let mut publisher_event = rcl_get_zero_initialized_event();
        let ret = rcl_publisher_event_init(
            &mut publisher_event,
            &publisher_event as *const _ as *const rcl_publisher_t, // dummy
            rcl_publisher_event_type_t::RCL_PUBLISHER_OFFERED_DEADLINE_MISSED,
        );
        assert_eq!(
            ret, RCL_RET_UNSUPPORTED as i32,
            "Expected UNSUPPORTED for events"
        );
    }
}

/// Test pubsub deadline missed (unsupported)
#[test]
fn test_pubsub_deadline_missed() {
    unsafe {
        // Try to init events - should return UNSUPPORTED
        let mut publisher_event = rcl_get_zero_initialized_event();
        let ret = rcl_publisher_event_init(
            &mut publisher_event,
            &publisher_event as *const _ as *const rcl_publisher_t,
            rcl_publisher_event_type_t::RCL_PUBLISHER_OFFERED_DEADLINE_MISSED,
        );
        assert_eq!(ret, RCL_RET_UNSUPPORTED as i32);

        let mut subscription_event = rcl_get_zero_initialized_event();
        let ret = rcl_subscription_event_init(
            &mut subscription_event,
            &subscription_event as *const _ as *const rcl_subscription_t,
            rcl_subscription_event_type_t::RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED,
        );
        assert_eq!(ret, RCL_RET_UNSUPPORTED as i32);
    }
}

/// Test pubsub liveliness kill pub (unsupported)
#[test]
fn test_pubsub_liveliness_kill_pub() {
    unsafe {
        let mut publisher_event = rcl_get_zero_initialized_event();
        let ret = rcl_publisher_event_init(
            &mut publisher_event,
            &publisher_event as *const _ as *const rcl_publisher_t,
            rcl_publisher_event_type_t::RCL_PUBLISHER_LIVELINESS_LOST,
        );
        assert_eq!(ret, RCL_RET_UNSUPPORTED as i32);

        let mut subscription_event = rcl_get_zero_initialized_event();
        let ret = rcl_subscription_event_init(
            &mut subscription_event,
            &subscription_event as *const _ as *const rcl_subscription_t,
            rcl_subscription_event_type_t::RCL_SUBSCRIPTION_LIVELINESS_CHANGED,
        );
        assert_eq!(ret, RCL_RET_UNSUPPORTED as i32);
    }
}

/// Test incompatible qos (unsupported)
#[test]
fn test_pubsub_incompatible_qos() {
    unsafe {
        let mut publisher_event = rcl_get_zero_initialized_event();
        let ret = rcl_publisher_event_init(
            &mut publisher_event,
            &publisher_event as *const _ as *const rcl_publisher_t,
            rcl_publisher_event_type_t::RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS,
        );
        assert_eq!(ret, RCL_RET_UNSUPPORTED as i32);

        let mut subscription_event = rcl_get_zero_initialized_event();
        let ret = rcl_subscription_event_init(
            &mut subscription_event,
            &subscription_event as *const _ as *const rcl_subscription_t,
            rcl_subscription_event_type_t::RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS,
        );
        assert_eq!(ret, RCL_RET_UNSUPPORTED as i32);
    }
}

/// Test event is valid (unsupported)
#[test]
fn test_event_is_valid() {
    unsafe {
        // Test with null
        assert!(!rcl_event_is_valid(ptr::null()));

        // Test with zero initialized
        let event = rcl_get_zero_initialized_event();
        assert!(!rcl_event_is_valid(&event));
    }
}

/// Test event is invalid
#[test]
fn test_event_is_invalid() {
    unsafe {
        // nullptr
        let mut deadline_status = rmw_offered_deadline_missed_status_t::default();
        assert_eq!(
            rcl_take_event(ptr::null(), &mut deadline_status as *mut _ as *mut _),
            RCL_RET_EVENT_INVALID as i32
        );
        assert_eq!(rcl_event_get_rmw_handle(ptr::null()), ptr::null_mut());

        // Zero initialized
        let event = rcl_get_zero_initialized_event();
        assert_eq!(
            rcl_take_event(&event, &mut deadline_status as *mut _ as *mut _),
            RCL_RET_EVENT_INVALID as i32
        );
        assert_eq!(rcl_event_get_rmw_handle(&event), ptr::null_mut());
    }
}

/// Test sub message lost event (unsupported)
#[test]
fn test_sub_message_lost_event() {
    unsafe {
        let mut subscription_event = rcl_get_zero_initialized_event();
        let ret = rcl_subscription_event_init(
            &mut subscription_event,
            &subscription_event as *const _ as *const rcl_subscription_t,
            rcl_subscription_event_type_t::RCL_SUBSCRIPTION_MESSAGE_LOST,
        );
        assert_eq!(ret, RCL_RET_UNSUPPORTED as i32);
    }
}

/// Test pub matched unmatched event (unsupported)
#[test]
fn test_pub_matched_unmatched_event() {
    unsafe {
        let mut publisher_event = rcl_get_zero_initialized_event();
        let ret = rcl_publisher_event_init(
            &mut publisher_event,
            &publisher_event as *const _ as *const rcl_publisher_t,
            rcl_publisher_event_type_t::RCL_PUBLISHER_MATCHED,
        );
        assert_eq!(ret, RCL_RET_UNSUPPORTED as i32);
    }
}

/// Test sub matched unmatched event (unsupported)
#[test]
fn test_sub_matched_unmatched_event() {
    unsafe {
        let mut subscription_event = rcl_get_zero_initialized_event();
        let ret = rcl_subscription_event_init(
            &mut subscription_event,
            &subscription_event as *const _ as *const rcl_subscription_t,
            rcl_subscription_event_type_t::RCL_SUBSCRIPTION_MATCHED,
        );
        assert_eq!(ret, RCL_RET_UNSUPPORTED as i32);
    }
}

/// Test pub previous matched event (unsupported)
#[test]
fn test_pub_previous_matched_event() {
    unsafe {
        let mut publisher_event = rcl_get_zero_initialized_event();
        let ret = rcl_publisher_event_init(
            &mut publisher_event,
            &publisher_event as *const _ as *const rcl_publisher_t,
            rcl_publisher_event_type_t::RCL_PUBLISHER_MATCHED,
        );
        assert_eq!(ret, RCL_RET_UNSUPPORTED as i32);
    }
}

/// Test sub previous matched event (unsupported)
#[test]
fn test_sub_previous_matched_event() {
    unsafe {
        let mut subscription_event = rcl_get_zero_initialized_event();
        let ret = rcl_subscription_event_init(
            &mut subscription_event,
            &subscription_event as *const _ as *const rcl_subscription_t,
            rcl_subscription_event_type_t::RCL_SUBSCRIPTION_MATCHED,
        );
        assert_eq!(ret, RCL_RET_UNSUPPORTED as i32);
    }
}
