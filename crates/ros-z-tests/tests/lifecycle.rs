//! Lifecycle node and publisher unit / integration tests.
//!
//! These tests exercise:
//! - State machine transitions (configure, activate, deactivate, cleanup, shutdown)
//! - Callback returns (Success, Failure, Error)
//! - Lifecycle publisher gating (publish() drops when deactivated, delivers when active)
//! - `create_publisher` mid-lifecycle registration
//!
//! The tests do **not** require a running Zenoh router; they use `ZContextBuilder`
//! in peer mode (no connect endpoints) so they can run offline.

mod common;

use std::sync::{
    Arc,
    atomic::{AtomicU32, Ordering},
};

use ros_z::{
    Builder,
    context::ZContextBuilder,
    lifecycle::{CallbackReturn, LifecycleState, ZLifecycleNode},
    prelude::*,
};
use ros_z_msgs::std_msgs::String as RosString;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Create a standalone lifecycle node (no router needed for unit-level tests).
fn make_node(name: &str) -> ZLifecycleNode {
    ZContextBuilder::default()
        .disable_multicast_scouting()
        .build()
        .expect("context")
        .create_lifecycle_node(name)
        .build()
        .expect("lifecycle node")
}

// ---------------------------------------------------------------------------
// State machine transition tests
// ---------------------------------------------------------------------------

#[test]
fn test_initial_state_is_unconfigured() {
    let node = make_node("lc_initial");
    assert_eq!(node.get_current_state(), LifecycleState::Unconfigured);
}

#[test]
fn test_configure_transitions_to_inactive() {
    let mut node = make_node("lc_configure");
    let s = node.configure().expect("configure");
    assert_eq!(s, LifecycleState::Inactive);
    assert_eq!(node.get_current_state(), LifecycleState::Inactive);
}

#[test]
fn test_configure_activate_deactivate_cleanup() {
    let mut node = make_node("lc_full_cycle");

    assert_eq!(node.configure().unwrap(), LifecycleState::Inactive);
    assert_eq!(node.activate().unwrap(), LifecycleState::Active);
    assert_eq!(node.deactivate().unwrap(), LifecycleState::Inactive);
    assert_eq!(node.cleanup().unwrap(), LifecycleState::Unconfigured);
}

#[test]
fn test_shutdown_from_unconfigured() {
    let mut node = make_node("lc_shutdown_unconfigured");
    let s = node.shutdown().expect("shutdown");
    assert_eq!(s, LifecycleState::Finalized);
}

#[test]
fn test_shutdown_from_inactive() {
    let mut node = make_node("lc_shutdown_inactive");
    node.configure().unwrap();
    let s = node.shutdown().unwrap();
    assert_eq!(s, LifecycleState::Finalized);
}

#[test]
fn test_shutdown_from_active() {
    let mut node = make_node("lc_shutdown_active");
    node.configure().unwrap();
    node.activate().unwrap();
    let s = node.shutdown().unwrap();
    assert_eq!(s, LifecycleState::Finalized);
}

// ---------------------------------------------------------------------------
// Callback tests
// ---------------------------------------------------------------------------

#[test]
fn test_callbacks_invoked_on_transitions() {
    let mut node = make_node("lc_callbacks");
    let counter = Arc::new(AtomicU32::new(0));

    let c = counter.clone();
    node.on_configure = Box::new(move |_| {
        c.fetch_add(1, Ordering::Relaxed);
        CallbackReturn::Success
    });
    let c = counter.clone();
    node.on_activate = Box::new(move |_| {
        c.fetch_add(1, Ordering::Relaxed);
        CallbackReturn::Success
    });
    let c = counter.clone();
    node.on_deactivate = Box::new(move |_| {
        c.fetch_add(1, Ordering::Relaxed);
        CallbackReturn::Success
    });
    let c = counter.clone();
    node.on_cleanup = Box::new(move |_| {
        c.fetch_add(1, Ordering::Relaxed);
        CallbackReturn::Success
    });

    node.configure().unwrap();
    node.activate().unwrap();
    node.deactivate().unwrap();
    node.cleanup().unwrap();

    assert_eq!(counter.load(Ordering::Relaxed), 4);
}

#[test]
fn test_failure_callback_stays_in_current_state() {
    let mut node = make_node("lc_failure_cb");
    node.on_configure = Box::new(|_| CallbackReturn::Failure);

    // configure() returns the state after error processing (Unconfigured for Failure)
    let s = node.configure().unwrap();
    // Failure on configure → stays Unconfigured
    assert_eq!(s, LifecycleState::Unconfigured);
}

#[test]
fn test_error_callback_reaches_finalized_by_default() {
    let mut node = make_node("lc_error_cb");
    // Default on_error returns Failure (→ Finalized)
    node.on_configure = Box::new(|_| CallbackReturn::Error);

    let s = node.configure().unwrap();
    assert_eq!(s, LifecycleState::Finalized);
}

#[test]
fn test_error_callback_can_recover_to_unconfigured() {
    let mut node = make_node("lc_error_recover");
    node.on_configure = Box::new(|_| CallbackReturn::Error);
    node.on_error = Box::new(|_| CallbackReturn::Success); // Success → Unconfigured

    let s = node.configure().unwrap();
    assert_eq!(s, LifecycleState::Unconfigured);
}

// ---------------------------------------------------------------------------
// Publisher gating tests (mirrors test_lifecycle_publisher.cpp)
// ---------------------------------------------------------------------------

#[cfg(feature = "ros-msgs")]
#[test]
fn test_publisher_deactivated_by_default() {
    let node = make_node("lc_pub_default");
    let pub_ = node
        .create_publisher::<RosString>("chatter")
        .expect("create_publisher");
    assert!(!pub_.is_activated());
}

#[cfg(feature = "ros-msgs")]
#[test]
fn test_publisher_activated_after_node_activate() {
    let mut node = make_node("lc_pub_activate");
    let pub_ = node
        .create_publisher::<RosString>("chatter")
        .expect("create_publisher");

    node.configure().unwrap();
    assert!(!pub_.is_activated());

    node.activate().unwrap();
    assert!(pub_.is_activated());

    node.deactivate().unwrap();
    assert!(!pub_.is_activated());
}

#[cfg(feature = "ros-msgs")]
#[test]
fn test_publish_does_not_error_when_deactivated() {
    let node = make_node("lc_pub_drop");
    let pub_ = node
        .create_publisher::<RosString>("chatter")
        .expect("create_publisher");

    // Publisher is deactivated — publish() must not error, it just drops the message.
    let msg = RosString {
        data: "hello".to_string(),
    };
    pub_.publish(&msg)
        .expect("publish when deactivated should be Ok");
}

#[cfg(feature = "ros-msgs")]
#[test]
fn test_publish_succeeds_when_activated() {
    let mut node = make_node("lc_pub_active_publish");
    let pub_ = node
        .create_publisher::<RosString>("chatter")
        .expect("create_publisher");

    node.configure().unwrap();
    node.activate().unwrap();
    assert!(pub_.is_activated());

    let msg = RosString {
        data: "hello".to_string(),
    };
    pub_.publish(&msg).expect("publish when activated");
}

#[cfg(feature = "ros-msgs")]
#[test]
fn test_publisher_registered_during_active_is_immediately_activated() {
    let mut node = make_node("lc_pub_late_register");
    node.configure().unwrap();
    node.activate().unwrap();

    // Publisher created after node is already active must be immediately activated.
    let pub_ = node
        .create_publisher::<RosString>("chatter_late")
        .expect("create_publisher");
    assert!(pub_.is_activated());
}

#[cfg(feature = "ros-msgs")]
#[test]
fn test_multiple_publishers_bulk_activated() {
    let mut node = make_node("lc_pub_bulk");
    let p1 = node.create_publisher::<RosString>("topic1").unwrap();
    let p2 = node.create_publisher::<RosString>("topic2").unwrap();
    let p3 = node.create_publisher::<RosString>("topic3").unwrap();

    node.configure().unwrap();
    node.activate().unwrap();

    assert!(p1.is_activated());
    assert!(p2.is_activated());
    assert!(p3.is_activated());

    node.deactivate().unwrap();

    assert!(!p1.is_activated());
    assert!(!p2.is_activated());
    assert!(!p3.is_activated());
}

// ---------------------------------------------------------------------------
// topic_name helper
// ---------------------------------------------------------------------------

#[cfg(feature = "ros-msgs")]
#[test]
fn test_publisher_topic_name() {
    let node = make_node("lc_pub_topic_name");
    let pub_ = node.create_publisher::<RosString>("my_topic").unwrap();
    // After qualification the topic should contain "my_topic"
    assert!(pub_.topic_name().contains("my_topic"));
}
