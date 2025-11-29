#![allow(dead_code)]

use crate::init::rcl_get_default_allocator;
use crate::pubsub::{PublisherImpl, SubscriptionImpl};
use crate::ros::*;
use ros_z::event::{RmEventHandle, ZenohEventType};
use std::collections::HashMap;
use std::ffi::c_void;
use std::sync::{Mutex, OnceLock};

pub(crate) static EVENT_MAP: OnceLock<Mutex<HashMap<usize, Box<EventImpl>>>> = OnceLock::new();





// Use the event types from bindings



pub const ZENOH_EVENT_ID_MAX: usize = 11;

impl From<rcl_publisher_event_type_t> for rmw_event_type_t {
    fn from(event_type: rcl_publisher_event_type_t) -> Self {
        match event_type {
            rcl_publisher_event_type_t::RCL_PUBLISHER_OFFERED_DEADLINE_MISSED => rmw_event_type_t::RMW_EVENT_OFFERED_DEADLINE_MISSED,
            rcl_publisher_event_type_t::RCL_PUBLISHER_LIVELINESS_LOST => rmw_event_type_t::RMW_EVENT_LIVELINESS_LOST,
            rcl_publisher_event_type_t::RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS => rmw_event_type_t::RMW_EVENT_OFFERED_QOS_INCOMPATIBLE,
            rcl_publisher_event_type_t::RCL_PUBLISHER_INCOMPATIBLE_TYPE => rmw_event_type_t::RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE,
            rcl_publisher_event_type_t::RCL_PUBLISHER_MATCHED => rmw_event_type_t::RMW_EVENT_PUBLICATION_MATCHED,
        }
    }
}

impl From<rcl_subscription_event_type_t> for rmw_event_type_t {
    fn from(event_type: rcl_subscription_event_type_t) -> Self {
        match event_type {
            rcl_subscription_event_type_t::RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED => rmw_event_type_t::RMW_EVENT_REQUESTED_DEADLINE_MISSED,
            rcl_subscription_event_type_t::RCL_SUBSCRIPTION_LIVELINESS_CHANGED => rmw_event_type_t::RMW_EVENT_LIVELINESS_CHANGED,
            rcl_subscription_event_type_t::RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS => rmw_event_type_t::RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE,
            rcl_subscription_event_type_t::RCL_SUBSCRIPTION_MESSAGE_LOST => rmw_event_type_t::RMW_EVENT_MESSAGE_LOST,
            rcl_subscription_event_type_t::RCL_SUBSCRIPTION_INCOMPATIBLE_TYPE => rmw_event_type_t::RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE,
            rcl_subscription_event_type_t::RCL_SUBSCRIPTION_MATCHED => rmw_event_type_t::RMW_EVENT_SUBSCRIPTION_MATCHED,
        }
    }
}

impl From<rmw_event_type_t> for Option<ZenohEventType> {
    fn from(rmw_event: rmw_event_type_t) -> Self {
        match rmw_event {
            rmw_event_type_t::RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE => Some(ZenohEventType::RequestedQosIncompatible),
            rmw_event_type_t::RMW_EVENT_OFFERED_QOS_INCOMPATIBLE => Some(ZenohEventType::OfferedQosIncompatible),
            rmw_event_type_t::RMW_EVENT_MESSAGE_LOST => Some(ZenohEventType::MessageLost),
            rmw_event_type_t::RMW_EVENT_SUBSCRIPTION_MATCHED => Some(ZenohEventType::SubscriptionMatched),
            rmw_event_type_t::RMW_EVENT_PUBLICATION_MATCHED => Some(ZenohEventType::PublicationMatched),
            rmw_event_type_t::RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE => Some(ZenohEventType::SubscriptionIncompatibleType),
            rmw_event_type_t::RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE => Some(ZenohEventType::PublisherIncompatibleType),
            rmw_event_type_t::RMW_EVENT_OFFERED_DEADLINE_MISSED => Some(ZenohEventType::OfferedDeadlineMissed),
            rmw_event_type_t::RMW_EVENT_REQUESTED_DEADLINE_MISSED => Some(ZenohEventType::RequestedDeadlineMissed),
            rmw_event_type_t::RMW_EVENT_LIVELINESS_LOST => Some(ZenohEventType::LivelinessLost),
            rmw_event_type_t::RMW_EVENT_LIVELINESS_CHANGED => Some(ZenohEventType::LivelinessChanged),
            _ => None,
        }
    }
}

// Event Status Structures
#[derive(Clone, Default)]
pub struct ZenohEventStatus {
    pub total_count: i32,
    pub total_count_change: i32,
    pub current_count: i32,
    pub current_count_change: i32,
    pub data: String,
    pub changed: bool,
}

pub type EventCallback = Option<unsafe extern "C" fn(*const c_void, usize)>;

// EventsManager - manages event state for a single publisher/subscription
#[allow(dead_code)]
pub struct EventsManager {
    pub event_statuses: Vec<ZenohEventStatus>,
    pub event_callbacks: Vec<EventCallback>,
    pub event_data: Vec<Option<*const c_void>>,
    pub event_unread_count: Vec<usize>,
    pub wait_set_data: Vec<Option<*mut c_void>>,
    pub event_mutex: Mutex<()>,
    pub event_condition_mutex: Mutex<()>,
}

impl Default for EventsManager {
    fn default() -> Self {
        Self::new()
    }
}

impl EventsManager {
    pub fn new() -> Self {
        Self {
            event_statuses: vec![ZenohEventStatus::default(); ZENOH_EVENT_ID_MAX],
            event_callbacks: vec![None; ZENOH_EVENT_ID_MAX],
            event_data: vec![None; ZENOH_EVENT_ID_MAX],
            event_unread_count: vec![0; ZENOH_EVENT_ID_MAX],
            wait_set_data: vec![None; ZENOH_EVENT_ID_MAX],
            event_mutex: Mutex::new(()),
            event_condition_mutex: Mutex::new(()),
        }
    }

    pub fn event_set_callback(&mut self, event_id: usize, callback: EventCallback, user_data: *const c_void) {
        let _lock = self.event_mutex.lock().unwrap();
        self.event_callbacks[event_id] = callback;
        self.event_data[event_id] = Some(user_data);
        if let Some(cb) = callback {
            if self.event_unread_count[event_id] > 0 {
                unsafe { cb(user_data, self.event_unread_count[event_id]); }
                self.event_unread_count[event_id] = 0;
            }
        }
    }

    pub fn trigger_event_callback(&mut self, event_id: usize) {
        let _lock = self.event_mutex.lock().unwrap();
        if let Some(callback) = self.event_callbacks[event_id] {
            unsafe { callback(self.event_data[event_id].unwrap(), 1); }
        } else {
            self.event_unread_count[event_id] += 1;
        }
    }

    pub fn take_event_status(&mut self, event_id: usize) -> ZenohEventStatus {
        let _lock = self.event_mutex.lock().unwrap();
        let ret = self.event_statuses[event_id].clone();
        self.event_statuses[event_id].current_count_change = 0;
        self.event_statuses[event_id].total_count_change = 0;
        self.event_statuses[event_id].changed = false;
        ret
    }

    pub fn update_event_status(&mut self, event_id: usize, current_count_change: i32) {
        {
            let _lock = self.event_mutex.lock().unwrap();
            let status = &mut self.event_statuses[event_id];
            status.total_count += current_count_change.max(0);
            status.total_count_change += current_count_change.max(0);
            status.current_count += current_count_change;
            status.current_count_change += current_count_change;
            status.changed = true;
        }
        self.trigger_event_callback(event_id);
        self.notify_event(event_id);
    }

    pub fn queue_has_data_and_attach_condition_if_not(&mut self, event_id: usize, wait_set_data: *mut c_void) -> bool {
        let _lock = self.event_condition_mutex.lock().unwrap();
        if self.event_statuses[event_id].changed {
            true
        } else {
            self.wait_set_data[event_id] = Some(wait_set_data);
            false
        }
    }

    pub fn detach_condition_and_event_queue_is_empty(&mut self, event_id: usize) -> bool {
        let _lock = self.event_condition_mutex.lock().unwrap();
        self.wait_set_data[event_id] = None;
        !self.event_statuses[event_id].changed
    }

    pub fn notify_event(&mut self, _event_id: usize) {
        let _lock = self.event_condition_mutex.lock().unwrap();
        // In C++, it sets triggered and notifies, but since we can't, do nothing
        // TODO: Implement proper notification mechanism
    }
}

#[derive(Debug)]
pub struct EventImpl {
    pub rmw_handle: Box<RmEventHandle>,
}

// RCL Event API Functions
#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_event() -> rcl_event_t {
    unsafe { std::mem::zeroed() }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_publisher_event_init(
    event: *mut rcl_event_t,
    publisher: *const rcl_publisher_t,
    event_type: rcl_publisher_event_type_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_publisher_event_init");

    // Validate arguments
    if event.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if publisher.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check if publisher is valid using rcl_publisher_is_valid
    use crate::pubsub::rcl_publisher_is_valid;
    if !rcl_publisher_is_valid(publisher) {
        return RCL_RET_UNSUPPORTED as _;
    }
    let pub_impl = unsafe { &*((*publisher).impl_ as *const PublisherImpl) };

    // Check if event is already initialized
    if !event.is_null() && !unsafe { (*event).impl_ }.is_null() {
        return RCL_RET_ALREADY_INIT as _;
    }

    // Get default allocator
    let _allocator = rcl_get_default_allocator();

    // Convert RCL event type to RMW event type
    let rmw_event_type = rmw_event_type_t::from(event_type);

    // Convert to Zenoh event type
    let zenoh_event_type = match rmw_event_type {
        rmw_event_type_t::RMW_EVENT_PUBLICATION_MATCHED => ZenohEventType::PublicationMatched,
        rmw_event_type_t::RMW_EVENT_OFFERED_QOS_INCOMPATIBLE => ZenohEventType::OfferedQosIncompatible,
        rmw_event_type_t::RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE => ZenohEventType::PublisherIncompatibleType,
        rmw_event_type_t::RMW_EVENT_OFFERED_DEADLINE_MISSED => ZenohEventType::OfferedDeadlineMissed,
        rmw_event_type_t::RMW_EVENT_LIVELINESS_LOST => ZenohEventType::LivelinessLost,
        _ => return RCL_RET_INVALID_ARGUMENT as _,
    };

    // Create RMW event handle using ros-z EventsManager
    let rmw_handle = RmEventHandle::new(pub_impl.inner.events_mgr().clone(), zenoh_event_type);

    // Create the EventImpl
    let event_impl = Box::new(EventImpl {
        rmw_handle: Box::new(rmw_handle),
    });

    // Store in global map
    let event_ptr = event as usize;
    EVENT_MAP.get_or_init(|| Mutex::new(HashMap::new())).lock().unwrap().insert(event_ptr, event_impl);

    // For the C impl, we'll use null since the struct is opaque and we're using global map
    unsafe {
        (*event).impl_ = std::ptr::null_mut::<rcl_event_impl_s>();
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_subscription_event_init(
    event: *mut rcl_event_t,
    subscription: *const rcl_subscription_t,
    event_type: rcl_subscription_event_type_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_subscription_event_init");

    // Validate arguments
    if event.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if subscription.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check if subscription is valid using rcl_subscription_is_valid
    use crate::pubsub::rcl_subscription_is_valid;
    if !rcl_subscription_is_valid(subscription) {
        return RCL_RET_UNSUPPORTED as _;
    }

    // Check if event is already initialized
    if !event.is_null() && !unsafe { (*event).impl_ }.is_null() {
        return RCL_RET_ALREADY_INIT as _;
    }

    // Get default allocator
    let _allocator = rcl_get_default_allocator();

    // Convert RCL event type to RMW event type
    let rmw_event_type = rmw_event_type_t::from(event_type);

    // Convert to Zenoh event type
    let zenoh_event_type = match rmw_event_type {
        rmw_event_type_t::RMW_EVENT_SUBSCRIPTION_MATCHED => ZenohEventType::SubscriptionMatched,
        rmw_event_type_t::RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE => ZenohEventType::RequestedQosIncompatible,
        rmw_event_type_t::RMW_EVENT_MESSAGE_LOST => ZenohEventType::MessageLost,
        rmw_event_type_t::RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE => ZenohEventType::SubscriptionIncompatibleType,
        rmw_event_type_t::RMW_EVENT_REQUESTED_DEADLINE_MISSED => ZenohEventType::RequestedDeadlineMissed,
        rmw_event_type_t::RMW_EVENT_LIVELINESS_CHANGED => ZenohEventType::LivelinessChanged,
        _ => return RCL_RET_INVALID_ARGUMENT as _,
    };

    // Get subscription impl
    let sub_impl = unsafe { &*((*subscription).impl_ as *const SubscriptionImpl) };

    // Create RMW event handle using ros-z EventsManager
    let rmw_handle = RmEventHandle::new(sub_impl.inner.events_mgr().clone(), zenoh_event_type);

    // Create the EventImpl
    let event_impl = Box::new(EventImpl {
        rmw_handle: Box::new(rmw_handle),
    });

    // Store in global map
    let event_ptr = event as usize;
    EVENT_MAP.get_or_init(|| Mutex::new(HashMap::new())).lock().unwrap().insert(event_ptr, event_impl);

    // For the C impl, we'll use null since the struct is opaque and we're using global map
    unsafe {
        (*event).impl_ = std::ptr::null_mut::<rcl_event_impl_s>();
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_take_event(
    event: *const rcl_event_t,
    event_info: *mut c_void,
) -> rcl_ret_t {
    tracing::trace!("rcl_take_event");

    // Validate arguments
    if event.is_null() {
        return RCL_RET_EVENT_INVALID as _;
    }
    if event_info.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check if event is valid (in the map)
    let event_ptr = event as usize;
    let map = EVENT_MAP.get_or_init(|| Mutex::new(HashMap::new())).lock().unwrap();
    let event_impl = match map.get(&event_ptr) {
        Some(impl_) => impl_,
        None => return RCL_RET_EVENT_INVALID as _,
    };

    // Take event from the RMW handle
    let rmw_handle = &*event_impl.rmw_handle;
    let status = rmw_handle.take_event();

    // Check if there are changes
    if status.total_count_change > 0 || status.current_count_change != 0 {
        // Event available - in a real implementation, we'd copy the status to event_info
        return RCL_RET_OK as _;
    }

    RCL_RET_EVENT_TAKE_FAILED as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_event_set_callback(
    event: *const rcl_event_t,
    _callback: rcl_event_callback_t,
    _user_data: *const c_void,
) -> rcl_ret_t {
    tracing::trace!("rcl_event_set_callback");

    // Validate arguments
    if event.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check if event is valid
    if event.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    let event_ptr = event as usize;
    if !EVENT_MAP.get_or_init(|| Mutex::new(HashMap::new())).lock().unwrap().contains_key(&event_ptr) {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // TODO: Implement setting callback via RMW layer
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_event_fini(event: *mut rcl_event_t) -> rcl_ret_t {
    tracing::trace!("rcl_event_fini");

    // Validate arguments
    if event.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Drop the implementation if it exists
    if !event.is_null() {
        let event_ptr = event as usize;
        EVENT_MAP.get_or_init(|| Mutex::new(HashMap::new())).lock().unwrap().remove(&event_ptr);
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_event_get_rmw_handle(event: *const rcl_event_t) -> *const c_void {
    if event.is_null() {
        std::ptr::null()
    } else {
        let event_ptr = event as usize;
        match EVENT_MAP.get_or_init(|| Mutex::new(HashMap::new())).lock().unwrap().get(&event_ptr) {
            Some(impl_) => &*impl_.rmw_handle as *const _ as *const c_void,
            None => std::ptr::null(),
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_event_is_valid(event: *const rcl_event_t) -> bool {
    if event.is_null() {
        return false;
    }
    let event_ptr = event as usize;
    EVENT_MAP.get_or_init(|| Mutex::new(HashMap::new())).lock().unwrap().contains_key(&event_ptr)
}