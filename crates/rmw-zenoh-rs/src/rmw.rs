#![allow(clippy::enum_variant_names)]
#![allow(clippy::missing_safety_doc)]
#![allow(clippy::explicit_auto_deref)]
#![allow(clippy::collapsible_if)]
#![allow(clippy::cast_slice_from_raw_parts)]
#![allow(clippy::not_unsafe_ptr_arg_deref)]
#![allow(clippy::useless_transmute)]
#![allow(clippy::missing_transmute_annotations)]
#![allow(unpredictable_function_pointer_comparisons)]

use crate::c_void;
use crate::ros::{
    rmw_event_callback_t, rmw_event_type_t, rmw_feature_t, rmw_gid_t,
    rmw_topic_endpoint_info_array_t,
};

use crate::type_support::MessageTypeSupport;
use ros_z::{
    Builder,
    event::{RmEventHandle, ZenohEventType},
};

use crate::{
    pubsub::{PublisherImpl, SubscriptionImpl},
    ros::*,
    service::{ClientImpl, ServiceImpl},
    traits::*,
};

// Helper function to normalize ROS namespace
// In ROS 2, both "" and "/" represent the root namespace and should be treated as equivalent
// We normalize to "" for consistent HashMap lookups
fn normalize_namespace(ns: &str) -> String {
    if ns == "/" {
        String::new()
    } else {
        ns.to_string()
    }
}

// Helper function to convert rmw_event_type_t to ZenohEventType
// Only supports events that rmw_zenoh_cpp supports - deadline and liveliness events are NOT supported
fn rmw_event_type_to_zenoh_event(rmw_event: rmw_event_type_t) -> Option<ZenohEventType> {
    match rmw_event {
        0 => None,                                               // RMW_EVENT_INVALID
        1 => None, // RMW_EVENT_LIVELINESS_CHANGED - not supported (matches rmw_zenoh_cpp)
        2 => None, // RMW_EVENT_REQUESTED_DEADLINE_MISSED - not supported (matches rmw_zenoh_cpp)
        3 => Some(ZenohEventType::RequestedQosIncompatible), // RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE
        4 => Some(ZenohEventType::MessageLost), // RMW_EVENT_MESSAGE_LOST
        5 => Some(ZenohEventType::SubscriptionIncompatibleType), // RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE
        6 => Some(ZenohEventType::SubscriptionMatched),          // RMW_EVENT_SUBSCRIPTION_MATCHED
        7 => None, // RMW_EVENT_LIVELINESS_LOST - not supported (matches rmw_zenoh_cpp)
        8 => None, // RMW_EVENT_OFFERED_DEADLINE_MISSED - not supported (matches rmw_zenoh_cpp)
        9 => Some(ZenohEventType::OfferedQosIncompatible), // RMW_EVENT_OFFERED_QOS_INCOMPATIBLE
        10 => Some(ZenohEventType::PublisherIncompatibleType), // RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE
        11 => Some(ZenohEventType::PublicationMatched),        // RMW_EVENT_PUBLICATION_MATCHED
        _ => None,
    }
}

// Implement the actual RMW functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_implementation_identifier() -> *const std::os::raw::c_char {
    crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const std::os::raw::c_char
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_serialization_format() -> *const std::os::raw::c_char {
    crate::RMW_ZENOH_SERIALIZATION_FORMAT.as_ptr() as *const std::os::raw::c_char
}

// Publishers
#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_publisher(
    node: *const rmw_node_t,
    type_support: *const rosidl_message_type_support_t,
    topic_name: *const std::os::raw::c_char,
    qos_profile: *const rmw_qos_profile_t,
    publisher_options: *const rmw_publisher_options_t,
) -> *mut rmw_publisher_t {
    if node.is_null()
        || type_support.is_null()
        || topic_name.is_null()
        || qos_profile.is_null()
        || publisher_options.is_null()
    {
        return std::ptr::null_mut();
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(e) => {
            tracing::trace!("rmw_create_publisher: Failed to borrow node data: {:?}", e);
            let msg = std::ffi::CString::new("Failed to get node implementation").unwrap();
            unsafe {
                crate::ros::rcutils_set_error_state(msg.as_ptr(), cfile!(), line!() as usize)
            };
            return std::ptr::null_mut();
        }
    };

    let topic_str = unsafe { std::ffi::CStr::from_ptr(topic_name) }
        .to_str()
        .unwrap_or("");
    let ts = match unsafe { crate::type_support::MessageTypeSupport::new(type_support) } {
        Ok(ts) => ts,
        Err(e) => {
            tracing::trace!(
                "rmw_create_publisher: Failed to create type support: {:?}",
                e
            );
            let msg = std::ffi::CString::new(format!("Failed to create type support: {}", e))
                .unwrap_or_else(|_| {
                    std::ffi::CString::new("Failed to create type support").unwrap()
                });
            unsafe {
                crate::ros::rcutils_set_error_state(msg.as_ptr(), cfile!(), line!() as usize)
            };
            return std::ptr::null_mut();
        }
    };

    let zpub_builder = node_impl
        .inner
        .create_pub::<crate::msg::RosMessage>(topic_str)
        .with_serdes::<crate::msg::RosSerdes>();
    let qos = crate::qos::rmw_qos_to_ros_z_qos(unsafe { &*qos_profile });
    let zpub_builder = zpub_builder
        .with_qos(qos)
        .with_type_info(ts.get_type_info()); // Set type_info BEFORE build() so liveliness token has correct type
    let zpub = match zpub_builder.build() {
        Ok(zpub) => zpub,
        Err(e) => {
            tracing::trace!("rmw_create_publisher: Failed to build ZPub: {:?}", e);
            let msg = std::ffi::CString::new(format!("Failed to build publisher: {}", e))
                .unwrap_or_else(|_| std::ffi::CString::new("Failed to build publisher").unwrap());
            unsafe {
                crate::ros::rcutils_set_error_state(msg.as_ptr(), cfile!(), line!() as usize)
            };
            return std::ptr::null_mut();
        }
    };

    let qualified_topic = zpub.entity.topic.clone();
    let entity = zpub.entity.clone();
    let entity_gid = ros_z::entity::endpoint_gid(&zpub.entity);

    // Get context to access the shared notifier
    let context = unsafe { (*node).context };
    let notifier = match context.borrow_impl() {
        Ok(impl_) => impl_.share_notifier(),
        Err(_) => {
            tracing::trace!("rmw_create_publisher: Failed to get context");
            return std::ptr::null_mut();
        }
    };

    // Register matched event callback with graph
    let events_mgr = zpub.events_mgr().clone();
    let graph = node_impl.inner.graph.clone();
    let notifier_clone_for_init = notifier.clone();
    let notifier_clone_for_matched = notifier.clone();
    let notifier_clone_for_incompatible = notifier.clone();
    let events_mgr_clone = zpub.events_mgr().clone();

    if let Err(e) = graph.event_manager.register_event_callback(
        entity_gid,
        ros_z::event::ZenohEventType::PublicationMatched,
        move |change| {
            if let Ok(mut mgr) = events_mgr.lock() {
                mgr.update_event_status(ros_z::event::ZenohEventType::PublicationMatched, change);
            }
            // Wake up wait sets
            notifier_clone_for_matched.notify_all();
        },
    ) {
        tracing::trace!(
            "rmw_create_publisher: Failed to register matched event callback: {:?}",
            e
        );
    }

    // Register QoS incompatibility event callback
    if let Err(e) = graph.event_manager.register_event_callback(
        entity_gid,
        ros_z::event::ZenohEventType::OfferedQosIncompatible,
        move |encoded_change| {
            // Decode policy_kind from upper 16 bits and change from lower 16 bits
            let policy_kind = ((encoded_change >> 16) & 0xFFFF) as u32;
            let change = encoded_change & 0xFFFF;
            if let Ok(mut mgr) = events_mgr_clone.lock() {
                mgr.update_event_status_with_policy(
                    ros_z::event::ZenohEventType::OfferedQosIncompatible,
                    change,
                    policy_kind,
                );
            }
            // Wake up wait sets
            notifier_clone_for_incompatible.notify_all();
        },
    ) {
        tracing::trace!(
            "rmw_create_publisher: Failed to register QoS incompatibility event callback: {:?}",
            e
        );
    }

    // Check if there are already existing subscriptions for this topic and trigger the event
    let matching_sub_count = graph.count(ros_z::entity::EntityKind::Subscription, &entity.topic);
    if matching_sub_count > 0 {
        if let Ok(mut mgr) = zpub.events_mgr().lock() {
            mgr.update_event_status(
                ros_z::event::ZenohEventType::PublicationMatched,
                matching_sub_count as i32,
            );
        }
        notifier_clone_for_init.notify_all();
    }

    // Check for QoS incompatibility with existing subscriptions (only once per unique subscription GID)
    let pub_qos = crate::qos::normalize_rmw_qos(unsafe { &*qos_profile });
    let sub_entities =
        graph.get_entities_by_topic(ros_z::entity::EntityKind::Subscription, &entity.topic);

    // Track which subscription GIDs we've already checked to avoid double-counting
    let local_zid = graph.zid;
    let mut checked_gids = std::collections::HashSet::new();
    let mut incompatible_count = 0;
    let mut last_policy_kind = 0u32;
    for sub_entity in &sub_entities {
        if let Some(endpoint) = ros_z::entity::entity_get_endpoint(sub_entity) {
            // Skip if we've already checked this GID (avoids double-counting if same subscription appears multiple times)
            let gid = ros_z::entity::endpoint_gid(endpoint);
            if !checked_gids.insert(gid) {
                continue;
            }

            // Only check QoS compatibility with entities from the same Zenoh session
            // This avoids counting subscriptions from previous test cases that used different sessions
            if endpoint.node.z_id != local_zid {
                continue;
            }

            let sub_qos = crate::qos::ros_z_qos_to_rmw_qos(
                &crate::pubsub::protocol_qos_to_ros_z_qos(&endpoint.qos),
            );
            let (compatible, policy_kind) =
                crate::qos::check_qos_compatibility_with_policy(&pub_qos, &sub_qos);
            if !compatible {
                incompatible_count += 1;
                last_policy_kind = policy_kind;
                // Also trigger the event on the subscription side (it's local, so it has an events_mgr)
                graph.event_manager.trigger_event_with_policy(
                    &ros_z::entity::endpoint_gid(endpoint),
                    ros_z::event::ZenohEventType::RequestedQosIncompatible,
                    1,
                    policy_kind,
                );
            }
        }
    }
    if incompatible_count > 0 {
        if let Ok(mut mgr) = zpub.events_mgr().lock() {
            mgr.update_event_status_with_policy(
                ros_z::event::ZenohEventType::OfferedQosIncompatible,
                incompatible_count,
                last_policy_kind,
            );
        }
        notifier_clone_for_init.notify_all();
    }

    let topic_cstr = match std::ffi::CString::new(qualified_topic) {
        Ok(cstr) => cstr,
        Err(e) => {
            tracing::trace!(
                "rmw_create_publisher: Failed to create CString for topic: {:?}",
                e
            );
            let msg = std::ffi::CString::new("Failed to create topic string").unwrap();
            unsafe {
                crate::ros::rcutils_set_error_state(msg.as_ptr(), cfile!(), line!() as usize)
            };
            return std::ptr::null_mut();
        }
    };

    let publisher_impl = PublisherImpl {
        inner: zpub,
        ts,
        topic: topic_cstr,
        options: unsafe { *publisher_options },
        qos: crate::qos::normalize_rmw_qos(unsafe { &*qos_profile }),
        graph: graph.clone(),
        entity: entity.clone(),
    };

    // Add local entity to graph for immediate discovery
    if let Err(e) = publisher_impl
        .graph
        .add_local_entity(ros_z::entity::Entity::Endpoint(entity))
    {
        tracing::error!("Failed to add local entity to graph: {:?}", e);
    }

    // Box the publisher_impl first so the topic CString lives on the heap
    let publisher_impl_boxed = Box::new(publisher_impl);
    let topic_ptr = publisher_impl_boxed.topic.as_ptr();
    let publisher_impl_ptr = Box::into_raw(publisher_impl_boxed);

    let publisher = Box::new(rmw_publisher_t {
        implementation_identifier: crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: publisher_impl_ptr as *mut _,
        topic_name: topic_ptr as *const _,
        options: unsafe { *publisher_options },
        can_loan_messages: false,
    });

    Box::into_raw(publisher)
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_publisher(
    node: *mut rmw_node_t,
    publisher: *mut rmw_publisher_t,
) -> rmw_ret_t {
    if node.is_null() || publisher.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Remove local entity from graph
    if let Ok(publisher_impl) = publisher.borrow_data() {
        let entity = ros_z::entity::Entity::Endpoint(publisher_impl.entity.clone());
        tracing::debug!(
            "rmw_destroy_publisher: Removing publisher entity: topic={}, type={:?}, id={}",
            publisher_impl.entity.topic,
            publisher_impl.entity.type_info.as_ref().map(|t| &t.name),
            publisher_impl.entity.id
        );
        if let Err(e) = publisher_impl.graph.remove_local_entity(&entity) {
            tracing::error!("Failed to remove local entity from graph: {:?}", e);
        }
    }

    tracing::debug!("rmw_destroy_publisher: Dropping publisher");

    // First, drop the PublisherImpl (which contains ZPub and LivelinessToken)
    let publisher_box = unsafe { Box::from_raw(publisher) };
    if !publisher_box.data.is_null() {
        let publisher_impl_ptr = publisher_box.data as *mut PublisherImpl;
        tracing::debug!("rmw_destroy_publisher: Dropping PublisherImpl");
        drop(unsafe { Box::from_raw(publisher_impl_ptr) });
        tracing::debug!("rmw_destroy_publisher: PublisherImpl dropped");
    }

    // Then drop the rmw_publisher_t (but without double-dropping the data field)
    // We need to prevent double-free, so we create a new rmw_publisher_t with null data
    let _ = rmw_publisher_t {
        implementation_identifier: publisher_box.implementation_identifier,
        data: std::ptr::null_mut(),
        topic_name: publisher_box.topic_name,
        options: publisher_box.options,
        can_loan_messages: publisher_box.can_loan_messages,
    };

    tracing::debug!("rmw_destroy_publisher: Publisher dropped");
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publish(
    publisher: *const rmw_publisher_t,
    ros_message: *const c_void,
    _allocation: *mut rmw_publisher_allocation_t,
) -> rmw_ret_t {
    if publisher.is_null() || ros_message.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let publisher_impl = match publisher.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match publisher_impl.publish(ros_message as *const std::os::raw::c_void) {
        Ok(_) => RMW_RET_OK as _,
        Err(e) => {
            tracing::error!("Failed to publish message: {}", e);
            RMW_RET_ERROR as _
        }
    }
}

// Subscriptions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_subscription(
    node: *const rmw_node_t,
    type_support: *const rosidl_message_type_support_t,
    topic_name: *const std::os::raw::c_char,
    qos_policies: *const rmw_qos_profile_t,
    subscription_options: *const rmw_subscription_options_t,
) -> *mut rmw_subscription_t {
    if node.is_null()
        || type_support.is_null()
        || topic_name.is_null()
        || qos_policies.is_null()
        || subscription_options.is_null()
    {
        return std::ptr::null_mut();
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };

    // Get context to access the shared notifier
    let context = unsafe { (*node).context };
    let context_impl = match context.borrow_impl() {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };
    let notifier = context_impl.share_notifier();

    let topic_str = unsafe { std::ffi::CStr::from_ptr(topic_name) }
        .to_str()
        .unwrap_or("");
    let ts = match unsafe { crate::type_support::MessageTypeSupport::new(type_support) } {
        Ok(ts) => ts,
        Err(_) => return std::ptr::null_mut(),
    };

    let zsub_builder = node_impl
        .inner
        .create_sub::<crate::msg::RosMessage>(topic_str)
        .with_serdes::<crate::msg::RosSerdes>();
    let qos = crate::qos::rmw_qos_to_ros_z_qos(unsafe { &*qos_policies });
    let mut zsub_builder = zsub_builder.with_qos(qos);

    // Apply ignore_local_publications option by restricting to remote publishers only
    let ignore_local = unsafe { (*subscription_options).ignore_local_publications };
    if ignore_local {
        zsub_builder = zsub_builder.with_locality(zenoh::sample::Locality::Remote);
        tracing::debug!("Subscription ignoring local publications");
    }

    let zsub_builder = zsub_builder.with_type_info(ts.get_type_info()); // Set type_info BEFORE build() so liveliness token has correct type

    // Create shared callback and user_data holders that will be populated after SubscriptionImpl is created
    let callback_holder: std::sync::Arc<
        std::sync::Mutex<crate::ros::rmw_subscription_new_message_callback_t>,
    > = std::sync::Arc::new(std::sync::Mutex::new(None));
    let user_data_holder = std::sync::Arc::new(std::sync::Mutex::new(0usize)); // Store pointer as usize for thread safety
    let unread_count_holder = std::sync::Arc::new(std::sync::Mutex::new(0usize)); // Track unread messages

    // Create notification callback that will wake up wait sets and invoke user callback
    let notifier_clone = notifier.clone();
    let callback_holder_clone = callback_holder.clone();
    let user_data_holder_clone = user_data_holder.clone();
    let unread_count_clone = unread_count_holder.clone();
    let notify_callback = move || {
        notifier_clone.notify_all();
        // Invoke the user callback if set, otherwise increment unread count
        if let Ok(cb) = callback_holder_clone.lock() {
            if let Some(callback_fn) = *cb {
                if let Ok(user_data_usize) = user_data_holder_clone.lock() {
                    unsafe {
                        let user_data_ptr = *user_data_usize as *const std::ffi::c_void;
                        callback_fn(user_data_ptr, 1); // 1 new message
                    }
                }
            } else {
                // No callback set, increment unread count
                if let Ok(mut unread) = unread_count_clone.lock() {
                    *unread += 1;
                }
            }
        }
    };

    let zsub = match zsub_builder.build_with_notifier(notify_callback) {
        Ok(zsub) => zsub,
        Err(_) => return std::ptr::null_mut(),
    };

    let entity = zsub.entity.clone();
    let entity_gid = ros_z::entity::endpoint_gid(&zsub.entity);

    // Register matched event callback with graph
    let events_mgr = zsub.events_mgr().clone();
    let graph = node_impl.inner.graph.clone();
    let notifier_clone_for_init = notifier.clone();
    let notifier_clone_for_matched = notifier.clone();
    let notifier_clone_for_incompatible = notifier.clone();
    let events_mgr_clone = zsub.events_mgr().clone();

    if let Err(e) = graph.event_manager.register_event_callback(
        entity_gid,
        ros_z::event::ZenohEventType::SubscriptionMatched,
        move |change| {
            if let Ok(mut mgr) = events_mgr.lock() {
                mgr.update_event_status(ros_z::event::ZenohEventType::SubscriptionMatched, change);
            }
            // Wake up wait sets
            notifier_clone_for_matched.notify_all();
        },
    ) {
        tracing::trace!(
            "rmw_create_subscription: Failed to register matched event callback: {:?}",
            e
        );
    }

    // Register QoS incompatibility event callback
    if let Err(e) = graph.event_manager.register_event_callback(
        entity_gid,
        ros_z::event::ZenohEventType::RequestedQosIncompatible,
        move |encoded_change| {
            // Decode policy_kind from upper 16 bits and change from lower 16 bits
            let policy_kind = ((encoded_change >> 16) & 0xFFFF) as u32;
            let change = encoded_change & 0xFFFF;
            if let Ok(mut mgr) = events_mgr_clone.lock() {
                mgr.update_event_status_with_policy(
                    ros_z::event::ZenohEventType::RequestedQosIncompatible,
                    change,
                    policy_kind,
                );
            }
            // Wake up wait sets
            notifier_clone_for_incompatible.notify_all();
        },
    ) {
        tracing::trace!(
            "rmw_create_subscription: Failed to register QoS incompatibility event callback: {:?}",
            e
        );
    }

    // Check if there are already existing publishers for this topic and trigger the event
    let matching_pub_count = graph.count(ros_z::entity::EntityKind::Publisher, &entity.topic);
    if matching_pub_count > 0 {
        if let Ok(mut mgr) = zsub.events_mgr().lock() {
            mgr.update_event_status(
                ros_z::event::ZenohEventType::SubscriptionMatched,
                matching_pub_count as i32,
            );
        }
        notifier_clone_for_init.notify_all();
    }

    // Check for QoS incompatibility with existing publishers (only once per unique publisher GID)
    let sub_qos = crate::qos::normalize_rmw_qos(unsafe { &*qos_policies });
    let pub_entities =
        graph.get_entities_by_topic(ros_z::entity::EntityKind::Publisher, &entity.topic);

    // Track which publisher GIDs we've already checked to avoid double-counting
    let local_zid = graph.zid;
    let mut checked_gids = std::collections::HashSet::new();
    let mut incompatible_count = 0;
    let mut last_policy_kind = 0u32;
    for pub_entity in &pub_entities {
        if let Some(endpoint) = ros_z::entity::entity_get_endpoint(pub_entity) {
            // Skip if we've already checked this GID (avoids double-counting if same publisher appears multiple times)
            let gid = ros_z::entity::endpoint_gid(endpoint);
            if !checked_gids.insert(gid) {
                continue;
            }

            // Only check QoS compatibility with entities from the same Zenoh session
            // This avoids counting publishers from previous test cases that used different sessions
            if endpoint.node.z_id != local_zid {
                continue;
            }

            let pub_qos = crate::qos::ros_z_qos_to_rmw_qos(
                &crate::pubsub::protocol_qos_to_ros_z_qos(&endpoint.qos),
            );
            let (compatible, policy_kind) =
                crate::qos::check_qos_compatibility_with_policy(&pub_qos, &sub_qos);
            if !compatible {
                incompatible_count += 1;
                last_policy_kind = policy_kind;
                // Also trigger the event on the publisher side (it's local, so it has an events_mgr)
                graph.event_manager.trigger_event_with_policy(
                    &ros_z::entity::endpoint_gid(endpoint),
                    ros_z::event::ZenohEventType::OfferedQosIncompatible,
                    1,
                    policy_kind,
                );
            }
        }
    }
    if incompatible_count > 0 {
        if let Ok(mut mgr) = zsub.events_mgr().lock() {
            mgr.update_event_status_with_policy(
                ros_z::event::ZenohEventType::RequestedQosIncompatible,
                incompatible_count,
                last_policy_kind,
            );
        }
        notifier_clone_for_init.notify_all();
    }

    let topic_cstr = match std::ffi::CString::new(topic_str) {
        Ok(cstr) => cstr,
        Err(_) => return std::ptr::null_mut(),
    };

    let subscription_impl = crate::pubsub::SubscriptionImpl {
        inner: zsub,
        ts,
        topic: topic_cstr.clone(),
        options: unsafe { *subscription_options },
        qos: crate::qos::normalize_rmw_qos(unsafe { &*qos_policies }),
        callback: callback_holder,
        callback_user_data: user_data_holder,
        unread_count: unread_count_holder,
        graph: graph.clone(),
        entity: entity.clone(),
        notifier,
    };

    // Add local entity to graph for immediate discovery
    if let Err(e) = subscription_impl
        .graph
        .add_local_entity(ros_z::entity::Entity::Endpoint(entity))
    {
        tracing::error!("Failed to add local entity to graph: {:?}", e);
    }

    let subscription = Box::new(rmw_subscription_t {
        implementation_identifier: crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        topic_name: subscription_impl.topic.as_ptr() as *const _,
        options: unsafe { *subscription_options },
        can_loan_messages: false,
        is_cft_enabled: false,
    });

    let subscription_ptr = Box::into_raw(subscription);
    unsafe {
        (*subscription_ptr).data = Box::into_raw(Box::new(subscription_impl)) as *mut _;
    }

    subscription_ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_subscription(
    node: *mut rmw_node_t,
    subscription: *mut rmw_subscription_t,
) -> rmw_ret_t {
    if node.is_null() || subscription.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Remove local entity from graph
    if let Ok(subscription_impl) = subscription.borrow_data() {
        let entity = ros_z::entity::Entity::Endpoint(subscription_impl.entity.clone());
        if let Err(e) = subscription_impl.graph.remove_local_entity(&entity) {
            tracing::trace!(
                "rmw_destroy_subscription: Failed to remove local entity from graph: {:?}",
                e
            );
        }
    }

    // First, drop the SubscriptionImpl (which contains ZSub and LivelinessToken)
    let subscription_box = unsafe { Box::from_raw(subscription) };
    if !subscription_box.data.is_null() {
        let subscription_impl_ptr = subscription_box.data as *mut SubscriptionImpl;
        drop(unsafe { Box::from_raw(subscription_impl_ptr) });
    }

    // Then drop the rmw_subscription_t without double-dropping the data field
    let _ = rmw_subscription_t {
        implementation_identifier: subscription_box.implementation_identifier,
        data: std::ptr::null_mut(),
        topic_name: subscription_box.topic_name,
        options: subscription_box.options,
        can_loan_messages: subscription_box.can_loan_messages,
        is_cft_enabled: subscription_box.is_cft_enabled,
    };

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take(
    subscription: *const rmw_subscription_t,
    ros_message: *mut c_void,
    taken: *mut bool,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    if subscription.is_null() || ros_message.is_null() || taken.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match subscription.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match subscription_impl.take(ros_message as *mut std::os::raw::c_void, taken) {
        Ok(_) => RMW_RET_OK as _,
        Err(_) => RMW_RET_ERROR as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_event(
    event: *const rmw_event_t,
    event_info: *mut c_void,
    taken: *mut bool,
) -> rmw_ret_t {
    if event.is_null() || event_info.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let rmw_event = unsafe { &*event };

    // The data field contains the raw pointer to RmEventHandle
    if rmw_event.data.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let event_handle = unsafe { &*(rmw_event.data as *const RmEventHandle) };
    let status = event_handle.take_event();

    // Initialize taken to false
    if !taken.is_null() {
        unsafe {
            *taken = false;
        }
    }

    // Convert event_type to ZenohEventType to determine which structure to fill
    let zenoh_event_type = rmw_event_type_to_zenoh_event(rmw_event.event_type);
    if zenoh_event_type.is_none() {
        tracing::error!("Unsupported event type: {}", rmw_event.event_type);
        return RMW_RET_UNSUPPORTED as _;
    }

    // Fill the appropriate status structure based on event type
    match zenoh_event_type.unwrap() {
        ZenohEventType::RequestedQosIncompatible => {
            let status_ptr = event_info as *mut rmw_requested_qos_incompatible_event_status_t;
            unsafe {
                (*status_ptr).total_count = status.total_count;
                (*status_ptr).total_count_change = status.total_count_change;
                (*status_ptr).last_policy_kind = status.last_policy_kind;
            }
        }
        ZenohEventType::OfferedQosIncompatible => {
            let status_ptr = event_info as *mut rmw_offered_qos_incompatible_event_status_t;
            unsafe {
                (*status_ptr).total_count = status.total_count;
                (*status_ptr).total_count_change = status.total_count_change;
                (*status_ptr).last_policy_kind = status.last_policy_kind;
            }
        }
        ZenohEventType::MessageLost => {
            let status_ptr = event_info as *mut rmw_message_lost_status_t;
            unsafe {
                (*status_ptr).total_count = status.total_count as usize;
                (*status_ptr).total_count_change = status.total_count_change as usize;
            }
        }
        ZenohEventType::SubscriptionIncompatibleType
        | ZenohEventType::PublisherIncompatibleType => {
            let status_ptr = event_info as *mut rmw_incompatible_type_status_t;
            unsafe {
                (*status_ptr).total_count = status.total_count;
                (*status_ptr).total_count_change = status.total_count_change;
            }
        }
        ZenohEventType::SubscriptionMatched | ZenohEventType::PublicationMatched => {
            let status_ptr = event_info as *mut rmw_matched_status_t;
            unsafe {
                (*status_ptr).total_count = status.total_count as usize;
                (*status_ptr).total_count_change = status.total_count_change as usize;
                (*status_ptr).current_count = status.current_count as usize;
                (*status_ptr).current_count_change = status.current_count_change;
            }
        }
        ZenohEventType::OfferedDeadlineMissed => {
            let status_ptr = event_info as *mut rmw_offered_deadline_missed_status_t;
            unsafe {
                (*status_ptr).total_count = status.total_count;
                (*status_ptr).total_count_change = status.total_count_change;
            }
        }
        ZenohEventType::RequestedDeadlineMissed => {
            let status_ptr = event_info as *mut rmw_requested_deadline_missed_status_t;
            unsafe {
                (*status_ptr).total_count = status.total_count;
                (*status_ptr).total_count_change = status.total_count_change;
            }
        }
        ZenohEventType::LivelinessLost => {
            let status_ptr = event_info as *mut rmw_liveliness_lost_status_t;
            unsafe {
                (*status_ptr).total_count = status.total_count;
                (*status_ptr).total_count_change = status.total_count_change;
            }
        }
        ZenohEventType::LivelinessChanged => {
            let status_ptr = event_info as *mut rmw_liveliness_changed_status_t;
            unsafe {
                (*status_ptr).alive_count = status.current_count;
                (*status_ptr).alive_count_change = status.current_count_change;
                (*status_ptr).not_alive_count = 0; // TODO: track not alive count
                (*status_ptr).not_alive_count_change = 0;
            }
        }
    }

    // Set taken to true to indicate the event was successfully retrieved
    if !taken.is_null() {
        unsafe {
            *taken = true;
        }
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_get_network_flow_endpoints(
    _subscription: *const rmw_subscription_t,
    _allocator: *const rcl_allocator_t,
    _network_flow_endpoints: *mut rmw_network_flow_endpoint_array_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

// Services
#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_client(
    node: *const rmw_node_t,
    type_support: *const rosidl_service_type_support_t,
    service_name: *const std::os::raw::c_char,
    qos_policies: *const rmw_qos_profile_t,
) -> *mut rmw_client_t {
    if node.is_null() || type_support.is_null() || service_name.is_null() || qos_policies.is_null()
    {
        return std::ptr::null_mut();
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };

    // Get context to access the shared notifier
    let context = unsafe { (*node).context };
    let context_impl = match context.borrow_impl() {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };
    let notifier = context_impl.share_notifier();

    let service_str = unsafe { std::ffi::CStr::from_ptr(service_name) }
        .to_str()
        .unwrap_or("");

    // Create client using ros-z
    let qos = crate::qos::rmw_qos_to_ros_z_qos(unsafe { &*qos_policies });
    let qualified_service = match ros_z::topic_name::qualify_service_name(
        service_str,
        &node_impl.inner.entity.namespace,
        &node_impl.inner.entity.name,
    ) {
        Ok(s) => s,
        Err(e) => {
            tracing::error!("Failed to qualify service name: {}", e);
            return std::ptr::null_mut();
        }
    };

    let service_type_support =
        match unsafe { crate::type_support::ServiceTypeSupport::new(type_support) } {
            Ok(ts) => ts,
            Err(e) => {
                tracing::error!("Failed to create service type support: {}", e);
                return std::ptr::null_mut();
            }
        };

    let mut zclient_builder = node_impl
        .inner
        .create_client::<crate::msg::RosService>(&qualified_service)
        .with_type_info(service_type_support.get_type_info());
    zclient_builder.entity.qos = qos.to_protocol_qos();
    let entity = zclient_builder.entity.clone();

    // Create shared callback and user_data holders
    let callback_holder: std::sync::Arc<
        std::sync::Mutex<crate::ros::rmw_client_new_response_callback_t>,
    > = std::sync::Arc::new(std::sync::Mutex::new(None));
    let user_data_holder = std::sync::Arc::new(std::sync::Mutex::new(0usize));

    // Build the client (notification callback will be set per-request in send_request)
    let zclient = match zclient_builder.build() {
        Ok(client) => client,
        Err(e) => {
            tracing::error!("Failed to create client: {}", e);
            return std::ptr::null_mut();
        }
    };

    let service_cstr = match std::ffi::CString::new(qualified_service.clone()) {
        Ok(cstr) => cstr,
        Err(_) => return std::ptr::null_mut(),
    };

    let client_impl = crate::service::ClientImpl {
        inner: zclient,
        service_name: service_cstr,
        options: rmw_client_options_t {
            qos: crate::qos::normalize_rmw_qos(unsafe { &*qos_policies }),
        },
        request_ts: service_type_support,
        response_ts: service_type_support,
        callback: callback_holder,
        callback_user_data: user_data_holder,
        notifier,
        sequence_counter: std::sync::atomic::AtomicI64::new(1), // Start at 1 for ROS compatibility
        unread_count: std::sync::Arc::new(std::sync::Mutex::new(0)), // Track unread responses
        graph: node_impl.inner.graph.clone(),
        entity: entity.clone(),
    };

    // Add local entity to graph for immediate discovery
    if let Err(e) = client_impl
        .graph
        .add_local_entity(ros_z::entity::Entity::Endpoint(entity))
    {
        tracing::trace!(
            "rmw_create_client: Failed to add local entity to graph: {:?}",
            e
        );
    }

    // Get the service name pointer before moving client_impl
    let service_name_ptr = client_impl.service_name.as_ptr();

    let client = Box::new(rmw_client_t {
        implementation_identifier: crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        service_name: service_name_ptr as *const _,
    });

    let client_ptr = Box::into_raw(client);
    client_ptr.assign_data(client_impl).unwrap_or(());

    client_ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_client(
    node: *mut rmw_node_t,
    client: *mut rmw_client_t,
) -> rmw_ret_t {
    if node.is_null() || client.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Extract entity info before dropping the client
    let (entity, graph) = if let Ok(client_impl) = client.borrow_data() {
        (
            Some(ros_z::entity::Entity::Endpoint(client_impl.entity.clone())),
            Some(client_impl.graph.clone()),
        )
    } else {
        (None, None)
    };

    // First, drop the ClientImpl (which contains ZClient and LivelinessToken)
    let client_box = unsafe { Box::from_raw(client) };
    if !client_box.data.is_null() {
        let client_impl_ptr = client_box.data as *mut ClientImpl;
        drop(unsafe { Box::from_raw(client_impl_ptr) });
    }

    // Then drop the rmw_client_t without double-dropping the data field
    let _ = rmw_client_t {
        implementation_identifier: client_box.implementation_identifier,
        data: std::ptr::null_mut(),
        service_name: client_box.service_name,
    };

    // Then remove from graph to clean up the slabs
    if let (Some(entity), Some(graph)) = (entity, graph) {
        let _ = graph.remove_local_entity(&entity);
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_service(
    node: *const rmw_node_t,
    type_support: *const rosidl_service_type_support_t,
    service_name: *const std::os::raw::c_char,
    qos_profile: *const rmw_qos_profile_t,
) -> *mut rmw_service_t {
    if node.is_null() || type_support.is_null() || service_name.is_null() || qos_profile.is_null() {
        return std::ptr::null_mut();
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };

    // Get context to access the shared notifier
    let context = unsafe { (*node).context };
    let context_impl = match context.borrow_impl() {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };
    let notifier = context_impl.share_notifier();

    let service_str = unsafe { std::ffi::CStr::from_ptr(service_name) }
        .to_str()
        .unwrap_or("");

    // Create service using ros-z
    let qos = crate::qos::rmw_qos_to_ros_z_qos(unsafe { &*qos_profile });
    let qualified_service = match ros_z::topic_name::qualify_service_name(
        service_str,
        &node_impl.inner.entity.namespace,
        &node_impl.inner.entity.name,
    ) {
        Ok(s) => s,
        Err(e) => {
            tracing::error!("Failed to qualify service name: {}", e);
            return std::ptr::null_mut();
        }
    };

    let service_type_support =
        match unsafe { crate::type_support::ServiceTypeSupport::new(type_support) } {
            Ok(ts) => ts,
            Err(e) => {
                tracing::error!("Failed to create service type support: {}", e);
                return std::ptr::null_mut();
            }
        };

    let mut zserver_builder = node_impl
        .inner
        .create_service::<crate::msg::RosService>(&qualified_service)
        .with_type_info(service_type_support.get_type_info());
    zserver_builder.entity.qos = qos.to_protocol_qos();
    let entity = zserver_builder.entity.clone();

    // Create shared callback and user_data holders that will be populated after ServiceImpl is created
    let callback_holder: std::sync::Arc<
        std::sync::Mutex<crate::ros::rmw_service_new_request_callback_t>,
    > = std::sync::Arc::new(std::sync::Mutex::new(None));
    let user_data_holder = std::sync::Arc::new(std::sync::Mutex::new(0usize)); // Store pointer as usize for thread safety
    let unread_count_holder = std::sync::Arc::new(std::sync::Mutex::new(0usize)); // Track unread requests

    // Create notification callback that will wake up wait sets and invoke user callback
    let notifier_clone = notifier.clone();
    let callback_holder_clone = callback_holder.clone();
    let user_data_holder_clone = user_data_holder.clone();
    let unread_count_clone = unread_count_holder.clone();
    let notify_callback = move || {
        notifier_clone.notify_all();
        // Invoke user callback if set, otherwise increment unread count
        if let Ok(cb) = callback_holder_clone.lock() {
            if let Some(callback_fn) = *cb {
                if let Ok(user_data_usize) = user_data_holder_clone.lock() {
                    unsafe {
                        let user_data_ptr = *user_data_usize as *const std::ffi::c_void;
                        callback_fn(user_data_ptr, 1); // 1 new request
                    }
                }
            } else {
                // No callback set, increment unread count
                if let Ok(mut unread) = unread_count_clone.lock() {
                    *unread += 1;
                }
            }
        }
    };

    let zserver = match zserver_builder.build_with_notifier(notify_callback) {
        Ok(server) => server,
        Err(e) => {
            tracing::error!("Failed to create service: {}", e);
            return std::ptr::null_mut();
        }
    };

    let service_name_cstr = match std::ffi::CString::new(qualified_service.clone()) {
        Ok(cstr) => cstr,
        Err(_) => return std::ptr::null_mut(),
    };

    let service_impl = crate::service::ServiceImpl {
        inner: zserver,
        service_name: service_name_cstr,
        request_ts: service_type_support,
        response_ts: service_type_support,
        qos: crate::qos::normalize_rmw_qos(unsafe { &*qos_profile }),
        callback: callback_holder,
        callback_user_data: user_data_holder,
        unread_count: unread_count_holder, // Use the same Arc created earlier for notify_callback
        graph: node_impl.inner.graph.clone(),
        entity: entity.clone(),
    };

    // Add local entity to graph for immediate discovery
    if let Err(e) = service_impl
        .graph
        .add_local_entity(ros_z::entity::Entity::Endpoint(entity))
    {
        tracing::trace!(
            "rmw_create_service: Failed to add local entity to graph: {:?}",
            e
        );
    }

    let service = Box::new(rmw_service_t {
        implementation_identifier: crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        service_name: std::ptr::null(),
    });

    let service_ptr = Box::into_raw(service);
    service_ptr.assign_data(service_impl).unwrap_or(());

    // Update service_name pointer to point to the name stored in the impl
    unsafe {
        if let Ok(impl_ref) = service_ptr.borrow_data() {
            (*service_ptr).service_name = impl_ref.service_name.as_ptr();
        }
    }

    service_ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_service(
    node: *mut rmw_node_t,
    service: *mut rmw_service_t,
) -> rmw_ret_t {
    if node.is_null() || service.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Extract entity info before dropping the service
    let (entity, graph) = if let Ok(service_impl) = service.borrow_data() {
        (
            Some(ros_z::entity::Entity::Endpoint(service_impl.entity.clone())),
            Some(service_impl.graph.clone()),
        )
    } else {
        (None, None)
    };

    // First, drop the ServiceImpl (which contains ZService and LivelinessToken)
    let service_box = unsafe { Box::from_raw(service) };
    if !service_box.data.is_null() {
        let service_impl_ptr = service_box.data as *mut ServiceImpl;
        drop(unsafe { Box::from_raw(service_impl_ptr) });
    }

    // Then drop the rmw_service_t without double-dropping the data field
    let _ = rmw_service_t {
        implementation_identifier: service_box.implementation_identifier,
        data: std::ptr::null_mut(),
        service_name: service_box.service_name,
    };

    // Then remove from graph to clean up the slabs
    if let (Some(entity), Some(graph)) = (entity, graph) {
        let _ = graph.remove_local_entity(&entity);
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_service_get_service_name(
    service: *const rmw_service_t,
) -> *const std::os::raw::c_char {
    if service.is_null() {
        return std::ptr::null();
    }

    match service.borrow_data() {
        Ok(service_impl) => service_impl.service_name.as_ptr(),
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_client_get_service_name(
    client: *const rmw_client_t,
) -> *const std::os::raw::c_char {
    if client.is_null() {
        return std::ptr::null();
    }

    match client.borrow_data() {
        Ok(client_impl) => client_impl.service_name.as_ptr(),
        Err(_) => std::ptr::null(),
    }
}

// Graph queries
#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_topic_names_and_types(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    _no_demangle: bool,
    topic_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    if node.is_null() || allocator.is_null() || topic_names_and_types.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Get topic names and types from graph
    let topics_and_types = node_impl.inner.graph.get_topic_names_and_types();

    // Group by topic name
    let mut topic_map: std::collections::HashMap<String, Vec<String>> =
        std::collections::HashMap::new();
    for (topic, type_name) in topics_and_types {
        topic_map.entry(topic).or_default().push(type_name);
    }

    let topic_count = topic_map.len();

    // Initialize names_and_types
    unsafe {
        let ret = rmw_names_and_types_init(topic_names_and_types, topic_count, allocator as *mut _);
        if ret != RMW_RET_OK as i32 {
            return RMW_RET_BAD_ALLOC as _;
        }

        // Populate the arrays
        for (index, (topic_name, type_names)) in topic_map.iter().enumerate() {
            // Set topic name
            let topic_cstr = match std::ffi::CString::new(topic_name.as_str()) {
                Ok(s) => s,
                Err(_) => {
                    rmw_names_and_types_fini(topic_names_and_types);
                    return RMW_RET_ERROR as _;
                }
            };

            (*topic_names_and_types)
                .names
                .data
                .add(index)
                .write(rcutils_strdup(topic_cstr.as_ptr(), *allocator));
            if (*topic_names_and_types)
                .names
                .data
                .add(index)
                .read()
                .is_null()
            {
                rmw_names_and_types_fini(topic_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            // Initialize types array for this topic
            let ret = rcutils_string_array_init(
                (*topic_names_and_types).types.add(index),
                type_names.len(),
                allocator,
            );
            if ret != 0 {
                rmw_names_and_types_fini(topic_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            // Populate type names
            for (type_index, type_name) in type_names.iter().enumerate() {
                let type_cstr = match std::ffi::CString::new(type_name.as_str()) {
                    Ok(s) => s,
                    Err(_) => {
                        rmw_names_and_types_fini(topic_names_and_types);
                        return RMW_RET_ERROR as _;
                    }
                };

                (*(*topic_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .write(rcutils_strdup(type_cstr.as_ptr(), *allocator));
                if (*(*topic_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .read()
                    .is_null()
                {
                    rmw_names_and_types_fini(topic_names_and_types);
                    return RMW_RET_BAD_ALLOC as _;
                }
            }
        }
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_service_names_and_types(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    service_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    if node.is_null() || allocator.is_null() || service_names_and_types.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Get service names and types from graph
    let services_and_types = node_impl.inner.graph.get_service_names_and_types();

    // Group by service name
    let mut service_map: std::collections::HashMap<String, Vec<String>> =
        std::collections::HashMap::new();
    for (service, type_name) in services_and_types {
        service_map.entry(service).or_default().push(type_name);
    }

    let service_count = service_map.len();

    // Initialize names_and_types
    unsafe {
        let ret =
            rmw_names_and_types_init(service_names_and_types, service_count, allocator as *mut _);
        if ret != RMW_RET_OK as i32 {
            return RMW_RET_BAD_ALLOC as _;
        }

        // Populate the arrays
        for (index, (service_name, type_names)) in service_map.iter().enumerate() {
            // Set service name
            let service_cstr = match std::ffi::CString::new(service_name.as_str()) {
                Ok(s) => s,
                Err(_) => {
                    rmw_names_and_types_fini(service_names_and_types);
                    return RMW_RET_ERROR as _;
                }
            };

            (*service_names_and_types)
                .names
                .data
                .add(index)
                .write(rcutils_strdup(service_cstr.as_ptr(), *allocator));
            if (*service_names_and_types)
                .names
                .data
                .add(index)
                .read()
                .is_null()
            {
                rmw_names_and_types_fini(service_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            // Initialize types array for this service
            let ret = rcutils_string_array_init(
                (*service_names_and_types).types.add(index),
                type_names.len(),
                allocator,
            );
            if ret != 0 {
                rmw_names_and_types_fini(service_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            // Populate type names
            for (type_index, type_name) in type_names.iter().enumerate() {
                let type_cstr = match std::ffi::CString::new(type_name.as_str()) {
                    Ok(s) => s,
                    Err(_) => {
                        rmw_names_and_types_fini(service_names_and_types);
                        return RMW_RET_ERROR as _;
                    }
                };

                (*(*service_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .write(rcutils_strdup(type_cstr.as_ptr(), *allocator));
                if (*(*service_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .read()
                    .is_null()
                {
                    rmw_names_and_types_fini(service_names_and_types);
                    return RMW_RET_BAD_ALLOC as _;
                }
            }
        }
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_count_publishers(
    node: *const rmw_node_t,
    topic_name: *const std::os::raw::c_char,
    count: *mut usize,
) -> rmw_ret_t {
    if node.is_null() || topic_name.is_null() || count.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let topic_str = match unsafe { std::ffi::CStr::from_ptr(topic_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Query graph for publisher count on this topic
    let publisher_count = node_impl
        .inner
        .graph
        .count(ros_z::entity::EntityKind::Publisher, topic_str);

    unsafe {
        *count = publisher_count;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_count_subscribers(
    node: *const rmw_node_t,
    topic_name: *const std::os::raw::c_char,
    count: *mut usize,
) -> rmw_ret_t {
    if node.is_null() || topic_name.is_null() || count.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let topic_str = match unsafe { std::ffi::CStr::from_ptr(topic_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Query graph for subscriber count on this topic
    let subscriber_count = node_impl
        .inner
        .graph
        .count(ros_z::entity::EntityKind::Subscription, topic_str);

    unsafe {
        *count = subscriber_count;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_node_get_graph_guard_condition(
    node: *const rmw_node_t,
) -> *const rmw_guard_condition_t {
    if node.is_null() {
        return std::ptr::null();
    }

    // Get the graph guard condition from the node implementation
    match node.borrow_data() {
        Ok(node_impl) => node_impl.graph_guard_condition,
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_count_clients(
    node: *const rmw_node_t,
    service_name: *const std::os::raw::c_char,
    count: *mut usize,
) -> rmw_ret_t {
    if node.is_null() || service_name.is_null() || count.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let service_str = match unsafe { std::ffi::CStr::from_ptr(service_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Query graph for client count on this service
    let client_count = node_impl
        .inner
        .graph
        .count_by_service(ros_z::entity::EntityKind::Client, service_str);

    unsafe {
        *count = client_count;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_count_services(
    node: *const rmw_node_t,
    service_name: *const std::os::raw::c_char,
    count: *mut usize,
) -> rmw_ret_t {
    if node.is_null() || service_name.is_null() || count.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let service_str = match unsafe { std::ffi::CStr::from_ptr(service_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Query graph for service count on this service
    let service_count = node_impl
        .inner
        .graph
        .count_by_service(ros_z::entity::EntityKind::Service, service_str);

    unsafe {
        *count = service_count;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_publishers_info_by_topic(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    topic_name: *const std::os::raw::c_char,
    _no_mangle: bool,
    publishers_info: *mut rmw_topic_endpoint_info_array_t,
) -> rmw_ret_t {
    if node.is_null() || allocator.is_null() || topic_name.is_null() || publishers_info.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let topic_str = match unsafe { std::ffi::CStr::from_ptr(topic_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Get publishers for this topic
    let publishers = node_impl
        .inner
        .graph
        .get_entities_by_topic(ros_z::entity::EntityKind::Publisher, topic_str);

    let count = publishers.len();

    // Allocate array
    unsafe {
        let ret = rmw_topic_endpoint_info_array_init_with_size(
            publishers_info,
            count,
            allocator as *mut _,
        );
        if ret != RMW_RET_OK as i32 {
            return RMW_RET_BAD_ALLOC as _;
        }

        // Populate publisher info
        for (i, entity) in publishers.iter().enumerate() {
            // Extract endpoint entity from Entity enum
            let endpoint = match entity.as_ref() {
                ros_z::entity::Entity::Endpoint(ep) => ep,
                _ => continue, // Skip non-endpoint entities
            };

            // Convert entity to endpoint info
            let endpoint_info = (*publishers_info).info_array.add(i);

            // Set node name
            let node_name_cstr = match std::ffi::CString::new(endpoint.node.name.as_str()) {
                Ok(s) => s,
                Err(_) => {
                    rmw_topic_endpoint_info_array_fini(publishers_info, allocator as *mut _);
                    return RMW_RET_ERROR as _;
                }
            };
            (*endpoint_info).node_name = rcutils_strdup(node_name_cstr.as_ptr(), *allocator);

            // Set node namespace
            let node_ns = if endpoint.node.namespace.is_empty() {
                "/"
            } else {
                &endpoint.node.namespace
            };
            let node_ns_cstr = match std::ffi::CString::new(node_ns) {
                Ok(s) => s,
                Err(_) => {
                    rmw_topic_endpoint_info_array_fini(publishers_info, allocator as *mut _);
                    return RMW_RET_ERROR as _;
                }
            };
            (*endpoint_info).node_namespace = rcutils_strdup(node_ns_cstr.as_ptr(), *allocator);

            // Set topic type
            if let Some(ref type_info) = endpoint.type_info {
                let type_cstr = match std::ffi::CString::new(type_info.name.as_str()) {
                    Ok(s) => s,
                    Err(_) => {
                        rmw_topic_endpoint_info_array_fini(publishers_info, allocator as *mut _);
                        return RMW_RET_ERROR as _;
                    }
                };
                (*endpoint_info).topic_type = rcutils_strdup(type_cstr.as_ptr(), *allocator);
                (*endpoint_info).topic_type_hash = rosidl_type_hash_t {
                    version: type_info.hash.version,
                    value: type_info.hash.value,
                };
            } else {
                (*endpoint_info).topic_type = std::ptr::null();
            }

            // Set endpoint type
            (*endpoint_info).endpoint_type = rmw_endpoint_type_e_RMW_ENDPOINT_PUBLISHER;

            // Set GID - endpoint_gid is just a [u8; 16] array
            let gid_bytes = endpoint.id.to_ne_bytes();
            let mut gid_data = [0u8; 16];
            let copy_len = std::cmp::min(std::mem::size_of::<usize>(), 16);
            gid_data[..copy_len].copy_from_slice(&gid_bytes[..copy_len]);
            (*endpoint_info).endpoint_gid = gid_data;

            // Set QoS profile - convert from protocol QoS to ros_z QoS to rmw QoS
            let ros_z_qos = ros_z::qos::QosProfile {
                reliability: match endpoint.qos.reliability {
                    ros_z_protocol::qos::QosReliability::Reliable => {
                        ros_z::qos::QosReliability::Reliable
                    }
                    ros_z_protocol::qos::QosReliability::BestEffort => {
                        ros_z::qos::QosReliability::BestEffort
                    }
                },
                durability: match endpoint.qos.durability {
                    ros_z_protocol::qos::QosDurability::TransientLocal => {
                        ros_z::qos::QosDurability::TransientLocal
                    }
                    ros_z_protocol::qos::QosDurability::Volatile => {
                        ros_z::qos::QosDurability::Volatile
                    }
                },
                history: match endpoint.qos.history {
                    ros_z_protocol::qos::QosHistory::KeepLast(depth) => {
                        ros_z::qos::QosHistory::from_depth(depth)
                    }
                    ros_z_protocol::qos::QosHistory::KeepAll => ros_z::qos::QosHistory::KeepAll,
                },
                ..Default::default()
            };
            (*endpoint_info).qos_profile = crate::qos::ros_z_qos_to_rmw_qos(&ros_z_qos);
        }
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_subscriber_names_and_types_by_node(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    node_name: *const std::os::raw::c_char,
    node_namespace: *const std::os::raw::c_char,
    _no_demangle: bool,
    topic_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    if node.is_null()
        || allocator.is_null()
        || node_name.is_null()
        || node_namespace.is_null()
        || topic_names_and_types.is_null()
    {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Convert node name and namespace to strings
    let target_node_name = match unsafe { std::ffi::CStr::from_ptr(node_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };
    let target_node_ns = match unsafe { std::ffi::CStr::from_ptr(node_namespace) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let node_key = (
        normalize_namespace(target_node_ns),
        target_node_name.to_string(),
    );

    // Check if the node exists in the graph
    if !node_impl.inner.graph.node_exists(node_key.clone()) {
        return RMW_RET_NODE_NAME_NON_EXISTENT as _;
    }

    // Get entities for this node
    let entities_and_types = node_impl
        .inner
        .graph
        .get_names_and_types_by_node(node_key, ros_z::entity::EntityKind::Subscription);

    // Group by entity name
    let mut entity_map: std::collections::HashMap<String, Vec<String>> =
        std::collections::HashMap::new();
    for (name, type_name) in entities_and_types {
        entity_map.entry(name).or_default().push(type_name);
    }

    let entity_count = entity_map.len();

    // Initialize names_and_types
    unsafe {
        let ret =
            rmw_names_and_types_init(topic_names_and_types, entity_count, allocator as *mut _);
        if ret != RMW_RET_OK as i32 {
            return RMW_RET_BAD_ALLOC as _;
        }

        // Populate the arrays
        for (index, (entity_name, type_names)) in entity_map.iter().enumerate() {
            let entity_cstr = match std::ffi::CString::new(entity_name.as_str()) {
                Ok(s) => s,
                Err(_) => {
                    rmw_names_and_types_fini(topic_names_and_types);
                    return RMW_RET_ERROR as _;
                }
            };

            (*topic_names_and_types)
                .names
                .data
                .add(index)
                .write(rcutils_strdup(entity_cstr.as_ptr(), *allocator));
            if (*topic_names_and_types)
                .names
                .data
                .add(index)
                .read()
                .is_null()
            {
                rmw_names_and_types_fini(topic_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            // Initialize types array
            let ret = rcutils_string_array_init(
                (*topic_names_and_types).types.add(index),
                type_names.len(),
                allocator,
            );
            if ret != 0 {
                rmw_names_and_types_fini(topic_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            // Populate type names
            for (type_index, type_name) in type_names.iter().enumerate() {
                let type_cstr = match std::ffi::CString::new(type_name.as_str()) {
                    Ok(s) => s,
                    Err(_) => {
                        rmw_names_and_types_fini(topic_names_and_types);
                        return RMW_RET_ERROR as _;
                    }
                };

                (*(*topic_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .write(rcutils_strdup(type_cstr.as_ptr(), *allocator));
                if (*(*topic_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .read()
                    .is_null()
                {
                    rmw_names_and_types_fini(topic_names_and_types);
                    return RMW_RET_BAD_ALLOC as _;
                }
            }
        }
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_serialize(
    ros_message: *const c_void,
    type_support: *const rosidl_message_type_support_t,
    serialized_message: *mut rcl_serialized_message_t,
) -> rmw_ret_t {
    if ros_message.is_null() || type_support.is_null() || serialized_message.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let ts = match unsafe { MessageTypeSupport::new(type_support) } {
        Ok(ts) => ts,
        Err(_) => return RMW_RET_ERROR as _,
    };

    let serialized = unsafe { ts.serialize_message(ros_message) };

    // Allocate or resize the buffer in serialized_message
    unsafe {
        let msg = &mut *serialized_message;
        if msg.buffer_capacity < serialized.len() {
            // Need to reallocate
            if !msg.buffer.is_null() {
                // Drop the old buffer
                let _ = Vec::from_raw_parts(msg.buffer, msg.buffer_length, msg.buffer_capacity);
            }
            // Allocate new buffer
            let mut new_buffer = vec![0u8; serialized.len()];
            msg.buffer = new_buffer.as_mut_ptr();
            msg.buffer_capacity = new_buffer.len();
            std::mem::forget(new_buffer);
        }
        msg.buffer_length = serialized.len();
        std::ptr::copy_nonoverlapping(serialized.as_ptr(), msg.buffer, serialized.len());
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_deserialize(
    serialized_message: *const rcl_serialized_message_t,
    type_support: *const rosidl_message_type_support_t,
    ros_message: *mut c_void,
) -> rmw_ret_t {
    if serialized_message.is_null() || type_support.is_null() || ros_message.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let ts = match unsafe { MessageTypeSupport::new(type_support) } {
        Ok(ts) => ts,
        Err(_) => return RMW_RET_ERROR as _,
    };

    let data = unsafe {
        let msg = &*serialized_message;
        std::slice::from_raw_parts(msg.buffer, msg.buffer_length)
    };
    let data_vec = data.to_vec();

    let res = unsafe { ts.deserialize_message(&data_vec, ros_message) };
    if res {
        RMW_RET_OK as _
    } else {
        RMW_RET_ERROR as _
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_serialization_support_init(
    _serialization_support: *mut rmw_serialization_support_t,
    _allocator: *const rcl_allocator_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_borrow_loaned_message(
    publisher: *const rmw_publisher_t,
    type_support: *const rosidl_message_type_support_t,
    ros_message: *mut *mut c_void,
) -> rmw_ret_t {
    // Validate input arguments
    if publisher.is_null() || type_support.is_null() || ros_message.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Loaned messages are not currently supported in this implementation
    // Return RMW_RET_UNSUPPORTED to match the behavior of rmw_zenoh_cpp
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_return_loaned_message_from_publisher(
    publisher: *const rmw_publisher_t,
    loaned_message: *mut c_void,
) -> rmw_ret_t {
    // Validate input arguments
    if publisher.is_null() || loaned_message.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Loaned messages are not currently supported in this implementation
    // Return RMW_RET_UNSUPPORTED to match the behavior of rmw_zenoh_cpp
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_return_loaned_message_from_subscription(
    _subscription: *const rmw_subscription_t,
    _loaned_message: *mut c_void,
) -> rmw_ret_t {
    // Loaned messages are not currently supported in this implementation
    // Return RMW_RET_UNSUPPORTED to match the behavior of rmw_zenoh_cpp
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_serialized_message_size(
    _type_support: *const rosidl_message_type_support_t,
    _message_bounds: *const rosidl_message_bounds_t,
    _size: *mut usize,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publisher_event_init(
    rmw_event: *mut rmw_event_t,
    publisher: *const rmw_publisher_t,
    event_type: rmw_event_type_t,
) -> rmw_ret_t {
    if rmw_event.is_null() || publisher.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Get publisher implementation
    let pub_impl = match publisher.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Convert RMW event type to Zenoh event type
    let zenoh_event_type = match rmw_event_type_to_zenoh_event(event_type) {
        Some(t) => t,
        None => return RMW_RET_UNSUPPORTED as _,
    };

    // Verify this is a publisher event type
    match zenoh_event_type {
        ZenohEventType::LivelinessLost
        | ZenohEventType::OfferedDeadlineMissed
        | ZenohEventType::OfferedQosIncompatible
        | ZenohEventType::PublisherIncompatibleType
        | ZenohEventType::PublicationMatched => {}
        _ => return RMW_RET_INVALID_ARGUMENT as _,
    }

    // Get the events manager from the publisher
    let events_mgr = pub_impl.inner.events_mgr().clone();

    // Create the event handle
    let event_handle = Box::new(RmEventHandle::new(events_mgr, zenoh_event_type));

    // Fill the rmw_event_t structure
    unsafe {
        (*rmw_event).implementation_identifier =
            crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const std::os::raw::c_char;
        (*rmw_event).data = Box::into_raw(event_handle) as *mut std::os::raw::c_void;
        (*rmw_event).event_type = event_type;
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_event_init(
    rmw_event: *mut rmw_event_t,
    subscription: *const rmw_subscription_t,
    event_type: rmw_event_type_t,
) -> rmw_ret_t {
    if rmw_event.is_null() || subscription.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Get subscription implementation
    let sub_impl = match subscription.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Convert RMW event type to Zenoh event type
    let zenoh_event_type = match rmw_event_type_to_zenoh_event(event_type) {
        Some(t) => t,
        None => return RMW_RET_UNSUPPORTED as _,
    };

    // Verify this is a subscription event type
    match zenoh_event_type {
        ZenohEventType::LivelinessChanged
        | ZenohEventType::RequestedDeadlineMissed
        | ZenohEventType::RequestedQosIncompatible
        | ZenohEventType::MessageLost
        | ZenohEventType::SubscriptionIncompatibleType
        | ZenohEventType::SubscriptionMatched => {}
        _ => return RMW_RET_INVALID_ARGUMENT as _,
    }

    // Get the events manager from the subscription
    let events_mgr = sub_impl.inner.events_mgr().clone();

    // Create the event handle
    let event_handle = Box::new(RmEventHandle::new(events_mgr, zenoh_event_type));

    // Fill the rmw_event_t structure
    unsafe {
        (*rmw_event).implementation_identifier =
            crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const std::os::raw::c_char;
        (*rmw_event).data = Box::into_raw(event_handle) as *mut std::os::raw::c_void;
        (*rmw_event).event_type = event_type;
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_event_set_callback(
    event: *mut rmw_event_t,
    callback: rmw_event_callback_t,
    user_data: *mut c_void,
    _allocator: *const rcl_allocator_t,
) -> rmw_ret_t {
    if event.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    if unsafe { (*event).data }.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let rm_event_handle = unsafe { &mut *((*event).data as *mut RmEventHandle) };
    let user_data_ptr = user_data as usize;
    rm_event_handle.set_callback(move |change: i32| {
        if let Some(cb) = callback {
            let ud = user_data_ptr as *mut ::std::os::raw::c_void;
            unsafe { cb(ud as *const ::std::os::raw::c_void, change as usize) };
        }
    });

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_event_type_is_supported(event_type: rmw_event_type_t) -> bool {
    // Check if the event type can be converted to a ZenohEventType
    // This properly handles unsupported events like deadline and liveliness (matches rmw_zenoh_cpp)
    rmw_event_type_to_zenoh_event(event_type).is_some()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_event_fini(event: *mut rmw_event_t) -> rmw_ret_t {
    if event.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let rmw_event = unsafe { &mut *event };

    // Free the event handle if it exists
    if !rmw_event.data.is_null() {
        let _event_handle = unsafe { Box::from_raw(rmw_event.data as *mut RmEventHandle) };
        // Box will be dropped here, freeing the event handle
        rmw_event.data = std::ptr::null_mut();
    }

    rmw_event.implementation_identifier = std::ptr::null();
    rmw_event.event_type = 0;

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_set_on_new_message_callback(
    subscription: *mut rmw_subscription_t,
    callback: rmw_subscription_new_message_callback_t,
    user_data: *mut c_void,
) -> rmw_ret_t {
    if subscription.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match subscription.borrow_mut_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Set user_data first
    if let Ok(mut ud) = subscription_impl.callback_user_data.lock() {
        *ud = user_data as usize; // Store pointer as usize for thread safety
    }

    // Then set callback and check for unread messages
    if let Ok(mut cb) = subscription_impl.callback.lock() {
        if callback.is_some() {
            // Check if there are unread messages and invoke callback if needed
            if let Ok(mut unread) = subscription_impl.unread_count.lock() {
                if *unread > 0 {
                    // Invoke callback with unread count
                    unsafe {
                        if let Some(callback_fn) = callback {
                            callback_fn(user_data as *const std::ffi::c_void, *unread);
                        }
                    }
                    *unread = 0; // Reset unread count after notifying
                }
            }
        }
        *cb = callback;
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_service_set_on_new_request_callback(
    service: *mut rmw_service_t,
    callback: rmw_service_new_request_callback_t,
    user_data: *mut c_void,
) -> rmw_ret_t {
    if service.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let service_impl = match service.borrow_mut_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    if let Some(callback_fn) = callback {
        // Push events arrived before setting the executor callback (retroactive notification)
        if let Ok(mut unread) = service_impl.unread_count.lock() {
            if *unread > 0 {
                tracing::debug!(
                    "[rmw_service_set_on_new_request_callback] Invoking callback retroactively for {} unread requests",
                    *unread
                );
                unsafe {
                    callback_fn(user_data as *const std::ffi::c_void, *unread);
                }
                *unread = 0; // Reset unread count after notification
            }
        }
        // Store the new callback and user_data
        if let Ok(mut cb) = service_impl.callback.lock() {
            *cb = callback;
        }
        if let Ok(mut ud) = service_impl.callback_user_data.lock() {
            *ud = user_data as usize;
        }
    } else {
        // Callback is being cleared (set to None)
        if let Ok(mut cb) = service_impl.callback.lock() {
            *cb = None;
        }
        if let Ok(mut ud) = service_impl.callback_user_data.lock() {
            *ud = 0;
        }
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_client_set_on_new_response_callback(
    client: *mut rmw_client_t,
    callback: rmw_client_new_response_callback_t,
    user_data: *mut c_void,
) -> rmw_ret_t {
    if client.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let client_impl = match client.borrow_mut_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    if let Some(callback_fn) = callback {
        // Push events arrived before setting the executor callback (retroactive notification)
        if let Ok(mut unread) = client_impl.unread_count.lock() {
            if *unread > 0 {
                tracing::debug!(
                    "[rmw_client_set_on_new_response_callback] Invoking callback retroactively for {} unread responses",
                    *unread
                );
                unsafe {
                    callback_fn(user_data as *const std::ffi::c_void, *unread);
                }
                *unread = 0; // Reset unread count after notification
            }
        }
        // Store the new callback and user_data
        if let Ok(mut cb) = client_impl.callback.lock() {
            *cb = callback;
        }
        if let Ok(mut ud) = client_impl.callback_user_data.lock() {
            *ud = user_data as usize;
        }
    } else {
        // Callback is being cleared (set to None)
        if let Ok(mut cb) = client_impl.callback.lock() {
            *cb = None;
        }
        if let Ok(mut ud) = client_impl.callback_user_data.lock() {
            *ud = 0;
        }
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_gid_for_publisher(
    publisher: *const rmw_publisher_t,
    gid: *mut rmw_gid_t,
) -> rmw_ret_t {
    if publisher.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    if gid.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    if unsafe { (*publisher).implementation_identifier }
        != crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _
    {
        return RMW_RET_INCORRECT_RMW_IMPLEMENTATION as _;
    }

    let publisher_impl = match publisher.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let gid_array = ros_z::entity::endpoint_gid(&publisher_impl.entity);
    unsafe {
        (*gid).implementation_identifier = crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _;
        (*gid).data.copy_from_slice(&gid_array);
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_gid_for_client(
    client: *const rmw_client_t,
    gid: *mut rmw_gid_t,
) -> rmw_ret_t {
    if client.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    if gid.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    if unsafe { (*client).implementation_identifier }
        != crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _
    {
        return RMW_RET_INCORRECT_RMW_IMPLEMENTATION as _;
    }

    let client_impl = match client.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let gid_array = ros_z::entity::endpoint_gid(&client_impl.entity);
    unsafe {
        (*gid).implementation_identifier = crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _;
        (*gid).data.copy_from_slice(&gid_array);
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_compare_gids_equal(
    gid1: *const rmw_gid_t,
    gid2: *const rmw_gid_t,
    result: *mut bool,
) -> rmw_ret_t {
    if gid1.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let gid1_ref = unsafe { &*gid1 };
    if gid1_ref.implementation_identifier != crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _ {
        return RMW_RET_INCORRECT_RMW_IMPLEMENTATION as _;
    }

    if gid2.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let gid2_ref = unsafe { &*gid2 };
    if gid2_ref.implementation_identifier != crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _ {
        return RMW_RET_INCORRECT_RMW_IMPLEMENTATION as _;
    }

    if result.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        *result = gid1_ref.data == gid2_ref.data;
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_service_request_subscription_get_actual_qos(
    service: *const rmw_service_t,
    qos: *mut rmw_qos_profile_t,
) -> rmw_ret_t {
    if service.is_null() || qos.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let service_impl = match service.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    unsafe {
        *qos = service_impl.qos;
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_service_response_publisher_get_actual_qos(
    service: *const rmw_service_t,
    qos: *mut rmw_qos_profile_t,
) -> rmw_ret_t {
    // The same QoS profile is used for receiving requests and sending responses.
    rmw_service_request_subscription_get_actual_qos(service, qos)
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_service_server_is_available(
    node: *const rmw_node_t,
    client: *const rmw_client_t,
    is_available: *mut bool,
) -> rmw_ret_t {
    if node.is_null() || client.is_null() || is_available.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let client_impl = match client.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Get the client's service name
    let service_name = match client_impl.service_name.to_str() {
        Ok(name) => name,
        Err(_) => return RMW_RET_ERROR as _,
    };

    // Count servers for this service (namespace is included in service_name)
    let server_count = node_impl
        .inner
        .graph
        .count(ros_z::entity::EntityKind::Service, service_name);

    unsafe {
        *is_available = server_count > 0;
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_init_publisher_allocation(
    _type_support: *const rosidl_message_type_support_t,
    _message_bounds: *const rosidl_message_bounds_t,
    _allocation: *mut rmw_publisher_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_fini_publisher_allocation(
    _allocation: *mut rmw_publisher_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_init_subscription_allocation(
    _type_support: *const rosidl_message_type_support_t,
    _message_bounds: *const rosidl_message_bounds_t,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_fini_subscription_allocation(
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_dynamic_message(
    subscription: *const rmw_subscription_t,
    dynamic_message: *mut rcldynamic_message_t,
    taken: *mut bool,
    allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    // Call the _with_info version with a null message_info pointer
    rmw_take_dynamic_message_with_info(
        subscription,
        dynamic_message,
        taken,
        std::ptr::null_mut(),
        allocation,
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_dynamic_message_with_info(
    _subscription: *const rmw_subscription_t,
    _dynamic_message: *mut rcldynamic_message_t,
    _taken: *mut bool,
    _message_info: *mut rmw_message_info_t,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_feature_supported(feature: rmw_feature_t) -> bool {
    #[allow(non_upper_case_globals)]
    match feature {
        rmw_feature_e_RMW_FEATURE_MESSAGE_INFO_PUBLICATION_SEQUENCE_NUMBER => false,
        rmw_feature_e_RMW_FEATURE_MESSAGE_INFO_RECEPTION_SEQUENCE_NUMBER => false,
        rmw_feature_e_RMW_MIDDLEWARE_SUPPORTS_TYPE_DISCOVERY => true,
        rmw_feature_e_RMW_MIDDLEWARE_CAN_TAKE_DYNAMIC_MESSAGE => false,
        _ => false,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_set_log_severity(_severity: rmw_log_severity_t) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_test_isolation_start() -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_test_isolation_stop() -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

// NOTE: This function is not available in ROS 2 Jazzy.
// It was added in Rolling. Returning UNSUPPORTED for Jazzy compatibility.
#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_clients_info_by_service(
    _node: *const rmw_node_t,
    _allocator: *const rcl_allocator_t,
    _service_name: *const ::std::os::raw::c_char,
    _no_mangle: bool,
    _clients_info: *const ::std::os::raw::c_void, // Using c_void since type doesn't exist in Jazzy
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_client_names_and_types_by_node(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    node_name: *const std::os::raw::c_char,
    node_namespace: *const std::os::raw::c_char,
    service_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    if node.is_null()
        || allocator.is_null()
        || node_name.is_null()
        || node_namespace.is_null()
        || service_names_and_types.is_null()
    {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let target_node_name = match unsafe { std::ffi::CStr::from_ptr(node_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };
    let target_node_ns = match unsafe { std::ffi::CStr::from_ptr(node_namespace) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let node_key = (
        normalize_namespace(target_node_ns),
        target_node_name.to_string(),
    );

    // Check if the node exists in the graph
    if !node_impl.inner.graph.node_exists(node_key.clone()) {
        return RMW_RET_NODE_NAME_NON_EXISTENT as _;
    }

    let entities_and_types = node_impl
        .inner
        .graph
        .get_names_and_types_by_node(node_key, ros_z::entity::EntityKind::Client);

    let mut entity_map: std::collections::HashMap<String, Vec<String>> =
        std::collections::HashMap::new();
    for (name, type_name) in entities_and_types {
        entity_map.entry(name).or_default().push(type_name);
    }

    let entity_count = entity_map.len();

    unsafe {
        let ret =
            rmw_names_and_types_init(service_names_and_types, entity_count, allocator as *mut _);
        if ret != RMW_RET_OK as i32 {
            return RMW_RET_BAD_ALLOC as _;
        }

        for (index, (entity_name, type_names)) in entity_map.iter().enumerate() {
            let entity_cstr = match std::ffi::CString::new(entity_name.as_str()) {
                Ok(s) => s,
                Err(_) => {
                    rmw_names_and_types_fini(service_names_and_types);
                    return RMW_RET_ERROR as _;
                }
            };

            (*service_names_and_types)
                .names
                .data
                .add(index)
                .write(rcutils_strdup(entity_cstr.as_ptr(), *allocator));
            if (*service_names_and_types)
                .names
                .data
                .add(index)
                .read()
                .is_null()
            {
                rmw_names_and_types_fini(service_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            let ret = rcutils_string_array_init(
                (*service_names_and_types).types.add(index),
                type_names.len(),
                allocator,
            );
            if ret != 0 {
                rmw_names_and_types_fini(service_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            for (type_index, type_name) in type_names.iter().enumerate() {
                let type_cstr = match std::ffi::CString::new(type_name.as_str()) {
                    Ok(s) => s,
                    Err(_) => {
                        rmw_names_and_types_fini(service_names_and_types);
                        return RMW_RET_ERROR as _;
                    }
                };

                (*(*service_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .write(rcutils_strdup(type_cstr.as_ptr(), *allocator));
                if (*(*service_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .read()
                    .is_null()
                {
                    rmw_names_and_types_fini(service_names_and_types);
                    return RMW_RET_BAD_ALLOC as _;
                }
            }
        }
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_publisher_names_and_types_by_node(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    node_name: *const std::os::raw::c_char,
    node_namespace: *const std::os::raw::c_char,
    _no_demangle: bool,
    topic_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    if node.is_null()
        || allocator.is_null()
        || node_name.is_null()
        || node_namespace.is_null()
        || topic_names_and_types.is_null()
    {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let target_node_name = match unsafe { std::ffi::CStr::from_ptr(node_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };
    let target_node_ns = match unsafe { std::ffi::CStr::from_ptr(node_namespace) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let node_key = (
        normalize_namespace(target_node_ns),
        target_node_name.to_string(),
    );

    // Check if the node exists in the graph
    if !node_impl.inner.graph.node_exists(node_key.clone()) {
        return RMW_RET_NODE_NAME_NON_EXISTENT as _;
    }

    let entities_and_types = node_impl
        .inner
        .graph
        .get_names_and_types_by_node(node_key, ros_z::entity::EntityKind::Publisher);

    let mut entity_map: std::collections::HashMap<String, Vec<String>> =
        std::collections::HashMap::new();
    for (name, type_name) in entities_and_types {
        entity_map.entry(name).or_default().push(type_name);
    }

    let entity_count = entity_map.len();
    tracing::debug!(
        "rmw_get_publisher_names_and_types_by_node: target_node=({:?}, {:?}), entity_count={}, topics={:?}",
        target_node_ns,
        target_node_name,
        entity_count,
        entity_map.keys().collect::<Vec<_>>()
    );

    unsafe {
        let ret =
            rmw_names_and_types_init(topic_names_and_types, entity_count, allocator as *mut _);
        if ret != RMW_RET_OK as i32 {
            return RMW_RET_BAD_ALLOC as _;
        }

        for (index, (entity_name, type_names)) in entity_map.iter().enumerate() {
            let entity_cstr = match std::ffi::CString::new(entity_name.as_str()) {
                Ok(s) => s,
                Err(_) => {
                    rmw_names_and_types_fini(topic_names_and_types);
                    return RMW_RET_ERROR as _;
                }
            };

            (*topic_names_and_types)
                .names
                .data
                .add(index)
                .write(rcutils_strdup(entity_cstr.as_ptr(), *allocator));
            if (*topic_names_and_types)
                .names
                .data
                .add(index)
                .read()
                .is_null()
            {
                rmw_names_and_types_fini(topic_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            let ret = rcutils_string_array_init(
                (*topic_names_and_types).types.add(index),
                type_names.len(),
                allocator,
            );
            if ret != 0 {
                rmw_names_and_types_fini(topic_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            for (type_index, type_name) in type_names.iter().enumerate() {
                let type_cstr = match std::ffi::CString::new(type_name.as_str()) {
                    Ok(s) => s,
                    Err(_) => {
                        rmw_names_and_types_fini(topic_names_and_types);
                        return RMW_RET_ERROR as _;
                    }
                };

                (*(*topic_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .write(rcutils_strdup(type_cstr.as_ptr(), *allocator));
                if (*(*topic_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .read()
                    .is_null()
                {
                    rmw_names_and_types_fini(topic_names_and_types);
                    return RMW_RET_BAD_ALLOC as _;
                }
            }
        }
    }

    RMW_RET_OK as _
}

// NOTE: This function is not available in ROS 2 Jazzy.
// It was added in Rolling. Returning UNSUPPORTED for Jazzy compatibility.
#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_servers_info_by_service(
    _node: *const rmw_node_t,
    _allocator: *const rcl_allocator_t,
    _service_name: *const ::std::os::raw::c_char,
    _no_mangle: bool,
    _servers_info: *const ::std::os::raw::c_void, // Using c_void since type doesn't exist in Jazzy
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_service_names_and_types_by_node(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    node_name: *const std::os::raw::c_char,
    node_namespace: *const std::os::raw::c_char,
    service_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    if node.is_null()
        || allocator.is_null()
        || node_name.is_null()
        || node_namespace.is_null()
        || service_names_and_types.is_null()
    {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let target_node_name = match unsafe { std::ffi::CStr::from_ptr(node_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };
    let target_node_ns = match unsafe { std::ffi::CStr::from_ptr(node_namespace) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let node_key = (
        normalize_namespace(target_node_ns),
        target_node_name.to_string(),
    );

    // Check if the node exists in the graph
    if !node_impl.inner.graph.node_exists(node_key.clone()) {
        return RMW_RET_NODE_NAME_NON_EXISTENT as _;
    }

    let entities_and_types = node_impl
        .inner
        .graph
        .get_names_and_types_by_node(node_key, ros_z::entity::EntityKind::Service);

    let mut entity_map: std::collections::HashMap<String, Vec<String>> =
        std::collections::HashMap::new();
    for (name, type_name) in entities_and_types {
        entity_map.entry(name).or_default().push(type_name);
    }

    let entity_count = entity_map.len();

    unsafe {
        let ret =
            rmw_names_and_types_init(service_names_and_types, entity_count, allocator as *mut _);
        if ret != RMW_RET_OK as i32 {
            return RMW_RET_BAD_ALLOC as _;
        }

        for (index, (entity_name, type_names)) in entity_map.iter().enumerate() {
            let entity_cstr = match std::ffi::CString::new(entity_name.as_str()) {
                Ok(s) => s,
                Err(_) => {
                    rmw_names_and_types_fini(service_names_and_types);
                    return RMW_RET_ERROR as _;
                }
            };

            (*service_names_and_types)
                .names
                .data
                .add(index)
                .write(rcutils_strdup(entity_cstr.as_ptr(), *allocator));
            if (*service_names_and_types)
                .names
                .data
                .add(index)
                .read()
                .is_null()
            {
                rmw_names_and_types_fini(service_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            let ret = rcutils_string_array_init(
                (*service_names_and_types).types.add(index),
                type_names.len(),
                allocator,
            );
            if ret != 0 {
                rmw_names_and_types_fini(service_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            for (type_index, type_name) in type_names.iter().enumerate() {
                let type_cstr = match std::ffi::CString::new(type_name.as_str()) {
                    Ok(s) => s,
                    Err(_) => {
                        rmw_names_and_types_fini(service_names_and_types);
                        return RMW_RET_ERROR as _;
                    }
                };

                (*(*service_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .write(rcutils_strdup(type_cstr.as_ptr(), *allocator));
                if (*(*service_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .read()
                    .is_null()
                {
                    rmw_names_and_types_fini(service_names_and_types);
                    return RMW_RET_BAD_ALLOC as _;
                }
            }
        }
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_subscriptions_info_by_topic(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    topic_name: *const std::os::raw::c_char,
    _no_mangle: bool,
    subscriptions_info: *mut rmw_topic_endpoint_info_array_t,
) -> rmw_ret_t {
    if node.is_null() || allocator.is_null() || topic_name.is_null() || subscriptions_info.is_null()
    {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let topic_str = match unsafe { std::ffi::CStr::from_ptr(topic_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Get subscriptions for this topic
    let subscriptions = node_impl
        .inner
        .graph
        .get_entities_by_topic(ros_z::entity::EntityKind::Subscription, topic_str);

    let count = subscriptions.len();

    // Allocate array
    unsafe {
        let ret = rmw_topic_endpoint_info_array_init_with_size(
            subscriptions_info,
            count,
            allocator as *mut _,
        );
        if ret != RMW_RET_OK as i32 {
            return RMW_RET_BAD_ALLOC as _;
        }

        // Populate subscription info
        for (i, entity) in subscriptions.iter().enumerate() {
            // Extract endpoint entity from Entity enum
            let endpoint = match entity.as_ref() {
                ros_z::entity::Entity::Endpoint(ep) => ep,
                _ => continue, // Skip non-endpoint entities
            };

            // Convert entity to endpoint info
            let endpoint_info = (*subscriptions_info).info_array.add(i);

            // Set node name
            let node_name_cstr = match std::ffi::CString::new(endpoint.node.name.as_str()) {
                Ok(s) => s,
                Err(_) => {
                    rmw_topic_endpoint_info_array_fini(subscriptions_info, allocator as *mut _);
                    return RMW_RET_ERROR as _;
                }
            };
            (*endpoint_info).node_name = rcutils_strdup(node_name_cstr.as_ptr(), *allocator);

            // Set node namespace
            let node_ns = if endpoint.node.namespace.is_empty() {
                "/"
            } else {
                &endpoint.node.namespace
            };
            let node_ns_cstr = match std::ffi::CString::new(node_ns) {
                Ok(s) => s,
                Err(_) => {
                    rmw_topic_endpoint_info_array_fini(subscriptions_info, allocator as *mut _);
                    return RMW_RET_ERROR as _;
                }
            };
            (*endpoint_info).node_namespace = rcutils_strdup(node_ns_cstr.as_ptr(), *allocator);

            // Set topic type
            if let Some(ref type_info) = endpoint.type_info {
                let type_cstr = match std::ffi::CString::new(type_info.name.as_str()) {
                    Ok(s) => s,
                    Err(_) => {
                        rmw_topic_endpoint_info_array_fini(subscriptions_info, allocator as *mut _);
                        return RMW_RET_ERROR as _;
                    }
                };
                (*endpoint_info).topic_type = rcutils_strdup(type_cstr.as_ptr(), *allocator);
                (*endpoint_info).topic_type_hash = rosidl_type_hash_t {
                    version: type_info.hash.version,
                    value: type_info.hash.value,
                };
            } else {
                (*endpoint_info).topic_type = std::ptr::null();
            }

            // Set endpoint type
            (*endpoint_info).endpoint_type = rmw_endpoint_type_e_RMW_ENDPOINT_SUBSCRIPTION;

            // Set GID - endpoint_gid is just a [u8; 16] array
            let gid_bytes = endpoint.id.to_ne_bytes();
            let mut gid_data = [0u8; 16];
            let copy_len = std::cmp::min(std::mem::size_of::<usize>(), 16);
            gid_data[..copy_len].copy_from_slice(&gid_bytes[..copy_len]);
            (*endpoint_info).endpoint_gid = gid_data;

            // Set QoS profile - convert from protocol QoS to ros_z QoS to rmw QoS
            let ros_z_qos = ros_z::qos::QosProfile {
                reliability: match endpoint.qos.reliability {
                    ros_z_protocol::qos::QosReliability::Reliable => {
                        ros_z::qos::QosReliability::Reliable
                    }
                    ros_z_protocol::qos::QosReliability::BestEffort => {
                        ros_z::qos::QosReliability::BestEffort
                    }
                },
                durability: match endpoint.qos.durability {
                    ros_z_protocol::qos::QosDurability::TransientLocal => {
                        ros_z::qos::QosDurability::TransientLocal
                    }
                    ros_z_protocol::qos::QosDurability::Volatile => {
                        ros_z::qos::QosDurability::Volatile
                    }
                },
                history: match endpoint.qos.history {
                    ros_z_protocol::qos::QosHistory::KeepLast(depth) => {
                        ros_z::qos::QosHistory::from_depth(depth)
                    }
                    ros_z_protocol::qos::QosHistory::KeepAll => ros_z::qos::QosHistory::KeepAll,
                },
                ..Default::default()
            };
            (*endpoint_info).qos_profile = crate::qos::ros_z_qos_to_rmw_qos(&ros_z_qos);
        }
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_zenoh_get_session(_context: *const rmw_context_t) -> *const c_void {
    todo!()
}
