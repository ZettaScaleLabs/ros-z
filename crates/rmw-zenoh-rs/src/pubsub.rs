use std::ffi::CString;

use crate::rmw_impl_has_data_ptr;
use crate::ros::*;
use crate::traits::{BorrowData, Waitable};
use zenoh::{Result, sample::Sample};

/// Publisher implementation for RMW
pub struct PublisherImpl {
    pub inner: ros_z::pubsub::ZPub<crate::msg::RosMessage, crate::msg::RosSerdes>,
    pub ts: crate::type_support::MessageTypeSupport,
    pub topic: CString,
    pub options: rmw_publisher_options_t,
    pub qos: rmw_qos_profile_t,
    pub graph: std::sync::Arc<ros_z::graph::Graph>,
    pub entity: ros_z::entity::EndpointEntity,
}

impl PublisherImpl {
    pub fn publish(&self, msg: *const ::std::os::raw::c_void) -> Result<()> {
        let ros_msg = crate::msg::RosMessage::new(msg as *const crate::c_void, self.ts);
        self.inner.publish(&ros_msg)
    }

    pub fn publish_serialized_message(&self, msg: &[u8]) -> Result<()> {
        self.inner.publish_serialized(msg)
    }
}

/// Subscription implementation for RMW
pub struct SubscriptionImpl {
    pub inner: ros_z::pubsub::ZSub<crate::msg::RosMessage, Sample, crate::msg::RosSerdes>,
    pub ts: crate::type_support::MessageTypeSupport,
    pub topic: CString,
    pub options: rmw_subscription_options_t,
    pub qos: rmw_qos_profile_t,
    pub callback:
        std::sync::Arc<std::sync::Mutex<crate::ros::rmw_subscription_new_message_callback_t>>,
    pub callback_user_data: std::sync::Arc<std::sync::Mutex<usize>>, // Store pointer as usize for thread safety
    pub unread_count: std::sync::Arc<std::sync::Mutex<usize>>, // Track messages arrived before callback was set
    pub graph: std::sync::Arc<ros_z::graph::Graph>,
    pub entity: ros_z::entity::EndpointEntity,
    pub notifier: std::sync::Arc<crate::utils::Notifier>,
}

impl SubscriptionImpl {
    pub fn take(&self, ros_message: *mut std::os::raw::c_void, taken: *mut bool) -> Result<()> {
        unsafe {
            *taken = false;
        }
        let queue = self.inner.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Subscriber was built with callback, no queue available")
        })?;

        if let Some(sample) = queue.try_recv() {
            // Deserialize the sample payload into ros_message using ts
            // Assume the payload is CDR serialized
            let payload = sample.payload();
            let bytes = payload.to_bytes().to_vec();
            unsafe { self.ts.deserialize_message(&bytes, ros_message as *mut _) };
            unsafe {
                *taken = true;
            }
        }
        Ok(())
    }

    pub fn take_with_info(
        &self,
        ros_message: *mut std::os::raw::c_void,
        message_info: *mut rmw_message_info_t,
        taken: *mut bool,
    ) -> Result<()> {
        unsafe {
            *taken = false;
        }
        let queue = self.inner.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Subscriber was built with callback, no queue available")
        })?;
        if let Some(sample) = queue.try_recv() {
            // Deserialize the sample payload into ros_message using ts
            let payload = sample.payload();
            let bytes = payload.to_bytes().to_vec();
            unsafe { self.ts.deserialize_message(&bytes, ros_message as *mut _) };

            // Fill in message_info
            if !message_info.is_null() {
                unsafe {
                    // Extract timestamp from attachment
                    let source_timestamp = if let Some(attachment_bytes) = sample.attachment() {
                        if let Ok(attachment) =
                            ros_z::attachment::Attachment::try_from(attachment_bytes)
                        {
                            attachment.source_timestamp
                        } else {
                            0
                        }
                    } else {
                        0
                    };

                    // Get current time for received_timestamp
                    let received_timestamp = std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_nanos() as i64;

                    (*message_info).source_timestamp = source_timestamp;
                    (*message_info).received_timestamp = received_timestamp;
                    (*message_info).publication_sequence_number = 0;
                    (*message_info).reception_sequence_number = 0;

                    // Set publisher GID to zeros for now
                    // TODO: Extract proper GID from Zenoh sample
                    (*message_info).publisher_gid.data = [0u8; 16];
                    (*message_info).publisher_gid.implementation_identifier =
                        crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _;
                    (*message_info).from_intra_process = false;
                }
            }

            unsafe {
                *taken = true;
            }
        }
        Ok(())
    }

    pub fn take_serialized(
        &self,
        serialized_message: *mut rcl_serialized_message_t,
        message_info: *mut rmw_message_info_t,
        taken: *mut bool,
    ) -> Result<()> {
        unsafe {
            *taken = false;
        }
        let queue = self.inner.queue.as_ref().ok_or_else(|| {
            zenoh::Error::from("Subscriber was built with callback, no queue available")
        })?;
        if let Some(sample) = queue.try_recv() {
            let payload = sample.payload();
            let bytes = payload.to_bytes();

            unsafe {
                // Check if there's enough capacity
                if (*serialized_message).buffer_capacity < bytes.len() {
                    // Reallocate buffer if needed
                    if !(*serialized_message).buffer.is_null() {
                        // TODO: Use proper allocator from RMW context
                        let _ = Vec::from_raw_parts(
                            (*serialized_message).buffer,
                            (*serialized_message).buffer_length,
                            (*serialized_message).buffer_capacity,
                        );
                    }
                    let mut new_buffer = vec![0u8; bytes.len()];
                    (*serialized_message).buffer = new_buffer.as_mut_ptr();
                    (*serialized_message).buffer_capacity = new_buffer.len();
                    std::mem::forget(new_buffer);
                }

                // Copy bytes to buffer
                std::ptr::copy_nonoverlapping(
                    bytes.as_ptr(),
                    (*serialized_message).buffer,
                    bytes.len(),
                );
                (*serialized_message).buffer_length = bytes.len();
            }

            // Fill in message_info if provided
            if !message_info.is_null() {
                unsafe {
                    (*message_info).source_timestamp = 0;
                    (*message_info).received_timestamp = 0;
                    (*message_info).publication_sequence_number = 0;
                    (*message_info).reception_sequence_number = 0;
                    (*message_info).publisher_gid.data = [0u8; 16];
                    (*message_info).publisher_gid.implementation_identifier =
                        crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _;
                    (*message_info).from_intra_process = false;
                }
            }

            unsafe {
                *taken = true;
            }
        }
        Ok(())
    }
}

impl Waitable for SubscriptionImpl {
    fn is_ready(&self) -> bool {
        if let Some(queue) = self.inner.queue.as_ref() {
            !queue.is_empty()
        } else {
            false
        }
    }
}

rmw_impl_has_data_ptr!(rmw_publisher_t, rmw_publisher_impl_t, PublisherImpl);
rmw_impl_has_data_ptr!(
    rmw_subscription_t,
    rmw_subscription_impl_t,
    SubscriptionImpl
);

// RMW Publisher Functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_publish_serialized_message(
    publisher: *const rmw_publisher_t,
    serialized_message: *const rcl_serialized_message_t,
    _allocation: *mut rmw_publisher_allocation_t,
) -> rmw_ret_t {
    if publisher.is_null() || serialized_message.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let publisher_impl = match publisher.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let msg_slice = unsafe {
        std::slice::from_raw_parts(
            (*serialized_message).buffer,
            (*serialized_message).buffer_length,
        )
    };

    match publisher_impl.publish_serialized_message(msg_slice) {
        Ok(_) => RMW_RET_OK as _,
        Err(e) => {
            tracing::error!("Failed to publish serialized message: {}", e);
            RMW_RET_ERROR as _
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publish_loaned_message(
    _publisher: *const rmw_publisher_t,
    _ros_message: *mut ::std::os::raw::c_void,
    _allocation: *mut rmw_publisher_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publisher_count_matched_subscriptions(
    publisher: *const rmw_publisher_t,
    subscription_count: *mut usize,
) -> rmw_ret_t {
    if publisher.is_null() || subscription_count.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let publisher_impl = match publisher.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Get all subscription entities for this topic
    let topic_name = publisher_impl.topic.to_str().unwrap_or("");
    let entities = publisher_impl
        .graph
        .get_entities_by_topic(ros_z::entity::EntityKind::Subscription, topic_name);

    // Filter by QoS compatibility
    let pub_qos = &publisher_impl.qos;
    let count = entities
        .iter()
        .filter(|entity| {
            if let Some(endpoint) = entity.get_endpoint() {
                let sub_qos = crate::qos::ros_z_qos_to_rmw_qos(&endpoint.qos);
                crate::qos::qos_profiles_are_compatible(pub_qos, &sub_qos)
            } else {
                false
            }
        })
        .count();

    unsafe {
        *subscription_count = count;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publisher_get_actual_qos(
    publisher: *const rmw_publisher_t,
    qos: *mut rmw_qos_profile_t,
) -> rmw_ret_t {
    if publisher.is_null() || qos.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let publisher_impl = match publisher.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Return the original QoS profile with unspecified durations converted to infinite
    // Zenoh doesn't negotiate QoS like DDS, so the "actual" QoS is the requested QoS
    let mut actual_qos = publisher_impl.qos;

    // Convert zero/unspecified durations to RMW_DURATION_INFINITE (i32::MAX seconds)
    // This matches the expected behavior from ROS 2 tests
    const RMW_DURATION_INFINITE_SEC: u64 = i32::MAX as u64; // 2147483647

    if actual_qos.deadline.sec == 0 && actual_qos.deadline.nsec == 0 {
        actual_qos.deadline.sec = RMW_DURATION_INFINITE_SEC;
        actual_qos.deadline.nsec = 0;
    }
    if actual_qos.lifespan.sec == 0 && actual_qos.lifespan.nsec == 0 {
        actual_qos.lifespan.sec = RMW_DURATION_INFINITE_SEC;
        actual_qos.lifespan.nsec = 0;
    }
    if actual_qos.liveliness_lease_duration.sec == 0
        && actual_qos.liveliness_lease_duration.nsec == 0
    {
        actual_qos.liveliness_lease_duration.sec = RMW_DURATION_INFINITE_SEC;
        actual_qos.liveliness_lease_duration.nsec = 0;
    }

    // Convert liveliness SYSTEM_DEFAULT (0) or UNKNOWN to AUTOMATIC (1)
    if actual_qos.liveliness == rmw_qos_liveliness_policy_e_RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT
        || actual_qos.liveliness == rmw_qos_liveliness_policy_e_RMW_QOS_POLICY_LIVELINESS_UNKNOWN
    {
        actual_qos.liveliness = rmw_qos_liveliness_policy_e_RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    }

    unsafe {
        *qos = actual_qos;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publisher_assert_liveliness(publisher: *const rmw_publisher_t) -> rmw_ret_t {
    if publisher.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    // Assume liveliness is valid
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publisher_wait_for_all_acked(
    publisher: *const rmw_publisher_t,
    _wait_timeout: rmw_time_t,
) -> rmw_ret_t {
    if publisher.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    // Not tracking published data, return OK
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publisher_get_network_flow_endpoints(
    _publisher: *const rmw_publisher_t,
    _allocator: *const rcl_allocator_t,
    _endpoints: *mut rmw_network_flow_endpoint_array_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

// RMW Subscription Functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_with_info(
    subscription: *const rmw_subscription_t,
    ros_message: *mut ::std::os::raw::c_void,
    taken: *mut bool,
    message_info: *mut rmw_message_info_t,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    if subscription.is_null() || ros_message.is_null() || taken.is_null() || message_info.is_null()
    {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match subscription.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match subscription_impl.take_with_info(ros_message, message_info, taken) {
        Ok(_) => RMW_RET_OK as _,
        Err(_) => RMW_RET_ERROR as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_sequence(
    subscription: *const rmw_subscription_t,
    count: usize,
    message_sequence: *mut rmw_message_sequence_t,
    message_info_sequence: *mut rmw_message_info_sequence_t,
    taken: *mut usize,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    if subscription.is_null()
        || message_sequence.is_null()
        || message_info_sequence.is_null()
        || taken.is_null()
    {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match subscription.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    if count == 0 {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        if count > (*message_sequence).capacity || count > (*message_info_sequence).capacity {
            return RMW_RET_INVALID_ARGUMENT as _;
        }

        *taken = 0;
        while *taken < count {
            let mut one_taken = false;
            let msg_ptr = *(*message_sequence).data.add(*taken);
            let info_ptr =
                (*message_info_sequence).data.add(*taken) as *mut crate::ros::rmw_message_info_t;

            match subscription_impl.take_with_info(msg_ptr, info_ptr, &mut one_taken) {
                Ok(_) => {
                    if !one_taken {
                        break;
                    }
                    *taken += 1;
                }
                Err(_) => break,
            }
        }

        (*message_sequence).size = *taken;
        (*message_info_sequence).size = *taken;
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_serialized_message(
    subscription: *const rmw_subscription_t,
    serialized_message: *mut rcl_serialized_message_t,
    taken: *mut bool,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    if subscription.is_null() || serialized_message.is_null() || taken.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match subscription.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match subscription_impl.take_serialized(serialized_message, std::ptr::null_mut(), taken) {
        Ok(_) => RMW_RET_OK as _,
        Err(_) => RMW_RET_ERROR as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_serialized_message_with_info(
    subscription: *const rmw_subscription_t,
    serialized_message: *mut rcl_serialized_message_t,
    taken: *mut bool,
    message_info: *mut rmw_message_info_t,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    if subscription.is_null()
        || serialized_message.is_null()
        || taken.is_null()
        || message_info.is_null()
    {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match subscription.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match subscription_impl.take_serialized(serialized_message, message_info, taken) {
        Ok(_) => RMW_RET_OK as _,
        Err(_) => RMW_RET_ERROR as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_loaned_message(
    _subscription: *const rmw_subscription_t,
    _loaned_message: *mut *mut ::std::os::raw::c_void,
    _taken: *mut bool,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    // Loaned messages are not currently supported in this implementation
    // Return RMW_RET_UNSUPPORTED to match the behavior of rmw_zenoh_cpp
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_loaned_message_with_info(
    _subscription: *const rmw_subscription_t,
    _loaned_message: *mut *mut ::std::os::raw::c_void,
    _taken: *mut bool,
    _message_info: *mut rmw_message_info_t,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    // Loaned messages are not currently supported in this implementation
    // Return RMW_RET_UNSUPPORTED to match the behavior of rmw_zenoh_cpp
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_count_matched_publishers(
    subscription: *const rmw_subscription_t,
    publisher_count: *mut usize,
) -> rmw_ret_t {
    if subscription.is_null() || publisher_count.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match subscription.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Get all publisher entities for this topic
    let topic_name = subscription_impl.topic.to_str().unwrap_or("");
    let entities = subscription_impl
        .graph
        .get_entities_by_topic(ros_z::entity::EntityKind::Publisher, topic_name);

    // Filter by QoS compatibility
    let sub_qos = &subscription_impl.qos;
    let count = entities
        .iter()
        .filter(|entity| {
            if let Some(endpoint) = entity.get_endpoint() {
                let pub_qos = crate::qos::ros_z_qos_to_rmw_qos(&endpoint.qos);
                crate::qos::qos_profiles_are_compatible(&pub_qos, sub_qos)
            } else {
                false
            }
        })
        .count();

    unsafe {
        *publisher_count = count;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_get_actual_qos(
    subscription: *const rmw_subscription_t,
    qos: *mut rmw_qos_profile_t,
) -> rmw_ret_t {
    if subscription.is_null() || qos.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match subscription.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Return the original QoS profile with unspecified durations converted to infinite
    // Zenoh doesn't negotiate QoS like DDS, so the "actual" QoS is the requested QoS
    let mut actual_qos = subscription_impl.qos;

    // Convert zero/unspecified durations to RMW_DURATION_INFINITE (i32::MAX seconds)
    // This matches the expected behavior from ROS 2 tests
    const RMW_DURATION_INFINITE_SEC: u64 = i32::MAX as u64; // 2147483647

    if actual_qos.deadline.sec == 0 && actual_qos.deadline.nsec == 0 {
        actual_qos.deadline.sec = RMW_DURATION_INFINITE_SEC;
        actual_qos.deadline.nsec = 0;
    }
    if actual_qos.lifespan.sec == 0 && actual_qos.lifespan.nsec == 0 {
        actual_qos.lifespan.sec = RMW_DURATION_INFINITE_SEC;
        actual_qos.lifespan.nsec = 0;
    }
    if actual_qos.liveliness_lease_duration.sec == 0
        && actual_qos.liveliness_lease_duration.nsec == 0
    {
        actual_qos.liveliness_lease_duration.sec = RMW_DURATION_INFINITE_SEC;
        actual_qos.liveliness_lease_duration.nsec = 0;
    }

    // Convert liveliness SYSTEM_DEFAULT (0) or UNKNOWN to AUTOMATIC (1)
    if actual_qos.liveliness == rmw_qos_liveliness_policy_e_RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT
        || actual_qos.liveliness == rmw_qos_liveliness_policy_e_RMW_QOS_POLICY_LIVELINESS_UNKNOWN
    {
        actual_qos.liveliness = rmw_qos_liveliness_policy_e_RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    }

    unsafe {
        *qos = actual_qos;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_set_content_filter(
    _subscription: *const rmw_subscription_t,
    _content_filter: *const rmw_subscription_content_filter_options_t,
) -> rmw_ret_t {
    // Content filtering is not supported yet
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_get_content_filter(
    _subscription: *const rmw_subscription_t,
    _allocator: *const rcl_allocator_t,
    _content_filter: *mut rmw_subscription_content_filter_options_t,
) -> rmw_ret_t {
    // Content filtering is not supported yet
    RMW_RET_UNSUPPORTED as _
}
