#![allow(unused)]

use crate::impl_has_impl_ptr;
use crate::init::rcl_get_default_allocator;
use crate::msg::RosMessage;
use crate::node::NodeImpl;
use crate::rclz_try;
use crate::ros::*;
use std::ffi::c_char;
use crate::traits::Waitable;
use crate::traits::{BorrowImpl, OwnImpl};
use crate::type_support::MessageTypeSupport;
use crate::utils::Notifier;
use crate::utils::str_from_ptr;
use ros_z::{
    Builder,
    entity::TypeInfo,
    pubsub::{ZPub, ZSub},
};
use std::ffi::CString;
use std::ptr;
use std::sync::Arc;
use std::time::Duration;
use zenoh::{Result, sample::Sample};

pub struct PublisherImpl {
    pub(crate) inner: ZPub<RosMessage, crate::msg::RosSerdes>,
    pub(crate) ts: MessageTypeSupport,
    pub(crate) topic: CString,
    // This is needed by rcl_publisher_get_options
    pub(crate) options: rcl_publisher_options_t,
}

impl PublisherImpl {
    pub fn publish(&self, msg: *const crate::c_void) -> Result<()> {
        self.inner.publish(&RosMessage::new(msg, self.ts))
    }
    // pub fn publish(&self, msg: &RosMessage) -> Result<()> {
    //     self.zpub.publish(msg)
    // }

    pub fn publish_serialized_message(&self, msg: &[u8]) -> Result<()> {
        self.inner.publish_serialized(msg)
    }
}

pub struct SubscriptionImpl {
    pub(crate) inner: ZSub<RosMessage, Sample, crate::msg::RosSerdes>,
    pub(crate) topic: CString,
    pub(crate) ts: MessageTypeSupport,
    // This is needed by rcl_subscription_get_options
    pub(crate) options: rcl_subscription_options_t,
}

impl Waitable for SubscriptionImpl {
    fn is_ready(&self) -> bool {
        self.inner.queue.as_ref().map(|q| !q.is_empty()).unwrap_or(false)
    }
}

impl_has_impl_ptr!(rcl_publisher_t, rcl_publisher_impl_t, PublisherImpl);
impl_has_impl_ptr!(
    rcl_subscription_t,
    rcl_subscription_impl_t,
    SubscriptionImpl
);

#[unsafe(no_mangle)]
#[allow(unsafe_op_in_unsafe_fn)]
pub unsafe extern "C" fn rcl_publisher_init(
    publisher: *mut rcl_publisher_t,
    node: *const rcl_node_t,
    type_support: *const rosidl_message_type_support_t,
    topic_name: *const ::std::os::raw::c_char,
    _options: *const rcl_publisher_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_publisher_init");

    // Validate input parameters
    if publisher.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if node.is_null() {
        return RCL_RET_NODE_INVALID as _;
    }
    if type_support.is_null() {
        // TODO: For now, we don't support null type support
        // This would require a different code path or mock implementation
        tracing::warn!("Type support cannot be null");
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if topic_name.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if _options.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Validate allocator
    let options_ref = unsafe { &*_options };
    if options_ref.allocator.allocate.is_none() || options_ref.allocator.deallocate.is_none() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check if node is valid (initialized)
    if node.borrow_impl().is_err() {
        return RCL_RET_NODE_INVALID as _;
    }

    // Check if publisher is already initialized
    if publisher.borrow_impl().is_ok() {
        return RCL_RET_ALREADY_INIT as _;
    }

    let x = move || {
        let pub_impl = node
            .borrow_impl()?
            .new_pub(type_support, topic_name, _options)?;
        publisher.assign_impl(pub_impl)?;

        // Trigger graph guard condition for this node
        if let Ok(node_impl) = (node as *mut rcl_node_t).borrow_mut_impl() {
            node_impl.trigger_graph_guard_condition();
        }

        Result::Ok(())
    };
    rclz_try! { x()?; }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publish(
    publisher: *const rcl_publisher_t,
    ros_message: *const crate::c_void,
    _allocation: *mut rmw_publisher_allocation_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_publish");
    if publisher.is_null() {
        return RCL_RET_PUBLISHER_INVALID as _;
    }
    if ros_message.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    rclz_try! {
        publisher.borrow_impl()?.publish(ros_message)?;
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_publish_serialized_message(
    publisher: *const rcl_publisher_t,
    serialized_message: *const rcl_serialized_message_t,
    _allocation: *mut rmw_publisher_allocation_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_publish_serialized_message");
    let msg = unsafe {
        std::slice::from_raw_parts(
            (*serialized_message).buffer,
            (*serialized_message).buffer_length,
        )
    };
    rclz_try! {
        publisher
            .borrow_impl()?
            .publish_serialized_message(msg)?;
    }
}

#[unsafe(no_mangle)]
#[allow(unsafe_op_in_unsafe_fn)]
pub unsafe extern "C" fn rcl_subscription_init(
    subscription: *mut rcl_subscription_t,
    node: *const rcl_node_t,
    type_support: *const rosidl_message_type_support_t,
    topic_name: *const ::std::os::raw::c_char,
    options: *const rcl_subscription_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_subscription_init");

    // FIXME: Add tracing::warning to the following checks
    // Validate input parameters
    if subscription.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if node.is_null() {
        return RCL_RET_NODE_INVALID as _;
    }
    if type_support.is_null() {
        tracing::warn!("Type support cannot be null");
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if topic_name.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if options.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Validate allocator
    let options_ref = unsafe { &*options };
    if options_ref.allocator.allocate.is_none() || options_ref.allocator.deallocate.is_none() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check if node is valid (initialized)
    if node.borrow_impl().is_err() {
        return RCL_RET_NODE_INVALID as _;
    }

    // Check if subscription is already initialized
    if subscription.borrow_impl().is_ok() {
        return RCL_RET_ALREADY_INIT as _;
    }

    // Validate topic name
    unsafe {
        let mut validation_result: i32 = 0;
        let ret = crate::validate_topic_name::rcl_validate_topic_name(
            topic_name,
            &mut validation_result,
            ptr::null_mut(),
        );
        if ret != RCL_RET_OK as i32 {
            return ret;
        }
        if validation_result != crate::validate_topic_name::RCL_TOPIC_NAME_VALID {
            return RCL_RET_TOPIC_NAME_INVALID as _;
        }
    }

    let x = move || {
        let sub_impl = node
            .borrow_impl()?
            .new_sub(type_support, topic_name, options)?;
        subscription.assign_impl(sub_impl)?;

        // Trigger graph guard condition for this node
        if let Ok(node_impl) = (node as *mut rcl_node_t).borrow_mut_impl() {
            node_impl.trigger_graph_guard_condition();
        }

        Result::Ok(())
    };
    rclz_try! { x()?; }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_fini(
    publisher: *mut rcl_publisher_t,
    _node: *mut rcl_node_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_publisher_fini");

    // Validate input parameters
    if publisher.is_null() {
        return RCL_RET_PUBLISHER_INVALID as _;
    }
    if _node.is_null() {
        return RCL_RET_NODE_INVALID as _;
    }

    // Drop the data regardless of the pointer's condition.
    drop(publisher.own_impl());

    // Trigger graph guard condition for this node
    if let Ok(node_impl) = _node.borrow_mut_impl() {
        node_impl.trigger_graph_guard_condition();
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_fini(
    subscription: *mut rcl_subscription_t,
    _node: *mut rcl_node_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_subscription_fini");

    // Validate input parameters
    if subscription.is_null() {
        return RCL_RET_SUBSCRIPTION_INVALID as _;
    }
    if _node.is_null() {
        return RCL_RET_NODE_INVALID as _;
    }

    // Drop the data regardless of the pointer's condition.
    drop(subscription.own_impl());

    // Trigger graph guard condition for this node
    if let Ok(node_impl) = _node.borrow_mut_impl() {
        node_impl.trigger_graph_guard_condition();
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_take(
    subscription: *const rcl_subscription_t,
    ros_message: *mut crate::c_void,
    _message_info: *mut rmw_message_info_t,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_take");

    if ros_message.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        // Check if subscription is valid (properly initialized, including null check)
        if !rcl_subscription_is_valid(subscription) {
            return RCL_RET_SUBSCRIPTION_INVALID as _;
        }

        let sub_impl = match subscription.borrow_impl() {
            Ok(impl_) => impl_,
            Err(_) => return RCL_RET_SUBSCRIPTION_INVALID as _,
        };
        if !sub_impl.is_ready() {
            return RCL_RET_SUBSCRIPTION_TAKE_FAILED as _;
        }
        let Ok(sample) = sub_impl.inner.recv_serialized() else {
            return RCL_RET_ERROR as _;
        };

        let payload = sample.payload().to_bytes();
        // TODO: Can we avoid the contiguous here?
        sub_impl
            .ts
            .deserialize_message(&payload.to_vec(), ros_message);

        // TODO: check slices
        // sample.payload().slices().fold(0, |offset, slice| {
        //     std::ptr::copy_nonoverlapping(
        //         slice.as_ptr(),
        //         (*msg).data.data.add(offset),
        //         slice.len(),
        //     );
        //     offset + slice.len()
        // });

        // let payload = sample.payload().to_bytes();
        // std::ptr::copy_nonoverlapping(
        //     payload.as_ptr(),
        //     (*msg).data.data,
        //     payload.len(),
        // );

        RCL_RET_OK as _
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_take_loaned_message(
    subscription: *const rcl_subscription_t,
    loaned_message: *mut *mut crate::c_void,
    _message_info: *mut rmw_message_info_t,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_take_loaned_message");

    if subscription.is_null() || loaned_message.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check if loaned_message is already non-null (invalid state)
    unsafe {
        if !(*loaned_message).is_null() {
            return RCL_RET_INVALID_ARGUMENT as _;
        }
    }

    // For now, return unsupported as loaned messages are not implemented
    unsafe {
        *loaned_message = std::ptr::null_mut();
    }
    RCL_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_take_serialized_message(
    subscription: *const rcl_subscription_t,
    serialized_message: *mut rcl_serialized_message_t,
    _message_info: *mut rmw_message_info_t,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_take_serialized_message");

    if subscription.is_null() {
        return RCL_RET_SUBSCRIPTION_INVALID as _;
    }
    if serialized_message.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    match subscription.borrow_impl() {
        Ok(_) => {
            // Content filtering is not implemented
            RCL_RET_UNSUPPORTED as _
        }
        Err(_) => RCL_RET_SUBSCRIPTION_INVALID as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_get_rmw_handle(
    subscription: *const rcl_subscription_t,
) -> *mut rmw_subscription_t {
    if subscription.is_null() {
        return ptr::null_mut();
    }
    // Since the struct is opaque, return null for now
    ptr::null_mut()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_is_cft_enabled(subscription: *const rcl_subscription_t) -> bool {
    if subscription.is_null() {
        return false;
    }
    // Content filtering is not implemented
    false
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_content_filter_options_init(
    _subscription: *const rcl_subscription_t,
    _filter_expression: *const c_char,
    _expression_parameters_count: usize,
    _expression_parameters: *const *const c_char,
    _options: *mut rcl_subscription_content_filter_options_t,
) -> rcl_ret_t {
    // Content filtering is not implemented
    RCL_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_content_filter_options_fini(
    _subscription: *const rcl_subscription_t,
    _options: *mut rcl_subscription_content_filter_options_t,
) -> rcl_ret_t {
    // Content filtering is not implemented
    RCL_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_return_loaned_message_from_subscription(
    subscription: *const rcl_subscription_t,
    loaned_message: *mut crate::c_void,
) -> rcl_ret_t {
    tracing::trace!("rcl_return_loaned_message_from_subscription");

    if subscription.is_null() || subscription.borrow_impl().is_err() {
        return RCL_RET_SUBSCRIPTION_INVALID as _;
    }

    if loaned_message.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // TODO: Implement loaned message return
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_is_valid(subscription: *const rcl_subscription_t) -> bool {
    subscription.borrow_impl().is_ok()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_can_loan_messages(
    subscription: *const rcl_subscription_t,
) -> bool {
    false
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_get_topic_name(
    subscription: *const rcl_subscription_t,
) -> *const ::std::os::raw::c_char {
    match subscription.borrow_impl() {
        Ok(impl_) => impl_.topic.as_ptr(),
        Err(_) => std::ptr::null(),
    }
}

// Publisher utility functions

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_publisher() -> rcl_publisher_t {
    // TODO: Implement proper zero initialization
    unsafe { std::mem::zeroed() }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_get_default_options() -> rcl_publisher_options_t {
    // TODO: Implement proper default options with environment variable support
    let mut options = unsafe { std::mem::zeroed::<rcl_publisher_options_t>() };

    // Set default allocator
    options.allocator = rcl_get_default_allocator();

    // Check ROS_DISABLE_LOANED_MESSAGES environment variable (Jazzy+ only)
    #[cfg(not(ros_humble))]
    {
        if let Ok(disable_loaned) = std::env::var("ROS_DISABLE_LOANED_MESSAGES") {
            options.disable_loaned_message = matches!(disable_loaned.as_str(), "1");
        }
    }

    options
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_is_valid(publisher: *const rcl_publisher_t) -> bool {
    if publisher.is_null() {
        return false;
    }
    publisher.borrow_impl().is_ok()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_is_valid_except_context(publisher: *const rcl_publisher_t) -> bool {
    // TODO: Implement context-independent validation
    rcl_publisher_is_valid(publisher)
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_get_topic_name(
    publisher: *const rcl_publisher_t,
) -> *const ::std::os::raw::c_char {
    if publisher.is_null() {
        return std::ptr::null();
    }
    match publisher.borrow_impl() {
        Ok(impl_) => impl_.topic.as_ptr(),
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_get_options(
    publisher: *const rcl_publisher_t,
) -> *const rcl_publisher_options_t {
    if publisher.is_null() {
        return std::ptr::null();
    }
    match publisher.borrow_impl() {
        Ok(impl_) => &impl_.options as *const _,
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_get_actual_qos(
    publisher: *const rcl_publisher_t,
) -> *const rmw_qos_profile_t {
    if publisher.is_null() {
        return std::ptr::null();
    }
    match publisher.borrow_impl() {
        Ok(impl_) => &impl_.options.qos as *const _,
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_get_subscription_count(
    publisher: *const rcl_publisher_t,
    count: *mut usize,
) -> rcl_ret_t {
    if publisher.is_null() {
        return RCL_RET_PUBLISHER_INVALID as i32;
    }
    if count.is_null() {
        return RCL_RET_INVALID_ARGUMENT as i32;
    }
    // For Zenoh RMW implementation, we don't track matched subscriptions
    // in the traditional sense, so return 0
    unsafe {
        *count = 0;
    }
    RCL_RET_OK as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_get_publisher_count(
    subscription: *const rcl_subscription_t,
    count: *mut usize,
) -> rcl_ret_t {
    if subscription.is_null() {
        return RCL_RET_SUBSCRIPTION_INVALID as i32;
    }
    if count.is_null() {
        return RCL_RET_INVALID_ARGUMENT as i32;
    }
    match subscription.borrow_impl() {
        Ok(_) => {
            // For Zenoh RMW implementation, we don't track matched publishers
            // in the traditional sense, so return 0
            unsafe {
                *count = 0;
            }
            RCL_RET_OK as i32
        }
        Err(_) => RCL_RET_SUBSCRIPTION_INVALID as i32,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_can_loan_messages(publisher: *const rcl_publisher_t) -> bool {
    if publisher.is_null() {
        return false;
    }
    // TODO: Implement loaned message support check
    false
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_assert_liveliness(publisher: *const rcl_publisher_t) -> rcl_ret_t {
    if publisher.is_null() {
        return RCL_RET_PUBLISHER_INVALID as _;
    }
    // TODO: Implement liveliness assertion
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_wait_for_all_acked(
    publisher: *const rcl_publisher_t,
    _timeout: i64,
) -> rcl_ret_t {
    if publisher.is_null() {
        return RCL_RET_PUBLISHER_INVALID as _;
    }
    // TODO: Implement wait for all acked
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_borrow_loaned_message(
    publisher: *const rcl_publisher_t,
    _type_support: *const rosidl_message_type_support_t,
    ros_message: *mut *mut crate::c_void,
) -> rcl_ret_t {
    if publisher.is_null() {
        return RCL_RET_PUBLISHER_INVALID as _;
    }
    if ros_message.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    // Loaned messages are not supported in this implementation
    RCL_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publish_loaned_message(
    publisher: *const rcl_publisher_t,
    ros_message: *mut crate::c_void,
    _allocation: *mut rcl_publisher_allocation_t,
) -> rcl_ret_t {
    if publisher.is_null() {
        return RCL_RET_PUBLISHER_INVALID as _;
    }
    if ros_message.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    // Loaned messages are not supported in this implementation
    RCL_RET_UNSUPPORTED as _
}

// Subscriber utility functions

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_subscription() -> rcl_subscription_t {
    // TODO: Implement proper zero initialization
    unsafe { std::mem::zeroed() }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_get_default_options() -> rcl_subscription_options_t {
    let mut options = unsafe { std::mem::zeroed::<rcl_subscription_options_t>() };

    // Set default allocator
    options.allocator = rcl_get_default_allocator();

    // Check ROS_DISABLE_LOANED_MESSAGES environment variable (Jazzy+ only)
    // Default is to disable loaned messages (true)
    #[cfg(not(ros_humble))]
    {
        options.disable_loaned_message = match std::env::var("ROS_DISABLE_LOANED_MESSAGES") {
            Ok(val) if val == "0" => false,
            _ => true, // Default to true for any other value or if not set
        };
    }

    options
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_is_valid_except_context(
    subscription: *const rcl_subscription_t,
) -> bool {
    subscription.borrow_impl().is_ok()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_get_options(
    subscription: *const rcl_subscription_t,
) -> *const rcl_subscription_options_t {
    match subscription.borrow_impl() {
        Ok(impl_) => &impl_.options,
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_get_actual_qos(
    subscription: *const rcl_subscription_t,
) -> *const rmw_qos_profile_t {
    if subscription.is_null() {
        return std::ptr::null();
    }
    match subscription.borrow_impl() {
        Ok(impl_) => &impl_.options.qos as *const _,
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_assert_liveliness(
    subscription: *const rcl_subscription_t,
) -> rcl_ret_t {
    if subscription.is_null() {
        return RCL_RET_SUBSCRIPTION_INVALID as _;
    }
    // TODO: Implement liveliness assertion for subscribers
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_wait_for_data(
    subscription: *const rcl_subscription_t,
    wait_set: *mut rcl_wait_set_t,
    index: *mut usize,
) -> rcl_ret_t {
    if subscription.is_null() {
        return RCL_RET_SUBSCRIPTION_INVALID as _;
    }
    // TODO: Implement wait for data
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_take_sequence(
    subscription: *const rcl_subscription_t,
    count: usize,
    ros_messages: *mut *mut crate::c_void,
    message_infos: *mut rmw_message_info_t,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_take_sequence");

    if subscription.is_null() || ros_messages.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Sequence taking is not implemented
    RCL_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_subscription_set_content_filter(
    subscription: *const rcl_subscription_t,
    options: *const rcl_subscription_content_filter_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_subscription_set_content_filter");

    if subscription.is_null() || subscription.borrow_impl().is_err() {
        return RCL_RET_SUBSCRIPTION_INVALID as _;
    }

    if options.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Content filtering is not implemented
    RCL_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_subscription_get_content_filter(
    subscription: *const rcl_subscription_t,
    options: *mut rcl_subscription_content_filter_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_subscription_get_content_filter");

    if subscription.is_null() || subscription.borrow_impl().is_err() {
        return RCL_RET_SUBSCRIPTION_INVALID as _;
    }

    if options.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Content filtering is not implemented
    RCL_RET_UNSUPPORTED as _
}
