#![allow(unused)]

use crate::impl_has_impl_ptr;
use crate::msg::RosMessage;
use crate::node::NodeImpl;
use crate::rclz_try;
use crate::ros::*;
use crate::traits::{BorrowImpl, OwnImpl};
use crate::type_support::MessageTypeSupport;
use crate::utils::Notifier;
use crate::utils::str_from_ptr;
use crate::traits::Waitable;
use ros_z::{
    Builder,
    entity::TypeInfo,
    pubsub::{ZPub, ZSub},
};
use std::ffi::CString;
use std::sync::Arc;
use std::time::Duration;
use zenoh::{Result, sample::Sample};

pub struct PublisherImpl {
    pub(crate) inner: ZPub<RosMessage>,
    pub(crate) ts: MessageTypeSupport,
}

impl PublisherImpl {
    pub fn publish(&self, msg: *const crate::c_void) -> Result<()> {
        self.inner.publish(&RosMessage::new(msg, self.ts))
    }
    // pub fn publish(&self, msg: &RosMessage) -> Result<()> {
    //     self.zpub.publish(msg)
    // }

    pub fn publish_serialized_message(&self, msg: &[u8]) -> Result<()> {
        self.inner.publish_serialized_message(msg)
    }
}

pub struct SubscriptionImpl {
    pub(crate) inner: ZSub<RosMessage, Sample>,
    pub(crate) topic: CString,
    pub(crate) ts: MessageTypeSupport,
}

impl Waitable for SubscriptionImpl {
    fn is_ready(&self) -> bool {
        self.inner.queue.len() > 0
    }
}

impl_has_impl_ptr!(rcl_publisher_t, rcl_publisher_impl_t, PublisherImpl);
impl_has_impl_ptr!(
    rcl_subscription_t,
    rcl_subscription_impl_t,
    SubscriptionImpl
);

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_init(
    publisher: *mut rcl_publisher_t,
    node: *const rcl_node_t,
    type_support: *const rosidl_message_type_support_t,
    topic_name: *const ::std::os::raw::c_char,
    _options: *const rcl_publisher_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_publisher_init");

    let x = move || {
        let pub_impl = node
            .borrow_impl()?
            .new_pub(type_support, topic_name, _options)?;
        publisher.assign_impl(pub_impl)?;
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
    rclz_try! {
        publisher.borrow_impl()?.publish(ros_message)?;
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publish_serialized_message(
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
            .publish_serialized_message(&msg)?;
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_init(
    subscription: *mut rcl_subscription_t,
    node: *const rcl_node_t,
    type_support: *const rosidl_message_type_support_t,
    topic_name: *const ::std::os::raw::c_char,
    _options: *const rcl_subscription_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_subscription_init");
    let x = move || {
        let sub_impl = node
            .borrow_impl()?
            .new_sub(type_support, topic_name, _options)?;
        subscription.assign_impl(sub_impl)?;
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
    rclz_try! {
        std::mem::drop(publisher.own_impl()?);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_fini(
    subscription: *mut rcl_subscription_t,
    _node: *mut rcl_node_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_subscription_fini");
    rclz_try! {
        std::mem::drop(subscription.own_impl()?);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_take(
    subscription: *const rcl_subscription_t,
    ros_message: *mut crate::c_void,
    _message_info: *mut rmw_message_info_t,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_take");

    if subscription.is_null() || ros_message.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        let sub_impl = subscription.borrow_impl().unwrap();
        let Ok(sample) = sub_impl.inner.recv() else {
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
    subscription.borrow_impl().unwrap().topic.as_ptr()
}
