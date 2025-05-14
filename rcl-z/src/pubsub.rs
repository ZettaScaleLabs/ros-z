use crate::msg::RosMessage;
use crate::node::NodeImpl;
use crate::ros::*;
use crate::type_support::TypeSupport;
use crate::utils::str_from_ptr;
use ros_z::pubsub::{Builder, ZPub, ZSub};
use std::sync::Arc;
use zenoh::{Result, sample::Sample};

pub struct PublisherImpl {
    zpub: ZPub<RosMessage>,
    ts: TypeSupport,
}

impl PublisherImpl {
    pub fn from_ptr<'b>(ptr: *const rcl_publisher_t) -> &'b Self {
        unsafe { &*((*ptr).impl_ as *const PublisherImpl) }
    }

    pub fn publish(&self, msg: &RosMessage) -> Result<()> {
        self.zpub.publish(msg)
    }

    pub fn publish_serialized_message(&self, msg: &[u8]) -> Result<()> {
        self.zpub.publish_serialized_message(msg)
    }
}

pub struct SubscriptionImpl {
    // zsub: zenoh::pubsub::Subscriber<()>,
    // zsub: zenoh::pubsub::Subscriber<()>,
    // pub rx: flume::Receiver<Sample>,
    pub zsub: ZSub<RosMessage, Sample>,
    pub cv: Arc<(parking_lot::Mutex<bool>, parking_lot::Condvar)>,
    ts: TypeSupport,
}

impl From<*const rcl_subscription_t> for &SubscriptionImpl {
    fn from(value: *const rcl_subscription_t) -> Self {
        unsafe { &*((*value).impl_ as *const SubscriptionImpl) }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_init(
    publisher: *mut rcl_publisher_t,
    node: *const rcl_node_t,
    type_support: *const rosidl_message_type_support_t,
    topic_name: *const ::std::os::raw::c_char,
    _options: *const rcl_publisher_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_publisher_init");

    let node_impl = unsafe { &*((*node).impl_ as *const NodeImpl) };
    let ts = TypeSupport::new(type_support);
    let keyexpr = str_from_ptr(topic_name).expect("Error found in topic name");
    let zpub = node_impl.create_pub(keyexpr).build().unwrap();

    let pub_impl = PublisherImpl { zpub, ts };
    unsafe {
        (*publisher).impl_ = Box::into_raw(Box::new(pub_impl)) as _;
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publish(
    publisher: *const rcl_publisher_t,
    ros_message: *const crate::c_void,
    _allocation: *mut rmw_publisher_allocation_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_publish");
    let pub_impl = PublisherImpl::from_ptr(publisher);
    pub_impl
        .publish(&RosMessage::new(ros_message, pub_impl.ts))
        .unwrap();
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publish_serialized_message(
    publisher: *const rcl_publisher_t,
    serialized_message: *const rcl_serialized_message_t,
    _allocation: *mut rmw_publisher_allocation_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_publish_serialized_message");
    let pub_impl = PublisherImpl::from_ptr(publisher);
    let msg = unsafe {
        std::slice::from_raw_parts(
            (*serialized_message).buffer,
            (*serialized_message).buffer_length,
        )
    };
    pub_impl.publish_serialized_message(&msg).unwrap();
    RCL_RET_OK as _
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

    // let (tx, rx) = flume::bounded(10);

    let pair = Arc::new((parking_lot::Mutex::new(false), parking_lot::Condvar::new()));
    let pair2 = pair.clone();

    let node_impl = unsafe { &*((*node).impl_ as *const NodeImpl) };
    let key_expr = str_from_ptr(topic_name).expect("Error found in topic_name");
    let zsub = node_impl
        .create_sub::<RosMessage>(key_expr)
        .post_deserialization()
        .build_with_notifier(move || {
            let &(ref lock, ref cvar) = &*pair2;
            let mut started = lock.lock();
            *started = true;
            cvar.notify_one();
        })
        .expect("Failed to declare_subscriber");

    let sub_impl = SubscriptionImpl {
        zsub,
        // rx,
        cv: pair,
        ts: TypeSupport::new(type_support),
    };
    unsafe {
        (*subscription).impl_ = Box::into_raw(Box::new(sub_impl)) as _;
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_fini(
    publisher: *mut rcl_publisher_t,
    _node: *mut rcl_node_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_publisher_fini");
    unsafe {
        std::mem::drop(Box::from_raw((*publisher).impl_ as *mut PublisherImpl));
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_fini(
    subscription: *mut rcl_subscription_t,
    _node: *mut rcl_node_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_subscription_fini");
    unsafe {
        std::mem::drop(Box::from_raw(
            (*subscription).impl_ as *mut SubscriptionImpl,
        ));
    }
    RCL_RET_OK as _
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
        // let sub_impl = &mut *((*subscription).impl_ as *mut SubscriptionImpl);
        let sub_impl: &SubscriptionImpl = From::from(subscription);
        let Ok(sample) = sub_impl.zsub.recv() else {
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
