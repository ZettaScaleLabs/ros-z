#![allow(unused)]
use std::ffi::CString;
use std::ops::Deref;
use std::str::FromStr;
use std::sync::Arc;

use crate::context::ContextImpl;
use crate::impl_has_impl_ptr;
use crate::msg::RosMessage;
use crate::pubsub::PublisherImpl;
use crate::pubsub::SubscriptionImpl;
use crate::rclz_try;
use crate::ros::*;
use crate::service::ClientImpl;
use crate::service::ServiceImpl;
use crate::traits::{BorrowImpl, OwnImpl};
use crate::type_support::MessageTypeSupport;
use crate::type_support::ServiceTypeSupport;
use crate::utils::NOT_SUPPORTED_CSTR;
use crate::utils::Notifier;
use crate::utils::str_from_ptr;
use ros_z::graph::Graph;
use ros_z::Builder;
use ros_z::entity::TypeInfo;
use ros_z::node::ZNode;
use zenoh::Result;

pub struct NodeImpl {
    pub(crate) inner: ZNode,
    pub(crate) name: CString,
    pub(crate) namespace: CString,
    pub(crate) notifier: Arc<Notifier>,
}

impl NodeImpl {
    pub fn new_sub(
        &self,
        type_support: *const rosidl_message_type_support_t,
        topic_name: *const ::std::os::raw::c_char,
        _options: *const rcl_subscription_options_t,
    ) -> Result<SubscriptionImpl> {
        // Message type support
        let ts = MessageTypeSupport::new(type_support);
        let topic = str_from_ptr(topic_name)?;
        let type_info = TypeInfo::new(&ts.get_type_prefix(), ts.get_type_hash());

        // Notification
        let notifier = self.notifier.clone();
        let notify_callback = move || notifier.notify_all();

        let zsub = self
            .inner
            .create_sub::<RosMessage>(topic)
            .with_type_info(type_info)
            .with_serdes::<crate::msg::RosSerdes>()
            .build_with_notifier(notify_callback)?;

        // TODO: get the qualified topic
        // Cache the topic c_str
        let topic_cstr = CString::new(zsub.entity.topic.clone()).unwrap();

        Ok(SubscriptionImpl {
            inner: zsub,
            ts,
            topic: topic_cstr,
        })
    }

    pub fn new_pub(
        &self,
        type_support: *const rosidl_message_type_support_t,
        topic_name: *const ::std::os::raw::c_char,
        _options: *const rcl_publisher_options_t,
    ) -> Result<PublisherImpl> {
        // Message type support
        let ts = MessageTypeSupport::new(type_support);
        let type_info = TypeInfo::new(&ts.get_type_prefix(), ts.get_type_hash());
        let topic = str_from_ptr(topic_name)?;

        let zpub = self
            .inner
            .create_pub(topic)
            .with_type_info(type_info)
            .with_serdes::<crate::msg::RosSerdes>()
            .build()?;
        Ok(PublisherImpl { inner: zpub, ts })
    }

    pub fn new_client(
        &self,
        type_support: *const rosidl_service_type_support_t,
        service_name: *const ::std::os::raw::c_char,
        _options: *const rcl_client_options_t,
    ) -> Result<ClientImpl> {
        let ts = ServiceTypeSupport::new(type_support);
        let topic = str_from_ptr(service_name)?;
        let zcli = self
            .inner
            .create_client(topic)
            .with_type_info(ts.get_type_info())
            .build()?;
        Ok(ClientImpl {
            inner: zcli,
            ts,
            notifier: self.notifier.clone(),
        })
    }

    pub fn new_service(
        &self,
        type_support: *const rosidl_service_type_support_t,
        service_name: *const ::std::os::raw::c_char,
        _options: *const rcl_service_options_t,
    ) -> Result<ServiceImpl> {
        let ts = ServiceTypeSupport::new(type_support);

        let topic = str_from_ptr(service_name)?;
        let notifier = self.notifier.clone();
        let notify_callback = move || {
            notifier.notify_all();
        };
        let zsrv = self
            .inner
            .create_service(topic)
            .with_type_info(ts.get_type_info())
            .build_with_notifier(notify_callback)?;
        Ok(ServiceImpl { inner: zsrv, ts })
    }

    pub fn graph(&self) -> &Arc<Graph> {
        &self.inner.graph
    }
}

impl_has_impl_ptr!(rcl_node_t, rcl_node_impl_t, NodeImpl);

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_init(
    node: *mut rcl_node_t,
    name: *const ::std::os::raw::c_char,
    namespace_: *const ::std::os::raw::c_char,
    context: *mut rcl_context_t,
    _options: *const rcl_node_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_node_init");

    unsafe {
        (*node).context = context;
    }
    let x = move || {
        let node_impl = context
            .borrow_impl()?
            .new_node(name, namespace_, context, _options)?;
        node.assign_impl(node_impl)?;
        zenoh::Result::Ok(())
    };

    rclz_try! {x()?;}
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_fini(node: *mut rcl_node_t) -> rcl_ret_t {
    tracing::trace!("rcl_node_fini");
    rclz_try! {
        std::mem::drop(node.own_impl()?);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_logger_name(
    _node: *const rcl_node_t,
) -> *const ::std::os::raw::c_char {
    NOT_SUPPORTED_CSTR.as_ptr()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_fully_qualified_name(
    _node: *const rcl_node_t,
) -> *const ::std::os::raw::c_char {
    NOT_SUPPORTED_CSTR.as_ptr()
}

// #[unsafe(no_mangle)]
// pub extern "C"  fn rcl_node_get_default_options() -> rcl_node_options_t {
//     let mut args = rcl_arguments_s::default();
//     args.impl_ = Box::into_raw(Box::new(())) as _;
//     let mut opts = rcl_node_options_t::default();
//     opts.arguments = args;
//     opts
// }

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_namespace(node: *const rcl_node_t) -> *const ::std::os::raw::c_char {
    node.borrow_impl().unwrap().namespace.as_ptr()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_name(node: *const rcl_node_t) -> *const ::std::os::raw::c_char {
    node.borrow_impl().unwrap().name.as_ptr()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_expand_topic_name(
    input_topic_name: *const ::std::os::raw::c_char,
    node_name: *const ::std::os::raw::c_char,
    node_namespace: *const ::std::os::raw::c_char,
    substitutions: *const rcutils_string_map_t,
    allocator: rcl_allocator_t,
    output_topic_name: *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_remap_topic_name(
    local_arguments: *const rcl_arguments_t,
    global_arguments: *const rcl_arguments_t,
    topic_name: *const ::std::os::raw::c_char,
    node_name: *const ::std::os::raw::c_char,
    node_namespace: *const ::std::os::raw::c_char,
    allocator: rcl_allocator_t,
    output_name: *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_remap_service_name(
    local_arguments: *const rcl_arguments_t,
    global_arguments: *const rcl_arguments_t,
    service_name: *const ::std::os::raw::c_char,
    node_name: *const ::std::os::raw::c_char,
    node_namespace: *const ::std::os::raw::c_char,
    allocator: rcl_allocator_t,
    output_name: *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    RCL_RET_OK as _
}
#[unsafe(no_mangle)]
pub extern "C" fn rcl_remap_node_name(
    local_arguments: *const rcl_arguments_t,
    global_arguments: *const rcl_arguments_t,
    node_name: *const ::std::os::raw::c_char,
    allocator: rcl_allocator_t,
    output_name: *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    RCL_RET_OK as _
}
#[unsafe(no_mangle)]
pub extern "C" fn rcl_remap_node_namespace(
    local_arguments: *const rcl_arguments_t,
    global_arguments: *const rcl_arguments_t,
    node_name: *const ::std::os::raw::c_char,
    allocator: rcl_allocator_t,
    output_namespace: *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    RCL_RET_OK as _
}
#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_resolve_name(
    node: *const rcl_node_t,
    input_name: *const ::std::os::raw::c_char,
    allocator: rcl_allocator_t,
    is_service: bool,
    only_expand: bool,
    output_name: *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    let cstr = CString::new("skip_rcl_node_resolve_name").unwrap();
    unsafe {
        *output_name = cstr.into_raw();
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_is_valid(node: *const rcl_node_t) -> bool {
    node.borrow_impl().is_ok()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_type_description_service_init(
    service: *mut rcl_service_t,
    node: *const rcl_node_t,
) -> rcl_ret_t {
    let type_support = unsafe {
        rosidl_typesupport_c__get_service_type_support_handle__type_description_interfaces__srv__GetTypeDescription()
    };

    let x = node.borrow_impl().unwrap();
    let service_name = CString::from_str(&format!(
        "{:?}/{:?}/get_type_description",
        x.namespace, x.name
    ))
    .unwrap();

    unsafe {
        rcl_service_init(
            service,
            node,
            type_support,
            service_name.as_ptr(),
            std::ptr::null(),
        )
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_is_valid_except_context(node: *const rcl_node_t) -> bool {
    node.borrow_impl().is_ok()
}
