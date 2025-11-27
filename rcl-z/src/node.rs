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
use crate::qos::rmw_qos_to_ros_z_qos;
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
use ros_z::Builder;
use ros_z::entity::TypeInfo;
use ros_z::graph::Graph;
use ros_z::node::ZNode;
use zenoh::Result;

fn is_valid_node_name(name: &str) -> bool {
    if name.is_empty() {
        return false;
    }
    let bytes = name.as_bytes();
    if !bytes[0].is_ascii_alphabetic() && bytes[0] != b'_' {
        return false;
    }
    for &b in &bytes[1..] {
        if !b.is_ascii_alphanumeric() && b != b'_' {
            return false;
        }
    }
    true
}

fn is_valid_namespace(ns: &str) -> bool {
    if ns.is_empty() {
        return true;
    }
    if ns == "/" {
        return true;
    }
    if ns.ends_with('/') {
        return false;
    }
    for part in ns.split('/') {
        if part.is_empty() {
            continue;
        }
        let bytes = part.as_bytes();
        if bytes.is_empty() || !bytes[0].is_ascii_alphabetic() && bytes[0] != b'_' {
            return false;
        }
        for &b in &bytes[1..] {
            if !b.is_ascii_alphanumeric() && b != b'_' {
                return false;
            }
        }
    }
    true
}

pub struct NodeImpl {
    pub(crate) inner: ZNode,
    pub(crate) name: CString,
    pub(crate) namespace: CString,
    pub(crate) fq_name: CString,
    pub(crate) logger_name: CString,
    pub(crate) notifier: Arc<Notifier>,
    pub(crate) instance_id: u64,
    pub(crate) options: rcl_node_options_t,
    pub(crate) rmw_handle: *mut rmw_node_t,
    pub(crate) graph_guard_condition: rcl_guard_condition_t,
}

impl NodeImpl {
    pub unsafe fn new_sub(
        &self,
        type_support: *const rosidl_message_type_support_t,
        topic_name: *const ::std::os::raw::c_char,
        options: *const rcl_subscription_options_t,
    ) -> Result<SubscriptionImpl> {
        // Message type support
        let ts = unsafe { MessageTypeSupport::new(type_support)? };
        let topic_str = str_from_ptr(topic_name)?;
        let type_info = TypeInfo::new(&ts.get_type_prefix(), ts.get_type_hash());

        // Notification
        let notifier = self.notifier.clone();
        let notify_callback = move || notifier.notify_all();

        // Convert QoS profile from options
        let options_ref = unsafe { &*options };
        let qos_profile = rmw_qos_to_ros_z_qos(&options_ref.qos);

        // Topic qualification now happens inside ros-z ZSubBuilder::build()
        let zsub = self
            .inner
            .create_sub::<RosMessage>(topic_str)
            .with_type_info(type_info)
            .with_serdes::<crate::msg::RosSerdes>()
            .with_qos(qos_profile)
            .build_with_notifier(notify_callback)?;

        // Cache the qualified topic name from the built subscription
        let topic_cstr = CString::new(zsub.entity.topic.clone()).unwrap();

        let options = unsafe { std::ptr::read(options) };

        Ok(SubscriptionImpl {
            inner: zsub,
            ts,
            topic: topic_cstr,
            options,
        })
    }

    pub unsafe fn new_pub(
        &self,
        type_support: *const rosidl_message_type_support_t,
        topic_name: *const ::std::os::raw::c_char,
        options: *const rcl_publisher_options_t,
    ) -> Result<PublisherImpl> {
        // Message type support
        let ts = unsafe { MessageTypeSupport::new(type_support)? };
        let type_info = TypeInfo::new(&ts.get_type_prefix(), ts.get_type_hash());
        let topic_str = str_from_ptr(topic_name)?;

        // Convert QoS profile from options
        let options_ref = unsafe { &*options };
        let qos_profile = rmw_qos_to_ros_z_qos(&options_ref.qos);

        // Topic qualification now happens inside ros-z ZPubBuilder::build()
        let zpub = self
            .inner
            .create_pub(topic_str)
            .with_type_info(type_info)
            .with_serdes::<crate::msg::RosSerdes>()
            .with_qos(qos_profile)
            .build()?;

        // Cache the qualified topic name from the built publisher
        let topic_clone = CString::new(zpub.entity.topic.clone())?;

        let options = unsafe { std::ptr::read(options) };

        Ok(PublisherImpl {
            inner: zpub,
            ts,
            topic: topic_clone,
            options,
        })
    }

    pub unsafe fn new_client(
        &self,
        type_support: *const rosidl_service_type_support_t,
        service_name: *const ::std::os::raw::c_char,
        _options: *const rcl_client_options_t,
    ) -> Result<ClientImpl> {
        let ts = unsafe { ServiceTypeSupport::new(type_support)? };
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
            service_name: topic.to_string(),
            options: unsafe { std::ptr::read(_options) },
        })
    }

    pub unsafe fn new_service(
        &self,
        type_support: *const rosidl_service_type_support_t,
        service_name: *const ::std::os::raw::c_char,
        _options: *const rcl_service_options_t,
    ) -> Result<ServiceImpl> {
        let ts = unsafe { ServiceTypeSupport::new(type_support)? };

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

        // Compute the fully qualified service name using the qualified name logic
        let service_name_qualified = ros_z::topic_name::qualify_service_name(
            topic,
            &self.inner.entity.namespace,
            &self.inner.entity.name,
        )?;

        let service_name_cstring = CString::new(service_name_qualified)?;

        Ok(ServiceImpl {
            inner: zsrv,
            ts,
            service_name: service_name_cstring,
        })
    }

    pub fn graph(&self) -> &Arc<Graph> {
        &self.inner.graph
    }
}

impl_has_impl_ptr!(rcl_node_t, rcl_node_impl_t, NodeImpl);

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_node() -> rcl_node_t {
    // TODO: Implement proper zero initialization
    unsafe { std::mem::zeroed() }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_node_init(
    node: *mut rcl_node_t,
    name: *const ::std::os::raw::c_char,
    namespace_: *const ::std::os::raw::c_char,
    context: *mut rcl_context_t,
    options: *const rcl_node_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_node_init");

    // FIXME: Add tracing::warning for each case
    if node.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if context.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if name.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if options.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check if context is initialized
    if unsafe { !crate::context::rcl_context_is_valid(context) } {
        return RCL_RET_NOT_INIT as _;
    }

    // Check if node is already initialized
    unsafe {
        if !(*node).impl_.is_null() {
            return RCL_RET_ALREADY_INIT as _;
        }
    }

    let name_str = match str_from_ptr(name) {
        Ok(s) => s,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };
    let namespace_str = match str_from_ptr(namespace_) {
        Ok(s) => s,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };
    if !is_valid_node_name(name_str) {
        return RCL_RET_NODE_INVALID_NAME as _;
    }
    if !is_valid_namespace(namespace_str) {
        return RCL_RET_NODE_INVALID_NAMESPACE as _;
    }

    unsafe {
        (*node).context = context;
    }
    let x = move || {
        let node_impl = context
            .borrow_impl()?
            .new_node(name, namespace_, context, options)?;
        node.assign_impl(node_impl)?;
        zenoh::Result::Ok(())
    };

    rclz_try! {x()?;}
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_fini(node: *mut rcl_node_t) -> rcl_ret_t {
    tracing::trace!("rcl_node_fini");
    if node.is_null() {
        return RCL_RET_NODE_INVALID as _;
    }
    unsafe {
        if (*node).impl_.is_null() {
            return RCL_RET_OK as _;
        }
    }
    rclz_try! {
        std::mem::drop(node.own_impl()?);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_logger_name(
    node: *const rcl_node_t,
) -> *const ::std::os::raw::c_char {
    tracing::trace!("rcl_node_get_logger_name");
    // Check if node has impl (even if context is invalid)
    // This matches C++ behavior where logger name is returned even for invalid nodes
    if node.is_null() {
        return std::ptr::null();
    }
    match node.borrow_impl() {
        Ok(node_impl) => node_impl.logger_name.as_ptr(),
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_fully_qualified_name(
    node: *const rcl_node_t,
) -> *const ::std::os::raw::c_char {
    // Return FQ name even if context is invalid (matches C++ behavior)
    if node.is_null() {
        return std::ptr::null();
    }
    match node.borrow_impl() {
        Ok(impl_) => impl_.fq_name.as_ptr(),
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_default_options() -> rcl_node_options_t {
    // TODO: Implement proper default options with arguments
    unsafe { std::mem::zeroed() }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_namespace(node: *const rcl_node_t) -> *const ::std::os::raw::c_char {
    match node.borrow_impl() {
        Ok(impl_) => impl_.namespace.as_ptr(),
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_name(node: *const rcl_node_t) -> *const ::std::os::raw::c_char {
    match node.borrow_impl() {
        Ok(impl_) => impl_.name.as_ptr(),
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_expand_topic_name(
    input_topic_name: *const ::std::os::raw::c_char,
    node_name: *const ::std::os::raw::c_char,
    node_namespace: *const ::std::os::raw::c_char,
    substitutions: *const rcutils_string_map_t,
    allocator: rcl_allocator_t,
    output_topic_name: *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    // Validate input parameters
    if input_topic_name.is_null() || node_name.is_null() || node_namespace.is_null() || output_topic_name.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Convert C strings to Rust strings
    let topic_str = match str_from_ptr(input_topic_name) {
        Ok(s) => s,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };

    let name_str = match str_from_ptr(node_name) {
        Ok(s) => s,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };

    let namespace_str = match str_from_ptr(node_namespace) {
        Ok(s) => s,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };

    // TODO: Handle substitutions parameter (not currently used in ros-z)
    // For now, we ignore substitutions

    // Use ros-z's topic qualification
    let qualified = match ros_z::topic_name::qualify_topic_name(topic_str, namespace_str, name_str) {
        Ok(q) => q,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };

    // Allocate and return the qualified topic name
    match CString::new(qualified) {
        Ok(cstr) => {
            unsafe {
                *output_topic_name = cstr.into_raw();
            }
            RCL_RET_OK as _
        }
        Err(_) => RCL_RET_INVALID_ARGUMENT as _,
    }
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
pub unsafe extern "C" fn rcl_node_resolve_name(
    node: *const rcl_node_t,
    input_name: *const ::std::os::raw::c_char,
    allocator: rcl_allocator_t,
    is_service: bool,
    only_expand: bool,
    output_name: *mut *mut ::std::os::raw::c_char,
) -> rcl_ret_t {
    if node.is_null() || input_name.is_null() || output_name.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check if node has valid implementation
    if node.borrow_impl().is_err() {
        return RCL_RET_ERROR as _;
    }

    let cstr = CString::new("skip_rcl_node_resolve_name").unwrap();
    unsafe {
        *output_name = cstr.into_raw();
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_is_valid(node: *const rcl_node_t) -> bool {
    if node.is_null() {
        return false;
    }
    unsafe {
        if !crate::context::rcl_context_is_valid((*node).context as *const _) {
            return false;
        }
    }
    node.borrow_impl().is_ok()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_node_type_description_service_init(
    service: *mut rcl_service_t,
    node: *const rcl_node_t,
) -> rcl_ret_t {
    let type_support = unsafe {
        rosidl_typesupport_c__get_service_type_support_handle__type_description_interfaces__srv__GetTypeDescription()
    };

    let x = match node.borrow_impl() {
        Ok(impl_) => impl_,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };
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

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_options(node: *const rcl_node_t) -> *const rcl_node_options_t {
    // Return options even if context is invalid (matches C++ behavior)
    if node.is_null() {
        return std::ptr::null();
    }
    match node.borrow_impl() {
        Ok(impl_) => &impl_.options,
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_domain_id(
    node: *const rcl_node_t,
    domain_id: *mut usize,
) -> rcl_ret_t {
    if node.is_null() || domain_id.is_null() {
        return RCL_RET_NODE_INVALID as _;
    }
    if !rcl_node_is_valid(node) {
        return RCL_RET_NODE_INVALID as _;
    }
    // Get domain_id from the context
    let context = unsafe { (*node).context };
    crate::context::rcl_context_get_domain_id(context, domain_id)
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_rmw_handle(node: *const rcl_node_t) -> *mut rmw_node_t {
    // Return rmw_handle even if context is invalid (matches C++ behavior)
    if node.is_null() {
        return std::ptr::null_mut();
    }
    match node.borrow_impl() {
        Ok(impl_) => impl_.rmw_handle,
        Err(_) => std::ptr::null_mut(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_rcl_instance_id(node: *const rcl_node_t) -> u64 {
    if node.is_null() {
        return 0;
    }
    let node_impl = match node.borrow_impl() {
        Ok(impl_) => impl_,
        Err(_) => return 0,
    };
    let context = unsafe { (*node).context };
    if unsafe { !rcl_context_is_valid(context) } {
        return 0;
    }
    node_impl.instance_id
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_graph_guard_condition(
    node: *const rcl_node_t,
) -> *const rcl_guard_condition_t {
    // Return guard condition even if context is invalid (matches C++ behavior)
    if node.is_null() {
        return std::ptr::null();
    }
    match node.borrow_impl() {
        Ok(impl_) => &impl_.graph_guard_condition,
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_options_copy(
    src: *const rcl_node_options_t,
    dst: *mut rcl_node_options_t,
) -> rcl_ret_t {
    if src.is_null() || dst.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        // Check if dst and src are the same
        if src == dst {
            tracing::error!("Attempted to copy options into itself");
            return RCL_RET_INVALID_ARGUMENT as _;
        }

        // Check if dst->arguments.impl is not null (must be zero initialized)
        if !(*dst).arguments.impl_.is_null() {
            tracing::error!("Options out must be zero initialized");
            return RCL_RET_INVALID_ARGUMENT as _;
        }

        // Copy simple fields
        (*dst).allocator = (*src).allocator;
        (*dst).use_global_arguments = (*src).use_global_arguments;
        (*dst).enable_rosout = (*src).enable_rosout;
        (*dst).rosout_qos = (*src).rosout_qos;

        // If src has initialized arguments, copy them
        if !(*src).arguments.impl_.is_null() {
            use crate::arguments::rcl_arguments_copy;
            return rcl_arguments_copy(&(*src).arguments, &mut (*dst).arguments);
        }
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_options_fini(options: *mut rcl_node_options_t) -> rcl_ret_t {
    if options.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_disable_loaned_message(disable_loaned_message: *mut bool) -> rcl_ret_t {
    if disable_loaned_message.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    let val = std::env::var("ROS_DISABLE_LOANED_MESSAGES").unwrap_or_else(|_| "false".to_string());
    let disable = val == "1" || val.to_lowercase() == "true";
    unsafe {
        *disable_loaned_message = disable;
    }
    RCL_RET_OK as _
}
