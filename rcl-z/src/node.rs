#![allow(unused)]
use std::ffi::CString;
use std::ops::Deref;

use crate::context::ContextImpl;
use crate::impl_has_impl_ptr;
use crate::ros::*;
use crate::traits::{BorrowImpl, OwnImpl};
use crate::utils::NOT_SUPPORTED_CSTR;
use crate::utils::str_from_ptr;
use crate::rclz_try;
use ros_z::Builder;
use ros_z::node::ZNode;

pub struct NodeImpl(pub ZNode);

impl Deref for NodeImpl {
    type Target = ZNode;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl_has_impl_ptr!(rcl_node_t, rcl_node_impl_t, NodeImpl);

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_init(
    node: *mut rcl_node_t,
    name: *const ::std::os::raw::c_char,
    _namespace_: *const ::std::os::raw::c_char,
    context: *mut rcl_context_t,
    _options: *const rcl_node_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_node_init");

    rclz_try! {
        let znode = context
            .borrow_impl()?
            .create_node(str_from_ptr(name).unwrap())
            .build()
            .unwrap();
        node.assign_impl(NodeImpl(znode))?;
    }
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
    Box::into_raw(Box::new(CString::new("no rcl-z node namespace").unwrap())) as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_get_name(node: *const rcl_node_t) -> *const ::std::os::raw::c_char {
    Box::into_raw(Box::new(CString::new("no rcl-z node namespace").unwrap())) as _
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
    RCL_RET_OK as _
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
    true
}
