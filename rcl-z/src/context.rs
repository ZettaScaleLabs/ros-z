#![allow(unused)]

use crate::node::NodeImpl;
use crate::ros::*;
use crate::traits::{BorrowImpl, OwnImpl};
use crate::utils::{Notifier, str_from_ptr};
use crate::{impl_has_impl_ptr, rclz_try};
use ros_z::Builder;
use ros_z::context::ZContext;
use std::str::FromStr;
use std::sync::Arc;
use std::{
    ffi::{CString, c_char, c_int},
    ops::Deref,
};
use zenoh::Result;

pub struct ContextImpl {
    inner: ZContext,
    notifier: Arc<Notifier>,
}

impl ContextImpl {
    pub(crate) fn new(zcontext: ZContext) -> Self {
        Self {
            inner: zcontext,
            notifier: Arc::new(Notifier::default()),
        }
    }

    pub(crate) fn new_node(
        &self,
        name: *const ::std::os::raw::c_char,
        namespace_: *const ::std::os::raw::c_char,
        context: *mut rcl_context_t,
        _options: *const rcl_node_options_t,
    ) -> Result<NodeImpl> {
        let znode = self
            .inner
            .create_node(str_from_ptr(name)?)
            .with_namespace(str_from_ptr(namespace_)?)
            .build()?;

        let namespace = match &znode.entity.namespace {
            Some(ns) => CString::from_str(&ns)?,
            None => CString::new("")?,
        };
        let name = CString::from_str(&znode.entity.name)?;
        Ok(NodeImpl {
            inner: znode,
            name,
            namespace,
            notifier: self.notifier.clone(),
        })
    }

    pub(crate) fn share_notifier(&self) -> Arc<Notifier> {
        self.notifier.clone()
    }
}

impl_has_impl_ptr!(rcl_context_t, rcl_context_impl_t, ContextImpl);

#[unsafe(no_mangle)]
pub extern "C" fn rcl_logging_configure_with_output_handler(
    _global_args: *const rcl_arguments_t,
    _allocator: *const rcl_allocator_t,
    _output_handler: rcl_logging_output_handler_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_logging_configure_with_output_handler");
    RCL_RET_OK as _
}


#[unsafe(no_mangle)]
pub extern "C" fn rcl_parse_arguments(
    argc: ::std::os::raw::c_int,
    argv: *const *const ::std::os::raw::c_char,
    allocator: rcl_allocator_t,
    args_output: *mut rcl_arguments_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_parse_arguments");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_arguments_get_count_unparsed_ros(
    args: *const rcl_arguments_t,
) -> ::std::os::raw::c_int {
    tracing::trace!("rcl_arguments_get_count_unparsed_ros");
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_arguments_get_param_overrides(
    arguments: *const rcl_arguments_t,
    parameter_overrides: *mut *mut rcl_params_t,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_get_rmw_handle(
    publisher: *const rcl_publisher_t,
) -> *mut rmw_publisher_t {
    Box::into_raw(Box::new(())) as _
    // // TODO: Recycle this
    // let mut rmw_pub = rmw_publisher_t::default();
    // let gid = rmw_gid_t
    // rmw_pub.data =
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_gid_for_publisher(
    publisher: *const rmw_publisher_t,
    gid: *mut rmw_gid_t,
) -> rmw_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_publisher_event_init(
    event: *mut rcl_event_t,
    publisher: *const rcl_publisher_t,
    event_type: rcl_publisher_event_type_t,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_subscription_event_init(
    event: *mut rcl_event_t,
    subscription: *const rcl_subscription_t,
    event_type: rcl_subscription_event_type_t,
) -> rcl_ret_t {
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_context_is_valid(context: *const rcl_context_t) -> bool {
    unsafe { !(*context).impl_.is_null() }
}


#[unsafe(no_mangle)]
pub extern "C" fn rcl_shutdown(context: *mut rcl_context_t) -> rcl_ret_t {
    tracing::trace!("rcl_shutdown");
    rclz_try! {
        context.own_impl()?.inner.shutdown()?;
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_context_fini(_context: *mut rcl_context_t) -> rcl_ret_t {
    // FIXME: tracing is not usable at the exit stage
    // tracing::trace!("rcl_context_fini");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_context() -> rcl_context_t {
    rcl_context_t::default()
}
