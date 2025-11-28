#![allow(unused)]

use crate::event::*;
use crate::node::NodeImpl;
use crate::ros::*;
use crate::traits::{BorrowImpl, OwnImpl};
use crate::utils::{Notifier, str_from_ptr};
use crate::{impl_has_impl_ptr, rclz_try};
use ros_z::Builder;
use ros_z::context::ZContext;
use std::ffi::c_void;
use std::str::FromStr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::{
    ffi::{CString, c_char, c_int},
    ops::Deref,
};
use zenoh::Result;

// Global instance ID counter
static INSTANCE_ID_COUNTER: AtomicU64 = AtomicU64::new(1);

pub struct ContextImpl {
    inner: ZContext,
    notifier: Arc<Notifier>,
    /// Instance ID for this context (increments globally)
    instance_id: u64,
    /// Copy of init options given during init
    init_options: rcl_init_options_t,
    /// RMW context (zero-initialized for now)
    rmw_context: rmw_context_t,
    /// Whether the context is valid (active/not shutdown)
    is_valid: bool,
    /// Domain ID for this context
    domain_id: usize,
    /// Events manager for this context
    events_manager: EventsManager,
}

impl ContextImpl {
    pub(crate) fn new(zcontext: ZContext, init_options: rcl_init_options_t, domain_id: usize) -> Self {
        // Fetch and increment the global instance ID
        let instance_id = INSTANCE_ID_COUNTER.fetch_add(1, Ordering::SeqCst);

        Self {
            inner: zcontext,
            notifier: Arc::new(Notifier::default()),
            instance_id,
            init_options,
            events_manager: EventsManager::new(),
            rmw_context: rmw_context_t::default(),
            is_valid: true,
            domain_id,
        }
    }

    pub(crate) fn events_manager(&self) -> &EventsManager {
        &self.events_manager
    }

    pub(crate) fn events_manager_mut(&mut self) -> &mut EventsManager {
        &mut self.events_manager
    }

    pub(crate) fn new_node(
        &self,
        name: *const ::std::os::raw::c_char,
        namespace_: *const ::std::os::raw::c_char,
        context: *mut rcl_context_t,
        options: *const rcl_node_options_t,
    ) -> Result<NodeImpl> {
        // Normalize namespace: zenoh expects namespace WITHOUT leading slash
        // but ROS2 provides it WITH leading slash, so we need to strip it for zenoh
        let input_namespace = str_from_ptr(namespace_)?;
        let normalized_namespace = if input_namespace.is_empty() || input_namespace == "/" {
            ""
        } else {
            input_namespace.strip_prefix('/').unwrap_or(input_namespace)
        };

        let znode = self
            .inner
            .create_node(str_from_ptr(name)?)
            .with_namespace(normalized_namespace)
            .build()?;

        let namespace_str = &znode.entity.namespace;
        let name_str = &znode.entity.name;

        // Ensure namespace has leading / to match C++ RCL behavior
        let namespace_with_slash = if namespace_str.is_empty() {
            "/".to_string()
        } else {
            format!("/{}", namespace_str)
        };

        // Fully qualified name should always start with /
        let fq_name_str = if namespace_with_slash == "/" {
            format!("/{}", name_str)
        } else {
            format!("{}/{}", namespace_with_slash, name_str)
        };

        // Logger name: remove leading / and replace / with .
        let logger_name_str = if namespace_with_slash == "/" {
            name_str.to_string()
        } else {
            namespace_with_slash[1..].replace('/', ".") + "." + name_str
        };

        let namespace = CString::new(namespace_with_slash)?;
        let name = CString::from_str(name_str)?;
        let fq_name = CString::new(fq_name_str).unwrap();
        let logger_name = CString::new(logger_name_str).unwrap();

        // Copy options if provided
        let node_options = if options.is_null() {
            rcl_node_options_t::default()
        } else {
            unsafe { std::ptr::read(options) }
        };

        // Create a dummy rmw_handle (non-null pointer for compatibility)
        let rmw_handle = Box::into_raw(Box::new(rmw_node_t::default()));

        // Create and initialize the graph guard condition
        let mut graph_guard_condition = rcl_guard_condition_t::default();
        let guard_impl = Box::new(crate::guard_condition::GuardConditionImpl {
            notifier: Some(self.notifier.clone()),
            triggered: false,
        });
        graph_guard_condition.impl_ = Box::into_raw(guard_impl) as *mut _;

        Ok(NodeImpl {
            inner: znode,
            name,
            namespace,
            fq_name,
            logger_name,
            notifier: self.notifier.clone(),
            instance_id: self.instance_id,
            options: node_options,
            rmw_handle,
            graph_guard_condition,
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

// TODO: Not implemented yet
#[unsafe(no_mangle)]
pub extern "C" fn rcl_logging_rosout_enabled() -> bool {
    // Rosout logging is always enabled in our implementation
    true
}

// TODO: Not implemented yet
#[unsafe(no_mangle)]
pub extern "C" fn rcl_logging_rosout_init_publisher_for_node(
    _node: *mut rcl_node_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_logging_rosout_init_publisher_for_node");
    // In the Zenoh-based implementation, rosout publishers are not needed
    // as logging is handled differently. Return OK to indicate success.
    RCL_RET_OK as _
}

// TODO: Not implemented yet
#[unsafe(no_mangle)]
pub extern "C" fn rcl_logging_rosout_fini_publisher_for_node(
    _node: *mut rcl_node_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_logging_rosout_fini_publisher_for_node");
    // Nothing to clean up in our implementation
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
pub unsafe extern "C" fn rcl_context_is_valid(context: *const rcl_context_t) -> bool {
    if context.is_null() {
        return false;
    }

    unsafe {
        if (*context).impl_.is_null() {
            return false;
        }
    }

    // Check if the impl is valid (not shutdown)
    match context.borrow_impl() {
        Ok(impl_) => impl_.is_valid,
        Err(_) => false,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_shutdown(context: *mut rcl_context_t) -> rcl_ret_t {
    tracing::trace!("rcl_shutdown");

    // Check for null pointer
    if context.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check if context is zero-initialized (never init'd)
    unsafe {
        if (*context).impl_.is_null() {
            return RCL_RET_INVALID_ARGUMENT as _;
        }
    }

    // Check if context is valid (if not valid, it's already shutdown)
    if !unsafe { rcl_context_is_valid(context) } {
        return RCL_RET_ALREADY_SHUTDOWN as _;
    }

    // Shutdown the context
    match context.borrow_mut_impl() {
        Ok(impl_) => {
            match impl_.inner.shutdown() {
                Ok(_) => {
                    // Mark context as invalid
                    impl_.is_valid = false;
                    RCL_RET_OK as _
                }
                Err(e) => {
                    tracing::error!("rcl_shutdown failed: {e}");
                    RCL_RET_ERROR as _
                }
            }
        }
        Err(_) => RCL_RET_INVALID_ARGUMENT as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_context_fini(context: *mut rcl_context_t) -> rcl_ret_t {
    // TODO: tracing is not usable at the exit stage
    // tracing::trace!("rcl_context_fini");

    // Check for null argument
    if context.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        // If context is zero-initialized (impl is null), return OK
        if (*context).impl_.is_null() {
            return RCL_RET_OK as _;
        }

        // If context is still valid (not shutdown), return error
        if rcl_context_is_valid(context) {
            // In C, this sets an error: "rcl_shutdown() not called on the given context"
            return RCL_RET_INVALID_ARGUMENT as _;
        }
    }

    // TODO: Implement proper cleanup of context resources
    // For now, just return OK after checks
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_context() -> rcl_context_t {
    rcl_context_t::default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_context_get_instance_id(context: *const rcl_context_t) -> u64 {
    match context.borrow_impl() {
        Ok(impl_) => {
            // Return 0 if context is not valid (shutdown)
            if impl_.is_valid {
                impl_.instance_id
            } else {
                0
            }
        }
        Err(_) => 0,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_context_get_domain_id(
    context: *const rcl_context_t,
    domain_id: *mut usize,
) -> rcl_ret_t {
    if context.is_null() || domain_id.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    match context.borrow_impl() {
        Ok(impl_) => {
            unsafe {
                *domain_id = impl_.domain_id;
            }
            RCL_RET_OK as _
        }
        Err(_) => RCL_RET_INVALID_ARGUMENT as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_context_get_init_options(
    context: *const rcl_context_t,
) -> *const rcl_init_options_t {
    // Check for null argument
    if context.is_null() {
        return std::ptr::null();
    }

    // Check if context is zero-initialized
    unsafe {
        if (*context).impl_.is_null() {
            // In C, this sets an error: "context is zero-initialized"
            return std::ptr::null();
        }
    }

    // Return pointer to init_options stored in context impl
    match context.borrow_impl() {
        Ok(impl_) => &impl_.init_options,
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_context_get_rmw_context(
    context: *mut rcl_context_t,
) -> *mut rmw_context_t {
    // Check for null argument
    if context.is_null() {
        return std::ptr::null_mut();
    }

    // Check if context is zero-initialized
    unsafe {
        if (*context).impl_.is_null() {
            // In C, this sets an error: "context is zero-initialized"
            return std::ptr::null_mut();
        }
    }

    // Return pointer to rmw_context stored in context impl
    match context.borrow_mut_impl() {
        Ok(impl_) => &mut impl_.rmw_context,
        Err(_) => std::ptr::null_mut(),
    }
}
