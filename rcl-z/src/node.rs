use std::ops::Deref;

use crate::context::ContextImpl;
use crate::ros::*;
use ros_z::node::ZNode;

pub struct NodeImpl(pub ZNode);

impl Deref for NodeImpl {
    type Target = ZNode;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_init(
    node: *mut rcl_node_t,
    _name: *const ::std::os::raw::c_char,
    _namespace_: *const ::std::os::raw::c_char,
    context: *mut rcl_context_t,
    _options: *const rcl_node_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_node_init");
    unsafe {
        let context_impl = &*((*context).impl_ as *const rcl_context_t as *const ContextImpl);

        let znode = context_impl.create_node();
        let node_impl = NodeImpl(znode);
        (*node).impl_ = Box::into_raw(Box::new(node_impl)) as _;
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_fini(node: *mut rcl_node_t) -> rcl_ret_t {
    tracing::trace!("rcl_node_fini");
    unsafe {
        std::mem::drop(Box::from_raw((*node).impl_ as *mut NodeImpl));
    }
    RCL_RET_OK as _
}
