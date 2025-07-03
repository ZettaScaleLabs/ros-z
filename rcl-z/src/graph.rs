use crate::ros::*;
use crate::traits::{BorrowImpl, OwnImpl};
use ros_z::entity::EntityKind;

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_topic_names_and_types(
    node: *const rcl_node_t,
    _allocator: *mut rcl_allocator_t,
    _no_demangle: bool,
    topic_names_and_types: *mut rcl_names_and_types_t,
) -> rcl_ret_t {
    // let node_impl = node.borrow_impl().unwrap();
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_publishers_info_by_topic(
    node: *const rcl_node_t,
    allocator: *mut rcutils_allocator_t,
    topic_name: *const ::std::os::raw::c_char,
    no_mangle: bool,
    publishers_info: *mut rcl_topic_endpoint_info_array_t,
) -> rcl_ret_t {
    // let node_impl = node.borrow_impl().unwrap();
    // node_impl.graph().get_entities_by_topic(EntityKind::Publisher, topic_name);
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_subscriptions_info_by_topic(
    node: *const rcl_node_t,
    allocator: *mut rcutils_allocator_t,
    topic_name: *const ::std::os::raw::c_char,
    no_mangle: bool,
    subscriptions_info: *mut rcl_topic_endpoint_info_array_t,
) -> rcl_ret_t {
    RCL_RET_OK as _
}
