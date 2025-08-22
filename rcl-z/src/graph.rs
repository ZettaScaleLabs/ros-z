#![allow(unused)]
use std::convert::TryFrom;
use std::ffi::{CStr, CString, NulError};
use std::os::raw::c_char;
use std::ptr;
use std::slice::from_raw_parts;
use std::str::FromStr;

use crate::ros::*;
use crate::traits::{BorrowImpl, OwnImpl};
use crate::utils::str_from_ptr;
use ros_z::entity::{EndpointEntity, EntityKind, NodeKey};
use ros_z::qos::{QosProfile, QosReliability, QosDurability, QosHistory};

impl TryFrom<EndpointEntity> for rmw_topic_endpoint_info_t {
    type Error = NulError;
    fn try_from(value: EndpointEntity) -> Result<Self, Self::Error> {
        let node_name = CString::from_str(&value.node.name)?.into_raw();
        let node_namespace = if value.node.namespace.is_empty() {
            std::ptr::null()
        } else {
            CString::from_str(&value.node.namespace)?.into_raw()
        };
        let topic_type = match &value.type_info {
            Some(info) => CString::from_str(&info.name)?.into_raw(),
            None => std::ptr::null()
        };

        // Convert TypeHash to rosidl_type_hash_t
        let topic_type_hash = match &value.type_info {
            Some(info) => rosidl_type_hash_t {
                version: info.hash.version,
                value: info.hash.value,
            },
            None => rosidl_type_hash_t {
                version: 0,
                value: [0u8; 32],
            }
        };

        // Convert EntityKind to rmw_endpoint_type_t
        let endpoint_type = match value.kind {
            EntityKind::Publisher => rmw_endpoint_type_e::RMW_ENDPOINT_PUBLISHER,
            EntityKind::Subscription => rmw_endpoint_type_e::RMW_ENDPOINT_SUBSCRIPTION,
            _ => rmw_endpoint_type_e::RMW_ENDPOINT_INVALID,
        };

        // Get GID from EndpointEntity
        let endpoint_gid = value.gid();

        // Convert QosProfile to rmw_qos_profile_t
        let qos_profile = (&value.qos).into();

        Ok(rmw_topic_endpoint_info_s {
            node_name,
            node_namespace,
            topic_type,
            topic_type_hash,
            endpoint_type,
            endpoint_gid,
            qos_profile,
        })
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_names_and_types() -> rcl_names_and_types_t {
    let x = rcl_names_and_types_t::default();
    x
}

// impl TryFrom<EndpointEntity> for rcl_names_and_types_t {
//     type Error = NulError;
//     fn try_from(value: EndpointEntity) -> Result<Self, Self::Error> {
//     pub names: rcutils_string_array_t,
//     pub types: *mut rcutils_string_array_t,
//     }
// }

// impl<S> From<Vec<S>> for rcutils_string_array_t
// where
//     S: AsRef<str>,
// {
//     fn from(strings: Vec<S>) -> Self {
//         let mut c_strings: Vec<*mut c_char> = strings
//             .into_iter()
//             .map(|s| {
//                 CString::new(s.as_ref())
//                     .expect("CString::new failed")
//                     .into_raw()
//             })
//             .collect();
//
//         let len = c_strings.len();
//         let ptr = c_strings.as_mut_ptr();
//         std::mem::forget(c_strings);
//
//         Self {
//             data: ptr,
//             size: len,
//             allocator: rcutils_allocator_t::default(),
//         }
//     }
// }

impl From<Vec<CString>> for rcutils_string_array_t {
    fn from(strings: Vec<CString>) -> Self {
        let mut c_strings: Vec<*mut c_char> = strings.into_iter().map(|s| s.into_raw()).collect();

        let len = c_strings.len();
        let ptr = c_strings.as_mut_ptr();
        std::mem::forget(c_strings);

        Self {
            data: ptr,
            size: len,
            allocator: rcutils_allocator_t::default(),
        }
    }
}

impl rcutils_string_array_t {
    pub fn as_vec(&self) -> Vec<&CStr> {
        if self.data.is_null() || self.size == 0 {
            return Vec::new();
        }

        unsafe {
            std::slice::from_raw_parts(self.data, self.size)
                .iter()
                .map(|&ptr| CStr::from_ptr(ptr))
                .collect()
        }
    }

    pub fn free(self) {
        if self.data.is_null() || self.size == 0 {
            return;
        }

        unsafe {
            std::slice::from_raw_parts(self.data, self.size)
                .iter()
                .for_each(|&ptr| {
                    if !ptr.is_null() {
                        drop(CString::from_raw(ptr));
                    }
                });

            drop(Vec::from_raw_parts(self.data, self.size, self.size));
        }
    }
}

impl rcl_names_and_types_t {
    fn free(&mut self) {
        self.names.free();
        if !self.types.is_null() {
            unsafe {
                Box::from_raw(self.types).free();
            }
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_topic_names_and_types(
    node: *const rcl_node_t,
    _allocator: *mut rcl_allocator_t,
    _no_demangle: bool,
    topic_names_and_types: *mut rcl_names_and_types_t,
) -> rcl_ret_t {
    tracing::error!("rcl_get_topic_names_and_types");
    assert!(!topic_names_and_types.is_null());

    let node_impl = node.borrow_impl().unwrap();

    let topic_data = node_impl.graph().get_topic_names_and_types();

    let mut topic_names = Vec::new();
    let mut type_names = Vec::new();

    for (topic_name, type_name) in topic_data {
        topic_names.push(CString::from_str(&topic_name).unwrap());
        type_names.push(CString::from_str(&type_name).unwrap());
    }

    unsafe {
        (*topic_names_and_types).names = rcutils_string_array_t::from(topic_names);
        (*topic_names_and_types).types =
            Box::into_raw(Box::new(rcutils_string_array_t::from(type_names)));
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_names_and_types_fini(
    topic_names_and_types: *mut rcl_names_and_types_t,
) -> rcl_ret_t {
    tracing::error!("rcl_names_and_types_fini");
    if !topic_names_and_types.is_null() {
        unsafe {
            (*topic_names_and_types).free();
        }
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_service_names_and_types(
    node: *const rcl_node_t,
    _allocator: *mut rcl_allocator_t,
    service_names_and_types: *mut rcl_names_and_types_t,
) -> rcl_ret_t {
    tracing::error!("rcl_get_service_names_and_types");
    assert!(!service_names_and_types.is_null());

    let node_impl = node.borrow_impl().unwrap();

    let service_data = node_impl.graph().get_service_names_and_types();

    let mut service_names = Vec::new();
    let mut type_names = Vec::new();

    for (service_name, type_name) in service_data {
        service_names.push(CString::from_str(&service_name).unwrap());
        type_names.push(CString::from_str(&type_name).unwrap());
    }

    unsafe {
        (*service_names_and_types).names = rcutils_string_array_t::from(service_names);
        (*service_names_and_types).types =
            Box::into_raw(Box::new(rcutils_string_array_t::from(type_names)));
    }
    RCL_RET_OK as _
}

fn rcl_get_names_and_types_by_node_impl(
    node: *const rcl_node_t,
    remote_node_name: *const ::std::os::raw::c_char,
    remote_node_namespace: *const ::std::os::raw::c_char,
    names_and_types: *mut rcl_names_and_types_t,
    kind: EntityKind,
) -> rcl_ret_t {
    if node.is_null() || remote_node_name.is_null() || remote_node_namespace.is_null() || names_and_types.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_impl() {
        Ok(impl_) => impl_,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };
    
    let remote_name = match str_from_ptr(remote_node_name) {
        Ok(s) => s,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };
    
    let remote_namespace = match str_from_ptr(remote_node_namespace) {
        Ok(s) => s,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };
    
    let remote_node_key = (remote_namespace.to_string(), remote_name.to_string());
    let data = node_impl.graph().get_names_and_types_by_node(remote_node_key, kind);
    
    let mut names = Vec::new();
    let mut types = Vec::new();
    
    for (name, type_name) in data {
        names.push(CString::from_str(&name).unwrap());
        types.push(CString::from_str(&type_name).unwrap());
    }

    unsafe {
        (*names_and_types).names = rcutils_string_array_t::from(names);
        (*names_and_types).types = Box::into_raw(Box::new(rcutils_string_array_t::from(types)));
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_subscriber_names_and_types_by_node(
    node: *const rcl_node_t,
    _allocator: *mut rcl_allocator_t,
    _no_demangle: bool,
    remote_node_name: *const ::std::os::raw::c_char,
    remote_node_namespace: *const ::std::os::raw::c_char,
    subscriber_names_and_types: *mut rcl_names_and_types_t,
) -> rcl_ret_t {
    rcl_get_names_and_types_by_node_impl(node, remote_node_name, remote_node_namespace, subscriber_names_and_types, EntityKind::Subscription)
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_publisher_names_and_types_by_node(
    node: *const rcl_node_t,
    _allocator: *mut rcl_allocator_t,
    _no_demangle: bool,
    remote_node_name: *const ::std::os::raw::c_char,
    remote_node_namespace: *const ::std::os::raw::c_char,
    publisher_names_and_types: *mut rcl_names_and_types_t,
) -> rcl_ret_t {
    rcl_get_names_and_types_by_node_impl(node, remote_node_name, remote_node_namespace, publisher_names_and_types, EntityKind::Publisher)
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_service_names_and_types_by_node(
    node: *const rcl_node_t,
    _allocator: *mut rcl_allocator_t,
    remote_node_name: *const ::std::os::raw::c_char,
    remote_node_namespace: *const ::std::os::raw::c_char,
    service_names_and_types: *mut rcl_names_and_types_t,
) -> rcl_ret_t {
    rcl_get_names_and_types_by_node_impl(node, remote_node_name, remote_node_namespace, service_names_and_types, EntityKind::Service)
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_client_names_and_types_by_node(
    node: *const rcl_node_t,
    _allocator: *mut rcl_allocator_t,
    remote_node_name: *const ::std::os::raw::c_char,
    remote_node_namespace: *const ::std::os::raw::c_char,
    client_names_and_types: *mut rcl_names_and_types_t,
) -> rcl_ret_t {
    rcl_get_names_and_types_by_node_impl(node, remote_node_name, remote_node_namespace, client_names_and_types, EntityKind::Client)
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_publishers_info_by_topic(
    node: *const rcl_node_t,
    _allocator: *mut rcutils_allocator_t,
    topic_name: *const ::std::os::raw::c_char,
    _no_mangle: bool,
    publishers_info: *mut rcl_topic_endpoint_info_array_t,
) -> rcl_ret_t {
    rcl_get_entities_info_by_topic(node, topic_name, publishers_info, EntityKind::Publisher)
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_subscriptions_info_by_topic(
    node: *const rcl_node_t,
    _allocator: *mut rcutils_allocator_t,
    topic_name: *const ::std::os::raw::c_char,
    _no_mangle: bool,
    subscriptions_info: *mut rcl_topic_endpoint_info_array_t,
) -> rcl_ret_t {
    rcl_get_entities_info_by_topic(node, topic_name, subscriptions_info, EntityKind::Subscription)
}

fn rcl_get_entities_info_by_topic(
    node: *const rcl_node_t,
    topic_name: *const ::std::os::raw::c_char,
    entities_info: *mut rcl_topic_endpoint_info_array_t,
    kind: EntityKind,
) -> rcl_ret_t {
    if node.is_null() || topic_name.is_null() || entities_info.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_impl() {
        Ok(impl_) => impl_,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };

    let topic = match str_from_ptr(topic_name) {
        Ok(topic_str) => topic_str,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };

    // Get entities by topic
    let entities = node_impl
        .graph()
        .get_entities_by_topic(kind, topic);

    // Convert EndpointEntity to rmw_topic_endpoint_info_t
    let mut endpoint_infos = Vec::new();
    for entity in &entities {
        if let Some(endpoint) = entity.get_endpoint() {
            match rmw_topic_endpoint_info_t::try_from(endpoint.clone()) {
                Ok(info) => endpoint_infos.push(info),
                Err(_) => return RCL_RET_ERROR as _,
            }
        }
    }

    // Allocate memory for the array
    let len = endpoint_infos.len();
    let info_array = if endpoint_infos.is_empty() {
        ptr::null_mut()
    } else {
        let ptr = Box::into_raw(endpoint_infos.into_boxed_slice());
        ptr as *mut rmw_topic_endpoint_info_t
    };

    unsafe {
        (*entities_info).size = len;
        (*entities_info).info_array = info_array;
    }

    RCL_RET_OK as _
}

fn rcl_count_entities(
    node: *const rcl_node_t,
    name: *const ::std::os::raw::c_char,
    count: *mut usize,
    kind: EntityKind,
) -> rcl_ret_t {
    if node.is_null() || name.is_null() || count.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_impl() {
        Ok(impl_) => impl_,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };

    let name_str = match str_from_ptr(name) {
        Ok(s) => s,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };

    let entity_count = node_impl.graph().count(kind, name_str);

    unsafe {
        *count = entity_count;
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_count_publishers(
    node: *const rcl_node_t,
    topic_name: *const ::std::os::raw::c_char,
    count: *mut usize,
) -> rcl_ret_t {
    rcl_count_entities(node, topic_name, count, EntityKind::Publisher)
}


#[unsafe(no_mangle)]
pub extern "C" fn rcl_count_subscribers(
    node: *const rcl_node_t,
    topic_name: *const ::std::os::raw::c_char,
    count: *mut usize,
) -> rcl_ret_t {
    rcl_count_entities(node, topic_name, count, EntityKind::Subscription)
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_count_clients(
    node: *const rcl_node_t,
    service_name: *const ::std::os::raw::c_char,
    count: *mut usize,
) -> rcl_ret_t {
    rcl_count_entities(node, service_name, count, EntityKind::Client)
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_count_services(
    node: *const rcl_node_t,
    service_name: *const ::std::os::raw::c_char,
    count: *mut usize,
) -> rcl_ret_t {
    rcl_count_entities(node, service_name, count, EntityKind::Service)
}
