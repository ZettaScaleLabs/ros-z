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
use ros_z::entity::{EndpointEntity, EntityKind};
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

    let mut topics = Vec::new();
    let mut type_names = Vec::new();
    node_impl
        .graph()
        .data
        .lock()
        .visit_by_node(node_impl.inner.entity.key(), |ent| {
            if let Some(enp) = ent.get_endpoint() {
                topics.push(CString::from_str(&enp.topic).unwrap());
                type_names.push(CString::from_str(&enp.type_info.as_ref().unwrap().name).unwrap());
            }
        });

    unsafe {
        (*topic_names_and_types).names = rcutils_string_array_t::from(topics);
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
pub extern "C" fn rcl_get_publishers_info_by_topic(
    node: *const rcl_node_t,
    _allocator: *mut rcutils_allocator_t,
    topic_name: *const ::std::os::raw::c_char,
    _no_mangle: bool,
    publishers_info: *mut rcl_topic_endpoint_info_array_t,
) -> rcl_ret_t {
    // let node_impl = node.borrow_impl().unwrap();
    // let topic = str_from_ptr(topic_name).unwrap();
    // node_impl
    //     .graph()
    //     .data
    //     .lock()
    //     .get_entities_by_topic(EntityKind::Publisher, topic);
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
