// use crate::ros::rosidl_message_type_support_t;
use cxx::{ExternType, type_id};

unsafe impl ExternType for crate::ros::rosidl_message_type_support_t {
    type Id = type_id!("rosidl_message_type_support_t");
    type Kind = cxx::kind::Opaque;
}

#[cxx::bridge]
mod ffi {

    unsafe extern "C++" {
        include!("serde_bridge.h");

        type c_void = crate::c_void;
        type rosidl_message_type_support_t = crate::ros::rosidl_message_type_support_t;

        #[namespace = "serde_bridge"]
        unsafe fn serialize_message(
            ts: *const rosidl_message_type_support_t,
            ros_message: *const c_void,
            out: &mut Vec<u8>,
        ) -> bool;

        #[namespace = "serde_bridge"]
        unsafe fn deserialize_message(
            ts: *const rosidl_message_type_support_t,
            data: &Vec<u8>,
            ros_message: *mut c_void,
        ) -> bool;

        #[namespace = "serde_bridge"]
        unsafe fn get_message_typesupport(
            ts: *const rosidl_message_type_support_t,
        ) -> *const rosidl_message_type_support_t;

        #[namespace = "serde_bridge"]
        unsafe fn get_serialized_size(
            ts: *const rosidl_message_type_support_t,
            ros_message: *const c_void,
        ) -> usize;

        #[namespace = "serde_bridge"]
        unsafe fn get_message_name(ts: *const rosidl_message_type_support_t) -> String;

        #[namespace = "serde_bridge"]
        unsafe fn get_message_namespace(ts: *const rosidl_message_type_support_t) -> String;
    }
}

use ffi::*;

#[derive(Debug, Clone, Copy)]
pub struct TypeSupport {
    type_support: *const rosidl_message_type_support_t,
}

impl TypeSupport {
    pub fn new(type_support: *const rosidl_message_type_support_t) -> Self {
        let type_support = unsafe {
            let ts = get_message_typesupport(type_support);
            if ts.is_null() {
                tracing::error!("Failed to create the type support.");
            }
            ts
        };
        Self { type_support }
    }

    pub fn get_type_hash(&self) -> String {
        const HASH_PREFIX: &'static str = "RIHS01_";
        unsafe {
            let type_hash = (*self.type_support).get_type_hash_func.unwrap()(self.type_support);
            let hex_str: String = (*type_hash)
                .value
                .iter()
                .map(|b| format!("{:02x}", b))
                .collect();
            format!("{HASH_PREFIX}{hex_str}")
        }
    }

    pub fn serialize_message(&self, ros_message: *const c_void, out: &mut Vec<u8>) {
        let res = unsafe { serialize_message(self.type_support, ros_message, out) };
        if !res {
            tracing::error!("Failed to run serialize_message");
        }
    }

    pub fn get_serialized_size(&self, ros_message: *const c_void) -> usize {
        unsafe { get_serialized_size(self.type_support, ros_message) }
    }

    pub fn deserialize_message(&self, data: &Vec<u8>, ros_message: *mut c_void) {
        let res = unsafe { deserialize_message(self.type_support, data, ros_message) };
        if !res {
            tracing::error!("Failed to run serialize_message");
        }
    }

    pub fn get_type_prefix(&self) -> String {
        let (name, namespace) = unsafe {
            let ts = (*self).type_support;
            ( get_message_name(ts), get_message_namespace(ts) )
        };

        let ns = if namespace.is_empty() {
            ""
        } else {
            &(namespace + "::")
        };
        format!("{ns}dds_::{name}_")
    }
}
