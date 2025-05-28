use cxx::{ExternType, type_id};

unsafe impl ExternType for crate::ros::rosidl_message_type_support_t {
    type Id = type_id!("rosidl_message_type_support_t");
    type Kind = cxx::kind::Opaque;
}
unsafe impl ExternType for crate::ros::rosidl_service_type_support_t {
    type Id = type_id!("rosidl_service_type_support_t");
    type Kind = cxx::kind::Opaque;
}

#[cxx::bridge]
mod ffi {

    unsafe extern "C++" {
        include!("serde_bridge.h");

        type c_void = crate::c_void;
        type rosidl_message_type_support_t = crate::ros::rosidl_message_type_support_t;
        type rosidl_service_type_support_t = crate::ros::rosidl_service_type_support_t;

        #[namespace = "serde_bridge"]
        unsafe fn get_message_typesupport(
            ts: *const rosidl_message_type_support_t,
        ) -> *const rosidl_message_type_support_t;

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
        unsafe fn get_serialized_size(
            ts: *const rosidl_message_type_support_t,
            ros_message: *const c_void,
        ) -> usize;

        #[namespace = "serde_bridge"]
        unsafe fn get_message_name(ts: *const rosidl_message_type_support_t) -> String;

        #[namespace = "serde_bridge"]
        unsafe fn get_message_namespace(ts: *const rosidl_message_type_support_t) -> String;

        #[namespace = "serde_bridge"]
        unsafe fn get_service_typesupport(
            ts: *const rosidl_service_type_support_t,
        ) -> *const rosidl_service_type_support_t;

        #[namespace = "serde_bridge"]
        unsafe fn get_request_type_support(
            ts: *const rosidl_service_type_support_t,
        ) -> *const rosidl_message_type_support_t;

        #[namespace = "serde_bridge"]
        unsafe fn get_response_type_support(
            ts: *const rosidl_service_type_support_t,
        ) -> *const rosidl_message_type_support_t;
    }
}

use ffi::*;
use ros_z::entity::TypeInfo;

use crate::ros::rosidl_type_hash_t;

impl ToString for rosidl_type_hash_t {
    fn to_string(&self) -> String {
        const HASH_PREFIX: &'static str = "RIHS01_";
        let hex_str: String = self.value.iter().map(|b| format!("{:02x}", b)).collect();
        format!("{HASH_PREFIX}{hex_str}")
    }
}

#[derive(Debug, Clone, Copy)]
pub struct MessageTypeSupport {
    ptr: *const rosidl_message_type_support_t,
}

impl AsRef<rosidl_message_type_support_t> for MessageTypeSupport {
    fn as_ref(&self) -> &rosidl_message_type_support_t {
        unsafe {
            assert!(!self.ptr.is_null());
            &*(*self).ptr
        }
    }
}

impl MessageTypeSupport {
    pub fn new(type_support: *const rosidl_message_type_support_t) -> Self {
        let type_support = unsafe {
            let ts = get_message_typesupport(type_support);
            if ts.is_null() {
                tracing::error!("Failed to create the type support.");
            }
            ts
        };
        Self { ptr: type_support }
    }

    pub fn get_type_hash(&self) -> String {
        let th = unsafe {
            let type_hash = self
                .as_ref()
                .get_type_hash_func
                .expect("Failed to get_type_hash_func")(self.as_ref());
            assert!(!type_hash.is_null());
            *type_hash
        };
        th.to_string()
    }

    pub fn serialize_message(&self, ros_message: *const c_void, out: &mut Vec<u8>) {
        let res = unsafe { serialize_message(self.as_ref(), ros_message, out) };
        if !res {
            tracing::error!("Failed to run serialize_message");
        }
    }

    pub fn get_serialized_size(&self, ros_message: *const c_void) -> usize {
        unsafe { get_serialized_size(self.as_ref(), ros_message) }
    }

    pub fn deserialize_message(&self, data: &Vec<u8>, ros_message: *mut c_void) {
        let res = unsafe { deserialize_message(self.as_ref(), data, ros_message) };
        if !res {
            tracing::error!("Failed to run serialize_message");
        }
    }

    pub fn get_type_prefix(&self) -> String {
        let (name, namespace) = unsafe {
            let ts = self.as_ref();
            (get_message_name(ts), get_message_namespace(ts))
        };

        let ns = if namespace.is_empty() {
            ""
        } else {
            &(namespace + "::")
        };
        format!("{ns}dds_::{name}_")
    }

    pub fn get_type_info(&self) -> TypeInfo {
        TypeInfo::new(&self.get_type_prefix(), &self.get_type_hash())
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ServiceTypeSupport {
    ptr: *const rosidl_service_type_support_t,
    pub request: MessageTypeSupport,
    pub response: MessageTypeSupport,
}

impl AsRef<rosidl_service_type_support_t> for ServiceTypeSupport {
    fn as_ref(&self) -> &rosidl_service_type_support_t {
        unsafe {
            assert!(!self.ptr.is_null());
            &*(*self).ptr
        }
    }
}

impl ServiceTypeSupport {
    pub fn new(type_support: *const rosidl_service_type_support_t) -> Self {
        let type_support = unsafe {
            let ts = get_service_typesupport(type_support);
            if ts.is_null() {
                tracing::error!("Failed to create the type support.");
            }
            ts
        };
        let (request, response) = unsafe {
            (
                get_request_type_support(type_support),
                get_response_type_support(type_support),
            )
        };
        Self {
            ptr: type_support,
            request: MessageTypeSupport::new(request),
            response: MessageTypeSupport::new(response),
        }
    }

    pub fn get_type_hash(&self) -> String {
        let th = unsafe {
            let type_hash = self
                .as_ref()
                .get_type_hash_func
                .expect("Failed to get_type_hash_func")(self.as_ref());
            assert!(!type_hash.is_null());
            *type_hash
        };
        th.to_string()
    }

    pub fn get_type_info(&self) -> TypeInfo {
        let name_with_suffix = self.response.get_type_prefix();
        let name = name_with_suffix
            .strip_suffix("Response_")
            .expect("Invalid Response_ type");
        TypeInfo::new(name, &self.get_type_hash())
    }
}
