use std::{ffi::CString, sync::Arc};

use crate::{c_void, traits::Waitable, utils::Notifier};

use ros_z::{
    attachment::{Attachment, GidArray},
    service::{QueryKey, ZClient, ZServer},
};
use zenoh::Result;

use crate::{
    impl_has_impl_ptr,
    msg::{RosMessage, RosService},
    ros::*,
    traits::{BorrowImpl, OwnImpl},
    type_support::ServiceTypeSupport,
};

pub struct ClientImpl {
    pub(crate) inner: ZClient<RosService>,
    pub(crate) ts: ServiceTypeSupport,
    pub(crate) notifier: Arc<Notifier>,
    pub(crate) service_name: String,
    pub(crate) options: rcl_client_options_t,
}

pub struct ServiceImpl {
    pub(crate) inner: ZServer<RosService>,
    pub(crate) ts: ServiceTypeSupport,
    pub(crate) service_name: CString,
}

impl ServiceImpl {
    pub unsafe fn take_request(&mut self, ros_request: *mut c_void) -> Result<RequestId> {
        let query = self.inner.take_query().unwrap();

        let attachment: Attachment = query.attachment().unwrap().try_into().unwrap();
        let gid = attachment.source_gid;
        let sn = attachment.sequence_number;
        let key: QueryKey = attachment.into();
        if self.inner.map.contains_key(&key) {
            tracing::error!("Existing query detected");
            return Err("Existing query detected".into());
        }

        let bytes = query.payload().unwrap().to_bytes();
        unsafe {
            self.ts
                .request
                .deserialize_message(&bytes.to_vec(), ros_request);
        }
        self.inner.map.insert(key.clone(), query);

        Ok(RequestId { sn, gid })
    }

    pub unsafe fn send_response(
        &mut self,
        ros_response: *mut c_void,
        request_id: *mut rmw_request_id_t,
    ) -> Result<()> {
        let key = unsafe {
            QueryKey {
                sn: (*request_id).sequence_number,
                gid: (*request_id).writer_guid,
            }
        };

        let resp = RosMessage::new(ros_response, self.ts.response);
        self.inner.send_response(&resp, &key)
    }
}

impl Waitable for ServiceImpl {
    fn is_ready(&self) -> bool {
        !self.inner.rx().is_empty()
    }
}

pub struct RequestId {
    sn: i64,
    gid: GidArray,
}

impl ClientImpl {
    pub fn send_request(&self, ros_request: *const c_void) -> Result<i64> {
        let req = RosMessage::new(ros_request, self.ts.request);
        let c_notifier = self.notifier.clone();
        let notify = move || {
            c_notifier.notify_all();
        };
        self.inner.rcl_send_request(&req, notify)
    }

    pub unsafe fn take_response(&self, ros_response: *mut c_void) -> Result<RequestId> {
        let sample = self.inner.take_sample()?;
        let attachment: Attachment = sample.attachment().unwrap().try_into()?;
        let bytes = sample.payload().to_bytes();
        unsafe {
            self.ts
                .response
                .deserialize_message(&bytes.to_vec(), ros_response);
        }
        Ok(RequestId {
            sn: attachment.sequence_number,
            gid: attachment.source_gid,
        })
    }
}

impl Waitable for ClientImpl {
    fn is_ready(&self) -> bool {
        !self.inner.rx.is_empty()
    }
}

impl_has_impl_ptr!(rcl_client_t, rcl_client_impl_t, ClientImpl);
impl_has_impl_ptr!(rcl_service_t, rcl_service_impl_t, ServiceImpl);

#[unsafe(no_mangle)]
#[allow(unsafe_op_in_unsafe_fn)]
pub unsafe extern "C" fn rcl_client_init(
    client: *mut rcl_client_t,
    node: *const rcl_node_t,
    type_support: *const rosidl_service_type_support_t,
    service_name: *const ::std::os::raw::c_char,
    _options: *const rcl_client_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_client_init");

    // Validate input parameters - check null first, with specific checks for node
    if client.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check node separately for better error reporting (must be before other checks to match C++ behavior)
    if node.is_null() {
        return RCL_RET_NODE_INVALID as _;
    }

    if type_support.is_null() || service_name.is_null() || _options.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check allocator
    if unsafe { (*_options).allocator.allocate.is_none() || (*_options).allocator.deallocate.is_none() } {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Validate service name
    unsafe {
        let mut validation_result: i32 = 0;
        let ret = crate::validate_topic_name::rcl_validate_topic_name(
            service_name,
            &mut validation_result,
            std::ptr::null_mut(),
        );
        if ret != RCL_RET_OK as i32 {
            return ret;
        }
        if validation_result != crate::validate_topic_name::RCL_TOPIC_NAME_VALID {
            return RCL_RET_SERVICE_NAME_INVALID as _;
        }
    }

    let x = move || {
        let cli_impl = unsafe {
            node.borrow_impl()?
                .new_client(type_support, service_name, _options)?
        };
        Result::Ok(cli_impl)
    };

    // First create the client impl
    let cli_impl = match x() {
        Ok(impl_) => impl_,
        Err(e) => {
            let error_msg = format!("{e}");
            tracing::error!("{error_msg}");
            // Check if it's because the node is not valid (zero-initialized)
            if node.borrow_impl().is_err() {
                return RCL_RET_NODE_INVALID as _;
            }
            // Check if it's a service name validation error
            if error_msg.contains("Topic name contains invalid characters")
                || error_msg.contains("invalid component")
            {
                return RCL_RET_SERVICE_NAME_INVALID as _;
            }
            return RCL_RET_ERROR as _;
        }
    };

    // Then assign it, checking for already initialized
    match client.assign_impl(cli_impl) {
        Ok(_) => {
            // Trigger graph guard condition for this node
            if let Ok(node_impl) = (node as *mut rcl_node_t).borrow_mut_impl() {
                node_impl.trigger_graph_guard_condition();
            }
            RCL_RET_OK as _
        }
        Err(crate::traits::ImplAccessError::NonNullImplPtr) => RCL_RET_ALREADY_INIT as _,
        Err(e) => {
            tracing::error!("{e}");
            RCL_RET_ERROR as _
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_client_fini(client: *mut rcl_client_t, node: *mut rcl_node_t) -> rcl_ret_t {
    // Validate input parameters
    if client.is_null() {
        return RCL_RET_CLIENT_INVALID as _;
    }
    // Check if node is valid (not just null)
    if !crate::node::rcl_node_is_valid_except_context(node) {
        return RCL_RET_NODE_INVALID as _;
    }
    // Drop the data regardless of the pointer's condition.
    drop(client.own_impl());

    // Trigger graph guard condition for this node
    if let Ok(node_impl) = node.borrow_mut_impl() {
        node_impl.trigger_graph_guard_condition();
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_send_request(
    client: *const rcl_client_t,
    ros_request: *const c_void,
    sequence_number: *mut i64,
) -> rcl_ret_t {
    if client.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if sequence_number.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    match client.borrow_impl() {
        Ok(impl_) => match impl_.send_request(ros_request) {
            Ok(sn) => {
                unsafe {
                    *sequence_number = sn;
                }
                RCL_RET_OK as _
            }
            Err(_) => RCL_RET_ERROR as _,
        },
        Err(_) => RCL_RET_CLIENT_INVALID as _,
    }
}

#[unsafe(no_mangle)]
#[allow(unsafe_op_in_unsafe_fn)]
pub unsafe extern "C" fn rcl_take_response(
    client: *const rcl_client_t,
    request_header: *mut rmw_request_id_t,
    ros_response: *mut c_void,
) -> rcl_ret_t {
    if client.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check if client is valid before checking other arguments
    let client_impl = match client.borrow_impl() {
        Ok(impl_) => impl_,
        Err(_) => return RCL_RET_CLIENT_INVALID as _,
    };

    if request_header.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if ros_response.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let request_id = match unsafe { client_impl.take_response(ros_response) } {
        Ok(id) => id,
        Err(_) => return RCL_RET_CLIENT_TAKE_FAILED as _,
    };

    unsafe {
        (*request_header).sequence_number = request_id.sn;
        (*request_header).writer_guid = request_id.gid;
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_take_response_with_info(
    client: *const rcl_client_t,
    request_header: *mut rmw_service_info_t,
    ros_response: *mut c_void,
) -> rcl_ret_t {
    if client.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check if client is valid before checking other arguments
    let client_impl = match client.borrow_impl() {
        Ok(impl_) => impl_,
        Err(_) => return RCL_RET_CLIENT_INVALID as _,
    };

    if request_header.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if ros_response.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let request_id = match unsafe { client_impl.take_response(ros_response) } {
        Ok(id) => id,
        Err(_) => return RCL_RET_CLIENT_TAKE_FAILED as _,
    };

    unsafe {
        (*request_header).request_id.sequence_number = request_id.sn;
        (*request_header).request_id.writer_guid = request_id.gid;
        // Set timestamps to 0 for now (not currently supported)
        (*request_header).source_timestamp = 0;
        (*request_header).received_timestamp = 0;
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_take_request_with_info(
    service: *const rcl_service_t,
    request_header: *mut rmw_service_info_t,
    ros_request: *mut ::std::os::raw::c_void,
) -> rcl_ret_t {
    tracing::trace!("rcl_take_request_with_info");

    if service.is_null() {
        return RCL_RET_SERVICE_INVALID as _;
    }
    if request_header.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if ros_request.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Interior mutability is not allowed in Rust
    let service_mut = service as *mut rcl_service_t;
    let service_impl = match service_mut.borrow_mut_impl() {
        Ok(impl_) => impl_,
        Err(_) => return RCL_RET_SERVICE_INVALID as _,
    };

    let request_id = match unsafe { service_impl.take_request(ros_request as *mut c_void) } {
        Ok(id) => id,
        Err(_) => return RCL_RET_SERVICE_TAKE_FAILED as _,
    };

    unsafe {
        (*request_header).request_id.sequence_number = request_id.sn;
        (*request_header).request_id.writer_guid = request_id.gid;
        // Set timestamps to 0 for now (not currently supported)
        (*request_header).source_timestamp = 0;
        (*request_header).received_timestamp = 0;
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_service_server_is_available(
    node: *const rcl_node_t,
    client: *const rcl_client_t,
    is_available: *mut bool,
) -> rcl_ret_t {
    tracing::trace!("rcl_service_server_is_available");
    if node.is_null() {
        return RCL_RET_NODE_INVALID as _;
    }
    if client.is_null() || is_available.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    // SAFETY: is_available is checked to be non-null above
    unsafe { *is_available = false; }
    let node_impl = match node.borrow_impl() {
        Ok(n) => n,
        Err(_) => return RCL_RET_NODE_INVALID as _,
    };
    if let Ok(client_impl) = client.borrow_impl() {
        let service_names = node_impl.graph().get_service_names_and_types();
        let available = service_names.iter().any(|(name, _)| name == &client_impl.service_name);
        // SAFETY: is_available is checked to be non-null above
        unsafe { *is_available = available; }
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_type_description_service_handle_request(
    _node: *mut rcl_node_t,
    _request_header: *const rmw_request_id_t,
    _request: *const type_description_interfaces__srv__GetTypeDescription_Request,
    _response: *mut type_description_interfaces__srv__GetTypeDescription_Response,
) {
    tracing::trace!("rcl_node_type_description_service_handle_request");
    todo!()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_service_init(
    service: *mut rcl_service_t,
    node: *const rcl_node_t,
    type_support: *const rosidl_service_type_support_t,
    service_name: *const ::std::os::raw::c_char,
    _options: *const rcl_service_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_service_init: {service:?}");

    // Validate input parameters - check null first, with specific checks for node
    if service.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check node separately for better error reporting (must be before other checks to match C++ behavior)
    if node.is_null() {
        return RCL_RET_NODE_INVALID as _;
    }

    if type_support.is_null() || service_name.is_null() || _options.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Check allocator
    if unsafe { (*_options).allocator.allocate.is_none() || (*_options).allocator.deallocate.is_none() } {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Validate service name
    unsafe {
        let mut validation_result: i32 = 0;
        let ret = crate::validate_topic_name::rcl_validate_topic_name(
            service_name,
            &mut validation_result,
            std::ptr::null_mut(),
        );
        if ret != RCL_RET_OK as i32 {
            return ret;
        }
        if validation_result != crate::validate_topic_name::RCL_TOPIC_NAME_VALID {
            return RCL_RET_SERVICE_NAME_INVALID as _;
        }
    }

    let x = move || {
        let srv_impl = unsafe {
            node.borrow_impl()?
                .new_service(type_support, service_name, _options)
        }?;
        Result::Ok(srv_impl)
    };

    // First create the service impl
    let srv_impl = match x() {
        Ok(impl_) => impl_,
        Err(e) => {
            let error_msg = format!("{e}");
            tracing::error!("{error_msg}");
            // Check if it's because the node is not valid (zero-initialized)
            if node.borrow_impl().is_err() {
                return RCL_RET_NODE_INVALID as _;
            }
            // Check if it's a service name validation error
            if error_msg.contains("Topic name contains invalid characters")
                || error_msg.contains("invalid component")
            {
                return RCL_RET_SERVICE_NAME_INVALID as _;
            }
            return RCL_RET_ERROR as _;
        }
    };

    // Then assign it, checking for already initialized
    match service.assign_impl(srv_impl) {
        Ok(_) => {
            // Trigger graph guard condition for this node
            if let Ok(node_impl) = (node as *mut rcl_node_t).borrow_mut_impl() {
                node_impl.trigger_graph_guard_condition();
            }
            RCL_RET_OK as _
        }
        Err(crate::traits::ImplAccessError::NonNullImplPtr) => RCL_RET_ALREADY_INIT as _,
        Err(e) => {
            tracing::error!("{e}");
            RCL_RET_ERROR as _
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_fini(
    service: *mut rcl_service_t,
    _node: *mut rcl_node_t,
) -> rcl_ret_t {
    // Validate input parameters
    if service.is_null() {
        return RCL_RET_SERVICE_INVALID as _;
    }
    if _node.is_null() {
        return RCL_RET_NODE_INVALID as _;
    }
    // Drop the data regardless of the pointer's condition.
    drop(service.own_impl());

    // Trigger graph guard condition for this node
    if let Ok(node_impl) = _node.borrow_mut_impl() {
        node_impl.trigger_graph_guard_condition();
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_take_request(
    service: *const rcl_service_t,
    request_header: *mut rmw_request_id_t,
    ros_request: *mut c_void,
) -> rcl_ret_t {
    tracing::trace!("rcl_take_request: {service:?}");

    if service.is_null() {
        return RCL_RET_SERVICE_INVALID as _;
    }
    if request_header.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if ros_request.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Interior mutability is not allowed in Rust
    let service_impl = match (service as *mut rcl_service_t).borrow_mut_impl() {
        Ok(impl_) => impl_,
        Err(_) => return RCL_RET_SERVICE_INVALID as _,
    };

    let request_id = match unsafe { service_impl.take_request(ros_request) } {
        Ok(id) => id,
        Err(_) => return RCL_RET_SERVICE_TAKE_FAILED as _,
    };

    unsafe {
        (*request_header).sequence_number = request_id.sn;
        (*request_header).writer_guid = request_id.gid;
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub unsafe fn rcl_send_response(
    service: *const rcl_service_t,
    response_header: *mut rmw_request_id_t,
    ros_response: *mut c_void,
) -> rcl_ret_t {
    if service.is_null() {
        return RCL_RET_SERVICE_INVALID as _;
    }
    if response_header.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if ros_response.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Interior mutability is not allowed in Rust
    let service_impl = match (service as *mut rcl_service_t).borrow_mut_impl() {
        Ok(impl_) => impl_,
        Err(_) => return RCL_RET_SERVICE_INVALID as _,
    };

    match unsafe { service_impl.send_response(ros_response, response_header) } {
        Ok(_) => RCL_RET_OK as _,
        Err(_) => RCL_RET_ERROR as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_is_valid(_service: *const rcl_service_t) -> bool {
    _service.borrow_impl().is_ok()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_client_is_valid(_client: *const rcl_client_t) -> bool {
    _client.borrow_impl().is_ok()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_get_service_name(
    _service: *const rcl_service_t,
) -> *const ::std::os::raw::c_char {
    if _service.is_null() {
        return std::ptr::null();
    }
    match _service.borrow_impl() {
        Ok(impl_) => impl_.service_name.as_ptr(),
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_client_get_default_options() -> rcl_client_options_t {
    rcl_client_options_t {
        qos: rmw_qos_profile_t::default(),
        allocator: crate::init::rcl_get_default_allocator(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_client() -> rcl_client_t {
    unsafe { std::mem::zeroed() }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_service() -> rcl_service_t {
    unsafe { std::mem::zeroed() }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_get_default_options() -> rcl_service_options_t {
    rcl_service_options_t {
        qos: rmw_qos_profile_t::default(),
        allocator: crate::init::rcl_get_default_allocator(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_get_options(
    _service: *const rcl_service_t,
) -> *const rcl_service_options_t {
    if _service.is_null() {
        return std::ptr::null();
    }
    // In the current implementation, options are not stored in the service
    // Return null for now to indicate options are not available
    std::ptr::null()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_get_rmw_handle(
    _service: *const rcl_service_t,
) -> *const c_void {
    if _service.is_null() {
        return std::ptr::null();
    }
    match _service.borrow_impl() {
        Ok(_) => {
            // In the current implementation, we don't expose the rmw handle
            // Return null for now
            std::ptr::null()
        }
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_request_subscription_get_actual_qos(
    _service: *const rcl_service_t,
) -> *const rmw_qos_profile_t {
    if _service.is_null() {
        return std::ptr::null();
    }
    match _service.borrow_impl() {
        Ok(_) => {
            // In the current implementation, we don't store QoS profiles
            // Return null for now
            std::ptr::null()
        }
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_response_publisher_get_actual_qos(
    _service: *const rcl_service_t,
) -> *const rmw_qos_profile_t {
    if _service.is_null() {
        return std::ptr::null();
    }
    match _service.borrow_impl() {
        Ok(_) => {
            // In the current implementation, we don't store QoS profiles
            // Return null for now
            std::ptr::null()
        }
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_client_get_options(
    _client: *const rcl_client_t,
) -> *const rcl_client_options_t {
    if _client.is_null() {
        return std::ptr::null();
    }
    match _client.borrow_impl() {
        Ok(impl_) => &impl_.options,
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_client_get_rmw_handle(
    _client: *const rcl_client_t,
) -> *const c_void {
    if _client.is_null() {
        return std::ptr::null();
    }
    match _client.borrow_impl() {
        Ok(_) => {
            // In the current implementation, we don't expose the rmw handle
            // Return null for now
            std::ptr::null()
        }
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_client_get_service_name(
    client: *const rcl_client_t,
) -> *const ::std::os::raw::c_char {
    if client.is_null() {
        return std::ptr::null();
    }
    match client.borrow_impl() {
        Ok(impl_) => impl_.service_name.as_ptr() as *const ::std::os::raw::c_char,
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_client_request_publisher_get_actual_qos(
    _client: *const rcl_client_t,
) -> *const rmw_qos_profile_t {
    if _client.is_null() {
        return std::ptr::null();
    }
    match _client.borrow_impl() {
        Ok(impl_) => &impl_.options.qos,
        Err(_) => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_client_response_subscription_get_actual_qos(
    _client: *const rcl_client_t,
) -> *const rmw_qos_profile_t {
    if _client.is_null() {
        return std::ptr::null();
    }
    match _client.borrow_impl() {
        Ok(impl_) => &impl_.options.qos,
        Err(_) => std::ptr::null(),
    }
}
