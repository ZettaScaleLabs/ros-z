use std::sync::Arc;

use crate::{c_void, ros::*, traits::Waitable, utils::Notifier};

use ros_z::{
    attachment::{Attachment, GidArray},
    service::{QueryKey, ZClient, ZServer},
};
use zenoh::Result;

use crate::{
    impl_has_impl_ptr,
    msg::{RosMessage, RosService},
    rclz_try,
    ros::*,
    traits::{BorrowImpl, OwnImpl},
    type_support::ServiceTypeSupport,
};

pub struct ClientImpl {
    pub(crate) inner: ZClient<RosService>,
    pub(crate) ts: ServiceTypeSupport,
    pub(crate) notifier: Arc<Notifier>,
}

pub struct ServiceImpl {
    pub(crate) inner: ZServer<RosService>,
    pub(crate) ts: ServiceTypeSupport,
}

impl ServiceImpl {
    pub fn take_request(&mut self, ros_request: *mut c_void) -> Result<RequestId> {
        let query = self.inner.take_query().unwrap();

        let attachment: Attachment = query.attachment().unwrap().try_into().unwrap();
        let gid = attachment.source_gid.clone();
        let sn = attachment.sequence_number.clone();
        let key: QueryKey = attachment.into();
        if self.inner.map.contains_key(&key) {
            tracing::error!("Existing query detected");
            return Err("Existing query detected".into());
        }

        let bytes = query.payload().unwrap().to_bytes();
        self.ts
            .request
            .deserialize_message(&bytes.to_vec(), ros_request);
        self.inner.map.insert(key.clone(), query);

        Ok(RequestId { sn, gid })
    }

    pub fn send_response(
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
        self.inner.rx.len() > 0
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

    pub fn take_response(&self, ros_response: *mut c_void) -> Result<RequestId> {
        let sample = self.inner.take_sample()?;
        let attachment: Attachment = sample.attachment().unwrap().try_into()?;
        let bytes = sample.payload().to_bytes();
        self.ts
            .response
            .deserialize_message(&bytes.to_vec(), ros_response);
        Ok(RequestId {
            sn: attachment.sequence_number,
            gid: attachment.source_gid,
        })
    }
}

impl Waitable for ClientImpl {
    fn is_ready(&self) -> bool {
        self.inner.rx.len() > 0
    }
}

impl_has_impl_ptr!(rcl_client_t, rcl_client_impl_t, ClientImpl);
impl_has_impl_ptr!(rcl_service_t, rcl_service_impl_t, ServiceImpl);

#[unsafe(no_mangle)]
pub extern "C" fn rcl_client_init(
    client: *mut rcl_client_t,
    node: *const rcl_node_t,
    type_support: *const rosidl_service_type_support_t,
    service_name: *const ::std::os::raw::c_char,
    _options: *const rcl_client_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_client_init");

    let x = move || {
        let cli_impl = node
            .borrow_impl()?
            .new_client(type_support, service_name, _options)?;
        client.assign_impl(cli_impl)?;
        Result::Ok(())
    };
    rclz_try! { x()?; }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_client_fini(client: *mut rcl_client_t, _node: *mut rcl_node_t) -> rcl_ret_t {
    // Drop the data regardless of the pointer's condition.
    drop(client.own_impl());
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_send_request(
    client: *const rcl_client_t,
    ros_request: *const c_void,
    sequence_number: *mut i64,
) -> rcl_ret_t {
    let sn = client
        .borrow_impl()
        .unwrap()
        .send_request(ros_request)
        .unwrap();
    unsafe {
        *sequence_number = sn;
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_take_response(
    client: *const rcl_client_t,
    request_header: *mut rmw_request_id_t,
    ros_response: *mut c_void,
) -> rcl_ret_t {
    let request_id = client
        .borrow_impl()
        .unwrap()
        .take_response(ros_response)
        .unwrap();
    unsafe {
        (*request_header).sequence_number = request_id.sn;
        (*request_header).writer_guid = request_id.gid;
    }
    RCL_RET_OK as _
}

macro_rules! impl_trivial_ok_fn {
    (
        pub fn $fn_name:ident (
            $( $arg_name:ident : $arg_ty:ty ),* $(,)?
        ) -> $ret:ty ;
    ) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_name(
            $( $arg_name : $arg_ty ),*
        ) -> $ret {
            tracing::trace!(stringify!($fn_name));
            RCL_RET_OK as _
        }
    };
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_take_request_with_info(
    _service: *const rcl_service_t,
    _request_header: *mut rmw_service_info_t,
    _ros_request: *mut ::std::os::raw::c_void,
) -> rcl_ret_t {
    tracing::trace!("rcl_take_request_with_info");
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_server_is_available(
    _node: *const rcl_node_t,
    _client: *const rcl_client_t,
    is_available: *mut bool,
) -> rcl_ret_t {
    tracing::trace!("rcl_service_server_is_available");
    unsafe {
        *is_available = true;
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
pub extern "C" fn rcl_service_init(
    service: *mut rcl_service_t,
    node: *const rcl_node_t,
    type_support: *const rosidl_service_type_support_t,
    service_name: *const ::std::os::raw::c_char,
    _options: *const rcl_service_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_service_init: {service:?}");
    let x = move || {
        let srv_impl = node
            .borrow_impl()?
            .new_service(type_support, service_name, _options)?;
        service.assign_impl(srv_impl)?;
        Result::Ok(())
    };
    rclz_try! { x()?; }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_fini(
    service: *mut rcl_service_t,
    _node: *mut rcl_node_t,
) -> rcl_ret_t {
    // Drop the data regardless of the pointer's condition.
    drop(service.own_impl());
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_take_request(
    service: *const rcl_service_t,
    request_header: *mut rmw_request_id_t,
    ros_request: *mut c_void,
) -> rcl_ret_t {
    tracing::trace!("rcl_take_request: {service:?}");

    // Interior mutability is not allowed in Rust
    let x = (service as *mut rcl_service_t).borrow_mut_impl().unwrap();
    let request_id = x.take_request(ros_request).unwrap();
    unsafe {
        (*request_header).sequence_number = request_id.sn;
        (*request_header).writer_guid = request_id.gid;
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub fn rcl_send_response(
    service: *const rcl_service_t,
    response_header: *mut rmw_request_id_t,
    ros_response: *mut c_void,
) -> rcl_ret_t {
    // Interior mutability is not allowed in Rust
    let x = (service as *mut rcl_service_t).borrow_mut_impl().unwrap();
    x.send_response(ros_response, response_header).unwrap();
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_is_valid(service: *const rcl_service_t) -> bool {
    service.borrow_impl().is_ok()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_get_service_name(
    service: *const rcl_service_t,
) -> *const ::std::os::raw::c_char {
    c"rcl_service_get_service_name not yet implemented".as_ptr()
}
