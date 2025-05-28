#![allow(unused)]

use std::{sync::Arc, time::Duration};

use crate::{c_void, ros::*, utils::Notifier};

use ros_z::{
    Builder,
    attachment::{Attachment, GidArray},
    entity::TypeInfo,
    msg::ZService,
    service::{QueryKey, ZClient, ZServer},
};
use zenoh::Result;

use crate::{
    impl_has_impl_ptr,
    msg::{RosMessage, RosService},
    rclz_try,
    ros::*,
    traits::{BorrowImpl, OwnImpl},
    type_support::{MessageTypeSupport, ServiceTypeSupport},
    utils::str_from_ptr,
};

pub struct ClientImpl {
    zcli: ZClient<RosService>,
    ts: ServiceTypeSupport,
    notifier: Arc<Notifier>,
}

pub struct ServiceImpl {
    zsrv: ZServer<RosService>,
    ts: ServiceTypeSupport,
    notifier: Arc<Notifier>,
}

impl ServiceImpl {
    pub fn new(zsrv: ZServer<RosService>, ts: ServiceTypeSupport, notifier: Arc<Notifier>) -> Self {
        Self { zsrv, ts, notifier }
    }

    pub fn take_request(&mut self, ros_request: *mut c_void) -> Result<RequestId> {
        let query = self.zsrv.take_query().unwrap();

        let attachment: Attachment = query.attachment().unwrap().try_into().unwrap();
        let gid = attachment.source_gid.clone();
        let sn = attachment.sequence_number.clone();
        let key: QueryKey = attachment.into();
        if self.zsrv.map.contains_key(&key) {
            tracing::error!("Existing query detected");
            return Err("Existing query detected".into());
        }

        let bytes = query.payload().unwrap().to_bytes();
        self.ts
            .request
            .deserialize_message(&bytes.to_vec(), ros_request);
        self.zsrv.map.insert(key.clone(), query);

        Ok(RequestId { sn, gid })
    }

    pub fn send_response(&mut self, ros_response: *mut c_void, request_id: *mut rmw_request_id_t,) -> Result<()> {
        let key = unsafe {
            QueryKey { sn: (*request_id).sequence_number, gid: (*request_id).writer_guid }
        };

        let resp = RosMessage::new(ros_response, self.ts.response);
        self.zsrv.send_response(&resp, &key)
    }

    pub fn wait(&self, timeout: Duration) -> bool {
        if self.zsrv.rx.len() > 0 {
            return true;
        }

        let mut started = self.notifier.mutex.lock();
        if self.zsrv.rx.len() == 0 && !*started {
            !self.notifier.cv.wait_for(&mut started, timeout).timed_out()
        } else {
            true
        }
    }
}

pub struct RequestId {
    sn: i64,
    gid: GidArray,
}

impl ClientImpl {
    pub fn new(zcli: ZClient<RosService>, ts: ServiceTypeSupport) -> Self {
        Self {
            zcli,
            ts,
            notifier: Arc::new(Notifier::default()),
        }
    }

    pub fn send_request(&self, ros_request: *const c_void) -> Result<i64> {
        let req = RosMessage::new(ros_request, self.ts.request);
        let notifier = self.notifier.clone();
        let notify = move || {
            let mut started = notifier.mutex.lock();
            *started = true;
            notifier.cv.notify_one();
        };
        self.zcli.rcl_send_request(&req, notify)
    }

    pub fn take_response(&self, ros_response: *mut c_void) -> Result<RequestId> {
        let sample = self.zcli.take_sample()?;
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

    pub fn wait(&self, timeout: Duration) -> bool {
        if self.zcli.rx.len() > 0 {
            return true;
        }

        let mut started = self.notifier.mutex.lock();
        if self.zcli.rx.len() == 0 && !*started {
            !self.notifier.cv.wait_for(&mut started, timeout).timed_out()
        } else {
            true
        }
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

    let ts = ServiceTypeSupport::new(type_support);

    rclz_try! {
        let topic = str_from_ptr(service_name)?;
        let zcli = node.borrow_impl()?
            .create_client(topic)
            .with_type_info(ts.get_type_info())
            .build()?;
        client
            .assign_impl(ClientImpl::new(zcli, ts))?;
    }
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

impl_trivial_ok_fn! {
    pub fn rcl_take_request_with_info(
        service: *const rcl_service_t,
        request_header: *mut rmw_service_info_t,
        ros_request: *mut ::std::os::raw::c_void,
    ) -> rcl_ret_t;
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_server_is_available(
    node: *const rcl_node_t,
    client: *const rcl_client_t,
    is_available: *mut bool,
) -> rcl_ret_t {
    unsafe {
        *is_available = true;
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_node_type_description_service_handle_request(
    node: *mut rcl_node_t,
    request_header: *const rmw_request_id_t,
    request: *const type_description_interfaces__srv__GetTypeDescription_Request,
    response: *mut type_description_interfaces__srv__GetTypeDescription_Response,
) {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_service_init(
    service: *mut rcl_service_t,
    node: *const rcl_node_t,
    type_support: *const rosidl_service_type_support_t,
    service_name: *const ::std::os::raw::c_char,
    options: *const rcl_service_options_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_service_init");

    let ts = ServiceTypeSupport::new(type_support);

    let notifier = Arc::new(Notifier::default());
    let c_notifier = notifier.clone();
    let notify = move || {
        let mut started = c_notifier.mutex.lock();
        *started = true;
        c_notifier.cv.notify_one();
    };
    rclz_try! {
        let topic = str_from_ptr(service_name)?;
        let zsrv = node.borrow_impl()?
            .create_service(topic)
            .with_type_info(ts.get_type_info())
            .build_with_notifier(notify)?;
        service
            .assign_impl(ServiceImpl::new(zsrv, ts, notifier))?;
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_take_request(
    service: *const rcl_service_t,
    request_header: *mut rmw_request_id_t,
    ros_request: *mut c_void,
) -> rcl_ret_t {
    // NOTE: service actually requires the mutability! ¯\_(ツ)_/¯
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
    // NOTE: service actually requires the mutability! ¯\_(ツ)_/¯
    let x = (service as *mut rcl_service_t).borrow_mut_impl().unwrap();
    x.send_response(ros_response, response_header).unwrap();
    RCL_RET_OK as _
}
