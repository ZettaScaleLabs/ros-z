use std::ffi::CString;
use std::sync::Mutex;
use std::sync::atomic::{AtomicI64, Ordering};

use crate::c_void;
use crate::rmw_impl_has_data_ptr;
use crate::ros::*;
use crate::traits::{BorrowData, OwnData, Waitable};
use zenoh::Result;

/// Client implementation for RMW
pub struct ClientImpl {
    pub inner: ros_z::service::ZClient<crate::msg::RosService>,
    pub service_name: CString,
    pub options: rmw_client_options_t,
    pub request_ts: crate::type_support::ServiceTypeSupport,
    pub response_ts: crate::type_support::ServiceTypeSupport,
    pub callback: std::sync::Arc<Mutex<rmw_client_new_response_callback_t>>,
    pub callback_user_data: std::sync::Arc<Mutex<usize>>,
    pub notifier: std::sync::Arc<crate::utils::Notifier>,
    /// Sequence counter that mirrors ZClient's internal counter
    /// Must be kept in sync with inner.sn by calling fetch_add(1) for each request
    pub sequence_counter: AtomicI64,
    /// Tracks responses that arrived while no callback was set
    pub unread_count: std::sync::Arc<Mutex<usize>>,
    pub graph: std::sync::Arc<ros_z::graph::Graph>,
    pub entity: ros_z::entity::EndpointEntity,
}

impl ClientImpl {
    pub fn send_request(&self, request: *const c_void, sequence_id: *mut i64) -> Result<()> {
        // Create RosMessage from the raw pointer using request MessageTypeSupport
        let req = crate::msg::RosMessage::new(request, self.request_ts.request);

        // Get the sequence number before sending (fetch_add returns old value before incrementing)
        // This mirrors ZClient's internal sequence counter behavior
        let sn = self.sequence_counter.fetch_add(1, Ordering::AcqRel);

        // Create notification callback that wakes up wait sets and invokes user callback
        let notifier = self.notifier.clone();
        let callback_holder = self.callback.clone();
        let user_data_holder = self.callback_user_data.clone();
        let unread_count_holder = self.unread_count.clone();
        let notify_callback = move || {
            // Wake up wait sets
            notifier.notify_all();
            // Invoke user callback if set, otherwise increment unread count
            if let Ok(cb) = callback_holder.lock() {
                if let Some(callback_fn) = *cb {
                    if let Ok(user_data_usize) = user_data_holder.lock() {
                        unsafe {
                            let user_data_ptr = *user_data_usize as *const std::ffi::c_void;
                            callback_fn(user_data_ptr, 1); // 1 new response
                        }
                    }
                } else {
                    // No callback set, increment unread count
                    if let Ok(mut unread) = unread_count_holder.lock() {
                        *unread += 1;
                    }
                }
            }
        };

        // Send the request with notification callback
        let _ = self.inner.rmw_send_request(&req, notify_callback)?;

        // Return the sequence number we tracked
        unsafe {
            *sequence_id = sn;
        }

        tracing::debug!(
            "[ClientImpl::send_request] Request sent successfully, returned sn: {}",
            sn
        );
        Ok(())
    }

    pub fn take_response(
        &self,
        request_header: *mut rmw_service_info_t,
        response: *mut c_void,
        taken: *mut bool,
    ) -> Result<()> {
        unsafe {
            *taken = false;
        }

        tracing::debug!(
            "[ClientImpl::take_response] Attempting to take response, rx has {} items",
            if self.inner.rx.is_empty() {
                0
            } else {
                self.inner.rx.len()
            }
        );

        // Try to receive a response
        if let Ok(sample) = self.inner.rx.try_recv() {
            tracing::debug!("[ClientImpl::take_response] Got response sample");

            let payload = sample.payload();
            let bytes = payload.to_bytes().to_vec();

            // Deserialize response using response MessageTypeSupport
            unsafe {
                self.response_ts
                    .response
                    .deserialize_message(&bytes, response as *mut _);
            }

            // Fill request_header
            if !request_header.is_null() {
                // Extract sequence number and GID from attachment if available
                let (sn, gid, source_timestamp) = if let Some(attachment_bytes) =
                    sample.attachment()
                {
                    match ros_z::attachment::Attachment::try_from(attachment_bytes) {
                        Ok(attachment) => {
                            tracing::debug!(
                                "[ClientImpl::take_response] Extracted attachment: sn={}, gid={:?}",
                                attachment.sequence_number,
                                attachment.source_gid
                            );
                            (
                                attachment.sequence_number,
                                attachment.source_gid,
                                attachment.source_timestamp,
                            )
                        }
                        Err(e) => {
                            tracing::warn!(
                                "[ClientImpl::take_response] Failed to extract attachment: {}",
                                e
                            );
                            (0, [0u8; 16], 0)
                        }
                    }
                } else {
                    tracing::warn!("[ClientImpl::take_response] No attachment in response");
                    (0, [0u8; 16], 0)
                };

                // Set received_timestamp to current time
                let received_timestamp = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_or(0, |v| v.as_nanos() as i64);

                unsafe {
                    (*request_header).request_id.sequence_number = sn;
                    (*request_header)
                        .request_id
                        .writer_guid
                        .copy_from_slice(&gid);
                    (*request_header).source_timestamp = source_timestamp;
                    (*request_header).received_timestamp = received_timestamp;
                }
            }

            unsafe {
                *taken = true;
            }
            tracing::debug!("[ClientImpl::take_response] Response taken successfully");
        } else {
            tracing::debug!("[ClientImpl::take_response] No response available in rx channel");
        }
        Ok(())
    }
}

/// Service implementation for RMW
pub struct ServiceImpl {
    pub inner: ros_z::service::ZServer<crate::msg::RosService>,
    pub service_name: CString,
    pub request_ts: crate::type_support::ServiceTypeSupport,
    pub response_ts: crate::type_support::ServiceTypeSupport,
    pub qos: rmw_qos_profile_t,
    pub callback: std::sync::Arc<Mutex<rmw_service_new_request_callback_t>>,
    pub callback_user_data: std::sync::Arc<Mutex<usize>>,
    /// Tracks requests that arrived while no callback was set
    pub unread_count: std::sync::Arc<Mutex<usize>>,
    pub graph: std::sync::Arc<ros_z::graph::Graph>,
    pub entity: ros_z::entity::EndpointEntity,
}

impl ServiceImpl {
    pub fn take_request(
        &mut self,
        request_header: *mut rmw_service_info_t,
        request: *mut c_void,
        taken: *mut bool,
    ) -> Result<()> {
        unsafe {
            *taken = false;
        }

        // Try to receive a request from the raw receiver
        if let Some(query) = self.inner.queue.as_ref().and_then(|q| q.try_recv()) {
            // Get the payload bytes
            let bytes = if let Some(payload) = query.payload() {
                payload.to_bytes().to_vec()
            } else {
                return Ok(());
            };

            // Extract attachment from query to get GID, sequence number, and timestamp
            let key = if let Some(attachment_bytes) = query.attachment() {
                match ros_z::attachment::Attachment::try_from(attachment_bytes) {
                    Ok(attachment) => {
                        let key: ros_z::service::QueryKey = attachment.into();
                        tracing::debug!(
                            "[ServiceImpl::take_request] Got request with sn: {}, gid: {:?}",
                            key.sn,
                            key.gid
                        );
                        key
                    }
                    Err(e) => {
                        tracing::warn!("Failed to extract attachment from query: {}", e);
                        // Fallback to placeholder
                        ros_z::service::QueryKey {
                            gid: [0u8; 16],
                            sn: 0i64,
                        }
                    }
                }
            } else {
                tracing::warn!("No attachment in query, using placeholder QueryKey");
                ros_z::service::QueryKey {
                    gid: [0u8; 16],
                    sn: 0i64,
                }
            };

            // Extract timestamp from attachment
            let source_timestamp = if let Some(attachment_bytes) = query.attachment() {
                match ros_z::attachment::Attachment::try_from(attachment_bytes) {
                    Ok(attachment) => attachment.source_timestamp,
                    Err(_) => 0,
                }
            } else {
                0
            };

            // Set received_timestamp to current time
            let received_timestamp = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |v| v.as_nanos() as i64);

            // Store the query for later response
            tracing::debug!(
                "[ServiceImpl::take_request] Storing query with key sn:{}, inserting into map",
                key.sn
            );
            self.inner.map.insert(key.clone(), query);
            tracing::debug!(
                "[ServiceImpl::take_request] Map now has {} entries",
                self.inner.map.len()
            );

            // Deserialize into the provided request buffer using request MessageTypeSupport
            unsafe {
                self.request_ts
                    .request
                    .deserialize_message(&bytes, request as *mut _);
            }

            // Fill request_header with sequence info and timestamps
            if !request_header.is_null() {
                unsafe {
                    (*request_header).request_id.sequence_number = key.sn;
                    // Copy GID from key
                    for (i, &byte) in key.gid.iter().enumerate() {
                        if i < 16 {
                            (*request_header).request_id.writer_guid[i] = byte;
                        }
                    }
                    (*request_header).source_timestamp = source_timestamp;
                    (*request_header).received_timestamp = received_timestamp;
                }
            }

            unsafe {
                *taken = true;
            }
        }
        Ok(())
    }

    pub fn send_response(
        &mut self,
        request_header: *const rmw_request_id_t,
        response: *const c_void,
    ) -> Result<()> {
        // Extract QueryKey from request_header
        let key = unsafe {
            let mut gid = [0u8; 16];
            gid.copy_from_slice(&(*request_header).writer_guid);
            ros_z::service::QueryKey {
                gid,
                sn: (*request_header).sequence_number,
            }
        };

        tracing::debug!(
            "[ServiceImpl::send_response] Sending response for key sn:{}, gid:{:?}",
            key.sn,
            key.gid
        );
        tracing::debug!(
            "[ServiceImpl::send_response] Map has {} entries before send_response",
            self.inner.map.len()
        );

        // Create RosMessage Response from the raw pointer using response MessageTypeSupport
        let resp = crate::msg::RosMessage::new(response, self.response_ts.response);

        // Send response
        match self.inner.send_response(&resp, &key) {
            Ok(_) => {
                tracing::debug!("[ServiceImpl::send_response] Response sent successfully");
                Ok(())
            }
            Err(e) => {
                tracing::error!(
                    "[ServiceImpl::send_response] Failed to send response: {}",
                    e
                );
                Err(e)
            }
        }
    }
}

impl Waitable for ClientImpl {
    fn is_ready(&self) -> bool {
        // Acquire fence to ensure we see the latest channel state from other threads
        std::sync::atomic::fence(std::sync::atomic::Ordering::Acquire);
        !self.inner.rx.is_empty()
    }
}

impl Waitable for ServiceImpl {
    fn is_ready(&self) -> bool {
        // Acquire fence to ensure we see the latest channel state from other threads
        std::sync::atomic::fence(std::sync::atomic::Ordering::Acquire);
        self.inner.queue.as_ref().is_some_and(|q| !q.is_empty())
    }
}

rmw_impl_has_data_ptr!(rmw_client_t, rmw_client_impl_t, ClientImpl);
rmw_impl_has_data_ptr!(rmw_service_t, rmw_service_impl_t, ServiceImpl);

// RMW Service Functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_request(
    service: *const rmw_service_t,
    request_header: *mut rmw_service_info_t,
    ros_request: *mut c_void,
    taken: *mut bool,
) -> rmw_ret_t {
    if service.is_null() || request_header.is_null() || ros_request.is_null() || taken.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let service_impl = match (service as *mut rmw_service_t).borrow_mut_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match service_impl.take_request(request_header, ros_request, taken) {
        Ok(_) => RMW_RET_OK as _,
        Err(e) => {
            tracing::error!("Failed to take request: {}", e);
            RMW_RET_ERROR as _
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_send_response(
    service: *const rmw_service_t,
    response_header: *mut rmw_request_id_t,
    ros_response: *mut c_void,
) -> rmw_ret_t {
    if service.is_null() || response_header.is_null() || ros_response.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let service_impl = match (service as *mut rmw_service_t).borrow_mut_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match service_impl.send_response(response_header, ros_response) {
        Ok(_) => RMW_RET_OK as _,
        Err(e) => {
            tracing::error!("Failed to send response: {}", e);
            RMW_RET_ERROR as _
        }
    }
}

// RMW Client Functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_send_request(
    client: *const rmw_client_t,
    ros_request: *const c_void,
    sequence_id: *mut i64,
) -> rmw_ret_t {
    if client.is_null() || ros_request.is_null() || sequence_id.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let client_impl = match client.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match client_impl.send_request(ros_request, sequence_id) {
        Ok(_) => RMW_RET_OK as _,
        Err(e) => {
            tracing::error!("Failed to send request: {}", e);
            RMW_RET_ERROR as _
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_response(
    client: *const rmw_client_t,
    request_header: *mut rmw_service_info_t,
    ros_response: *mut c_void,
    taken: *mut bool,
) -> rmw_ret_t {
    if client.is_null() || request_header.is_null() || ros_response.is_null() || taken.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let client_impl = match client.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match client_impl.take_response(request_header, ros_response, taken) {
        Ok(_) => RMW_RET_OK as _,
        Err(e) => {
            tracing::error!("Failed to take response: {}", e);
            RMW_RET_ERROR as _
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_client_request_publisher_get_actual_qos(
    client: *const rmw_client_t,
    qos: *mut rmw_qos_profile_t,
) -> rmw_ret_t {
    if client.is_null() || qos.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let client_impl = match client.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    unsafe {
        *qos = client_impl.options.qos;
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_client_response_subscription_get_actual_qos(
    client: *const rmw_client_t,
    qos: *mut rmw_qos_profile_t,
) -> rmw_ret_t {
    rmw_client_request_publisher_get_actual_qos(client, qos)
}
