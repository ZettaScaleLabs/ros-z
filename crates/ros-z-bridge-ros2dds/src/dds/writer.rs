use std::ffi::{CStr, CString};

use anyhow::{Result, anyhow};
use cyclors::{
    cdds_create_blob_topic, dds_create_writer, dds_entity_t, dds_get_entity_sertype,
    dds_strretcode, dds_writecdr, ddsi_serdata_from_ser_iov, ddsi_serdata_kind_SDK_DATA,
    ddsi_sertype, ddsrt_iov_len_t, ddsrt_iovec_t, qos::Qos,
};

use super::types::ddsrt_iov_len_to_usize;

/// Create a DDS writer for a blob (schema-free) topic.
pub fn create_blob_writer(
    dp: dds_entity_t,
    topic_name: &str,
    type_name: &str,
    keyless: bool,
    qos: Qos,
) -> Result<dds_entity_t> {
    unsafe {
        let c_topic = CString::new(topic_name).unwrap();
        let c_type = CString::new(type_name).unwrap();
        let topic = cdds_create_blob_topic(dp, c_topic.into_raw(), c_type.into_raw(), keyless);
        if topic < 0 {
            return Err(anyhow!("cdds_create_blob_topic failed: {topic}"));
        }

        let qos_native = qos.to_qos_native();
        let writer = dds_create_writer(dp, topic, qos_native, std::ptr::null_mut());
        Qos::delete_qos_native(qos_native);

        if writer < 0 {
            let msg = CStr::from_ptr(dds_strretcode(-writer))
                .to_str()
                .unwrap_or("unknown");
            return Err(anyhow!("dds_create_writer failed: {msg}"));
        }
        Ok(writer)
    }
}

/// Write raw CDR bytes through a DDS writer.
///
/// `data` must include the 4-byte CDR representation header followed by the payload.
pub fn write_cdr(writer: dds_entity_t, data: Vec<u8>) -> Result<()> {
    unsafe {
        let len = data.len();
        // Safety: we reconstruct the Vec from raw parts after the DDS write to ensure proper drop.
        let mut data = std::mem::ManuallyDrop::new(data);
        let ptr = data.as_mut_ptr();
        let cap = data.capacity();

        let iov_len: ddsrt_iov_len_t = len
            .try_into()
            .map_err(|_| anyhow!("CDR payload too large"))?;

        let iov = ddsrt_iovec_t {
            iov_base: ptr as *mut std::os::raw::c_void,
            iov_len,
        };

        let mut sertype: *const ddsi_sertype = std::ptr::null();
        let ret = dds_get_entity_sertype(writer, &mut sertype);
        if ret < 0 {
            drop(Vec::from_raw_parts(ptr, len, cap));
            let msg = CStr::from_ptr(dds_strretcode(ret))
                .to_str()
                .unwrap_or("unknown");
            return Err(anyhow!("dds_get_entity_sertype failed: {msg}"));
        }

        let serdata = ddsi_serdata_from_ser_iov(sertype, ddsi_serdata_kind_SDK_DATA, 1, &iov, len);

        let ret = dds_writecdr(writer, serdata);
        drop(Vec::from_raw_parts(ptr, len, cap));

        if ret < 0 {
            let msg = CStr::from_ptr(dds_strretcode(ret))
                .to_str()
                .unwrap_or("unknown");
            return Err(anyhow!("dds_writecdr failed: {msg}"));
        }
        Ok(())
    }
}
