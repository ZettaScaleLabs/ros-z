use std::{ffi::CString, mem::MaybeUninit};

use anyhow::{Result, anyhow};
use cyclors::{
    DDS_ANY_STATE, cdds_create_blob_topic, dds_create_listener, dds_create_reader, dds_entity_t,
    dds_lset_data_available, dds_reader_wait_for_historical_data, dds_sample_info_t,
    dds_strretcode, dds_takecdr, ddsi_serdata, ddsi_serdata_unref, qos::Qos,
};

use super::types::DDSRawSample;

unsafe extern "C" fn on_data_available<F>(dr: dds_entity_t, arg: *mut std::os::raw::c_void)
where
    F: Fn(DDSRawSample),
{
    let cb = unsafe { &*(arg as *const F) };
    let mut zp: *mut ddsi_serdata = std::ptr::null_mut();
    #[allow(clippy::uninit_assumed_init)]
    let mut si = MaybeUninit::<[dds_sample_info_t; 1]>::uninit();
    unsafe {
        while dds_takecdr(
            dr,
            &mut zp,
            1,
            si.as_mut_ptr() as *mut dds_sample_info_t,
            DDS_ANY_STATE,
        ) > 0
        {
            let si = si.assume_init();
            if si[0].valid_data {
                let raw = DDSRawSample::create(zp);
                cb(raw);
            }
            ddsi_serdata_unref(zp);
        }
    }
}

/// Create a DDS reader for a blob (schema-free) topic with a listener callback.
///
/// The callback receives each valid CDR sample as a [`DDSRawSample`].
/// The listener fires immediately on data arrival (no polling).
pub fn create_blob_reader<F>(
    dp: dds_entity_t,
    topic_name: &str,
    type_name: &str,
    keyless: bool,
    qos: Qos,
    callback: F,
) -> Result<dds_entity_t>
where
    F: Fn(DDSRawSample) + Send + 'static,
{
    unsafe {
        let c_topic = CString::new(topic_name).unwrap();
        let c_type = CString::new(type_name).unwrap();
        let topic = cdds_create_blob_topic(dp, c_topic.into_raw(), c_type.into_raw(), keyless);
        if topic < 0 {
            return Err(anyhow!("cdds_create_blob_topic failed: {topic}"));
        }

        let cb_box = Box::new(callback);
        let listener = dds_create_listener(Box::into_raw(cb_box) as *mut std::os::raw::c_void);
        dds_lset_data_available(listener, Some(on_data_available::<F>));

        let qos_native = qos.to_qos_native();
        let reader = dds_create_reader(dp, topic, qos_native, listener);
        Qos::delete_qos_native(qos_native);

        if reader < 0 {
            let msg = std::ffi::CStr::from_ptr(dds_strretcode(-reader))
                .to_str()
                .unwrap_or("unknown DDS error");
            return Err(anyhow!("dds_create_reader failed: {msg}"));
        }

        // Wait for historical data (transient-local durability)
        let res = dds_reader_wait_for_historical_data(reader, cyclors::qos::DDS_100MS_DURATION);
        if res < 0 {
            tracing::warn!("dds_reader_wait_for_historical_data: {res}");
        }

        Ok(reader)
    }
}
