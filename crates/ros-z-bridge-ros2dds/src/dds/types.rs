use std::slice;

use cyclors::{
    ddsi_serdata, ddsi_serdata_size, ddsi_serdata_to_ser_ref, ddsi_serdata_to_ser_unref,
    ddsrt_iov_len_t, ddsrt_iovec_t,
};

pub fn ddsrt_iov_len_to_usize(len: ddsrt_iov_len_t) -> usize {
    len as usize
}

/// Zero-copy view of a raw DDS sample's CDR bytes.
///
/// The first 4 bytes are the CDR representation header; `payload_as_slice()` skips them.
pub struct DDSRawSample {
    sdref: *mut ddsi_serdata,
    data: ddsrt_iovec_t,
}

impl DDSRawSample {
    /// Create a raw sample from a `ddsi_serdata` pointer obtained by `dds_takecdr`.
    ///
    /// # Safety
    /// `serdata` must be a valid pointer returned by CycloneDDS.
    pub unsafe fn create(serdata: *const ddsi_serdata) -> Self {
        let mut data = ddsrt_iovec_t {
            iov_base: std::ptr::null_mut(),
            iov_len: 0,
        };
        let sdref = unsafe {
            let size = ddsi_serdata_size(serdata);
            ddsi_serdata_to_ser_ref(serdata, 0, size as usize, &mut data)
        };
        DDSRawSample { sdref, data }
    }

    /// Full CDR bytes including the 4-byte representation header.
    pub fn as_slice(&self) -> &[u8] {
        unsafe {
            slice::from_raw_parts(
                self.data.iov_base as *const u8,
                ddsrt_iov_len_to_usize(self.data.iov_len),
            )
        }
    }
}

impl Drop for DDSRawSample {
    fn drop(&mut self) {
        unsafe {
            ddsi_serdata_to_ser_unref(self.sdref, &self.data);
        }
    }
}

unsafe impl Send for DDSRawSample {}
