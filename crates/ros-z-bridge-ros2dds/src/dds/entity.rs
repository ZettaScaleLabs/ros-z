use cyclors::{dds_delete, dds_entity_t};

/// RAII wrapper around a `dds_entity_t`.
///
/// Calls `dds_delete` on drop, preventing ghost-subscriber accumulation (fix for #570).
pub struct DdsEntity(dds_entity_t);

impl DdsEntity {
    /// Wrap an existing DDS entity handle.
    ///
    /// # Safety
    /// `handle` must be a valid entity created by the caller and not yet deleted.
    pub unsafe fn new(handle: dds_entity_t) -> Self {
        Self(handle)
    }

    pub fn raw(&self) -> dds_entity_t {
        self.0
    }
}

impl Drop for DdsEntity {
    fn drop(&mut self) {
        if self.0 != 0 {
            unsafe {
                dds_delete(self.0);
            }
        }
    }
}

unsafe impl Send for DdsEntity {}
unsafe impl Sync for DdsEntity {}
