use std::ffi::CString;

use anyhow::{Result, anyhow};
use cyclors::{dds_create_domain, dds_create_participant, dds_entity_t};

use super::entity::DdsEntity;

/// Create a DDS participant for the given domain ID.
pub fn create_participant(domain_id: u32) -> Result<DdsEntity> {
    unsafe {
        // Ensure domain exists (idempotent if already created)
        dds_create_domain(domain_id, std::ptr::null());

        let qos = std::ptr::null();
        let listener = std::ptr::null();
        let participant: dds_entity_t = dds_create_participant(domain_id, qos, listener);
        if participant < 0 {
            return Err(anyhow!("dds_create_participant failed: {participant}"));
        }
        Ok(DdsEntity::new(participant))
    }
}

/// Get the instance handle of a DDS entity (stable across restarts within a session).
pub fn get_instance_handle(entity: dds_entity_t) -> Result<u64> {
    unsafe {
        let mut handle: cyclors::dds_instance_handle_t = 0;
        let ret = cyclors::dds_get_instance_handle(entity, &mut handle);
        if ret == 0 {
            Ok(handle)
        } else {
            Err(anyhow!("dds_get_instance_handle failed: {ret}"))
        }
    }
}
