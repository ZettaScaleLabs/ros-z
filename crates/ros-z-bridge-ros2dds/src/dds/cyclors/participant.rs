use anyhow::{Result, anyhow};
use cyclors::{dds_create_domain, dds_create_participant};
use flume::Receiver;

use crate::dds::{
    backend::{BridgeQos, DdsParticipant},
    discovery::DiscoveryEvent,
};

use super::{
    discovery::run_discovery_raw,
    entity::DdsEntity,
    reader::create_reader,
    writer::{CyclorsWriter, create_writer},
};

/// CycloneDDS-backed participant.
pub struct CyclorsParticipant {
    entity: DdsEntity,
}

// Safety: CycloneDDS entity handles are thread-safe for read/write operations.
unsafe impl Send for CyclorsParticipant {}
unsafe impl Sync for CyclorsParticipant {}

impl DdsParticipant for CyclorsParticipant {
    type Reader = DdsEntity;
    type Writer = CyclorsWriter;

    fn create(domain_id: u32) -> Result<Self> {
        unsafe {
            dds_create_domain(domain_id, std::ptr::null());
            let participant = dds_create_participant(domain_id, std::ptr::null(), std::ptr::null());
            if participant < 0 {
                return Err(anyhow!("dds_create_participant failed: {participant}"));
            }
            Ok(Self {
                entity: DdsEntity::new(participant),
            })
        }
    }

    fn run_discovery(&self) -> Receiver<DiscoveryEvent> {
        let (tx, rx) = flume::bounded(256);
        run_discovery_raw(self.entity.raw(), tx);
        rx
    }

    fn create_reader(
        &self,
        topic: &str,
        type_name: &str,
        keyless: bool,
        qos: BridgeQos,
        callback: Box<dyn Fn(Vec<u8>) + Send + 'static>,
    ) -> Result<DdsEntity> {
        create_reader(self.entity.raw(), topic, type_name, keyless, qos, callback)
    }

    fn create_writer(
        &self,
        topic: &str,
        type_name: &str,
        keyless: bool,
        qos: BridgeQos,
    ) -> Result<CyclorsWriter> {
        create_writer(self.entity.raw(), topic, type_name, keyless, qos)
    }
}
