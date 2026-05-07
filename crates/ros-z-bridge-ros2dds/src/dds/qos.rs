use crate::dds::backend::{BridgeQos, DurabilityKind, ReliabilityKind};

pub use crate::dds::cyclors::qos::{
    adapt_reader_qos_for_writer, adapt_writer_qos_for_reader, qos_mismatch_reason,
    service_default_bridge_qos,
};

pub fn is_reliable(qos: &BridgeQos) -> bool {
    qos.reliability
        .as_ref()
        .is_some_and(|r| r.kind == ReliabilityKind::Reliable)
}

pub fn is_transient_local(qos: &BridgeQos) -> bool {
    qos.durability
        .as_ref()
        .is_some_and(|d| d.kind == DurabilityKind::TransientLocal)
}
