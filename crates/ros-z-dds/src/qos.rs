use ros_z_protocol::qos::{QosDurability, QosHistory, QosProfile, QosReliability};

use crate::participant::{BridgeQos, DurabilityKind, HistoryKind, ReliabilityKind};

pub use crate::cyclors::qos::{
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

/// Convert `BridgeQos` (DDS QoS) to `ros-z-protocol` `QosProfile` for key construction.
pub fn bridge_qos_to_qos_profile(qos: &BridgeQos) -> QosProfile {
    let reliability = qos
        .reliability
        .as_ref()
        .map(|r| match r.kind {
            ReliabilityKind::Reliable => QosReliability::Reliable,
            ReliabilityKind::BestEffort => QosReliability::BestEffort,
        })
        .unwrap_or_default();

    let durability = qos
        .durability
        .as_ref()
        .map(|d| match d.kind {
            DurabilityKind::TransientLocal => QosDurability::TransientLocal,
            _ => QosDurability::Volatile,
        })
        .unwrap_or_default();

    let history = qos
        .history
        .as_ref()
        .map(|h| match h.kind {
            HistoryKind::KeepAll => QosHistory::KeepAll,
            HistoryKind::KeepLast => QosHistory::KeepLast(h.depth as usize),
        })
        .unwrap_or_default();

    QosProfile {
        reliability,
        durability,
        history,
    }
}
