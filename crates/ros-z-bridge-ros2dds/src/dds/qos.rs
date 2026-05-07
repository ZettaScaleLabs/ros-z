use cyclors::qos::{
    DDS_1S_DURATION, DDS_100MS_DURATION, DDS_INFINITE_TIME, Durability, DurabilityKind,
    DurabilityService, History, HistoryKind, IgnoreLocal, IgnoreLocalKind, Qos, Reliability,
    ReliabilityKind,
};

// cyclors 0.2.7 does not expose DDS_LENGTH_UNLIMITED publicly; use the raw value.
const DDS_LENGTH_UNLIMITED: i32 = -1;

/// Default QoS for ROS 2 services (request/reply topics).
pub fn service_default_qos() -> Qos {
    Qos {
        history: Some(History {
            kind: HistoryKind::KEEP_LAST,
            depth: 10,
        }),
        reliability: Some(Reliability {
            kind: ReliabilityKind::RELIABLE,
            max_blocking_time: DDS_INFINITE_TIME,
        }),
        ignore_local: Some(IgnoreLocal {
            kind: IgnoreLocalKind::PARTICIPANT,
        }),
        ..Default::default()
    }
}

/// True if the QoS specifies RELIABLE delivery.
pub fn is_reliable(qos: &Qos) -> bool {
    qos.reliability
        .as_ref()
        .is_some_and(|r| r.kind == ReliabilityKind::RELIABLE)
}

/// True if the QoS specifies TRANSIENT_LOCAL durability.
pub fn is_transient_local(qos: &Qos) -> bool {
    qos.durability
        .as_ref()
        .is_some_and(|d| d.kind == DurabilityKind::TRANSIENT_LOCAL)
}

/// Check whether a writer/reader QoS pair is compatible per RTPS rules.
///
/// Returns `Some(reason)` if the pair is incompatible and the mismatch should
/// be logged; returns `None` if the pair is compatible.
pub fn qos_mismatch_reason(writer_qos: &Qos, reader_qos: &Qos) -> Option<String> {
    // BEST_EFFORT writer + RELIABLE reader → incompatible.
    // Per RTPS, TRANSIENT_LOCAL durability implies RELIABLE delivery even when the
    // reliability field is absent from the QoS struct.
    let writer_reliable = writer_qos
        .reliability
        .as_ref()
        .is_some_and(|r| r.kind == ReliabilityKind::RELIABLE)
        || writer_qos
            .durability
            .as_ref()
            .is_some_and(|d| d.kind == DurabilityKind::TRANSIENT_LOCAL);
    let reader_reliable = reader_qos
        .reliability
        .as_ref()
        .is_some_and(|r| r.kind == ReliabilityKind::RELIABLE);
    if !writer_reliable && reader_reliable {
        return Some(
            "BEST_EFFORT writer cannot satisfy RELIABLE reader; samples may be dropped".to_string(),
        );
    }

    // VOLATILE writer + TRANSIENT_LOCAL reader → incompatible
    let writer_transient = writer_qos
        .durability
        .as_ref()
        .is_some_and(|d| d.kind == DurabilityKind::TRANSIENT_LOCAL);
    let reader_transient = reader_qos
        .durability
        .as_ref()
        .is_some_and(|d| d.kind == DurabilityKind::TRANSIENT_LOCAL);
    if !writer_transient && reader_transient {
        return Some(
            "VOLATILE writer cannot satisfy TRANSIENT_LOCAL reader; late-joiner samples lost"
                .to_string(),
        );
    }

    None
}

/// Adapt a discovered writer's QoS to create a matching reader.
pub fn adapt_writer_qos_for_reader(qos: &Qos) -> Qos {
    let mut reader_qos = qos.clone();
    reader_qos.durability_service = None;
    reader_qos.ownership_strength = None;
    reader_qos.transport_priority = None;
    reader_qos.lifespan = None;
    reader_qos.writer_data_lifecycle = None;
    reader_qos.writer_batching = None;
    reader_qos.properties = None;
    reader_qos.entity_name = None;
    reader_qos.ignore_local = Some(IgnoreLocal {
        kind: IgnoreLocalKind::PARTICIPANT,
    });
    if reader_qos.reliability.is_none() {
        reader_qos.reliability = Some(Reliability {
            kind: ReliabilityKind::BEST_EFFORT,
            max_blocking_time: DDS_100MS_DURATION,
        });
    }
    reader_qos
}

/// Adapt a discovered reader's QoS to create a matching writer.
pub fn adapt_reader_qos_for_writer(qos: &Qos) -> Qos {
    let mut writer_qos = qos.clone();
    writer_qos.time_based_filter = None;
    writer_qos.reader_data_lifecycle = None;
    writer_qos.properties = None;
    writer_qos.entity_name = None;
    writer_qos.ignore_local = Some(IgnoreLocal {
        kind: IgnoreLocalKind::PARTICIPANT,
    });

    let is_transient = writer_qos
        .durability
        .as_ref()
        .is_some_and(|d| d.kind == DurabilityKind::TRANSIENT_LOCAL);

    if is_transient {
        let history_kind = writer_qos
            .history
            .as_ref()
            .map_or(HistoryKind::KEEP_LAST, |h| h.kind);
        let history_depth = writer_qos.history.as_ref().map_or(1, |h| h.depth);
        writer_qos.durability_service = Some(DurabilityService {
            service_cleanup_delay: 60 * DDS_1S_DURATION,
            history_kind,
            history_depth,
            max_samples: DDS_LENGTH_UNLIMITED,
            max_instances: DDS_LENGTH_UNLIMITED,
            max_samples_per_instance: DDS_LENGTH_UNLIMITED,
        });
    }

    // Workaround for FastRTPS interop
    writer_qos.reliability = match writer_qos.reliability {
        Some(mut r) => {
            r.max_blocking_time = r.max_blocking_time.saturating_add(1);
            Some(r)
        }
        None => Some(Reliability {
            kind: ReliabilityKind::RELIABLE,
            max_blocking_time: DDS_100MS_DURATION.saturating_add(1),
        }),
    };

    writer_qos
}

#[cfg(test)]
mod tests {
    use cyclors::qos::{
        DurabilityKind, EntityName, History, HistoryKind, OwnershipStrength, Qos, Reliability,
        ReliabilityKind, TransportPriority,
    };

    use super::*;

    fn reliable_qos() -> Qos {
        Qos {
            reliability: Some(Reliability {
                kind: ReliabilityKind::RELIABLE,
                max_blocking_time: DDS_100MS_DURATION,
            }),
            ..Default::default()
        }
    }

    fn best_effort_qos() -> Qos {
        Qos {
            reliability: Some(Reliability {
                kind: ReliabilityKind::BEST_EFFORT,
                max_blocking_time: 0,
            }),
            ..Default::default()
        }
    }

    fn transient_local_qos(depth: i32) -> Qos {
        Qos {
            durability: Some(Durability {
                kind: DurabilityKind::TRANSIENT_LOCAL,
            }),
            history: Some(History {
                kind: HistoryKind::KEEP_LAST,
                depth,
            }),
            ..Default::default()
        }
    }

    // --- is_reliable / is_transient_local helpers ---

    #[test]
    fn test_is_reliable_true() {
        assert!(is_reliable(&reliable_qos()));
    }

    #[test]
    fn test_is_reliable_false_best_effort() {
        assert!(!is_reliable(&best_effort_qos()));
    }

    #[test]
    fn test_is_reliable_false_no_reliability_field() {
        assert!(!is_reliable(&Qos::default()));
    }

    #[test]
    fn test_is_transient_local_true() {
        assert!(is_transient_local(&transient_local_qos(1)));
    }

    #[test]
    fn test_is_transient_local_false_volatile() {
        assert!(!is_transient_local(&Qos::default()));
    }

    // --- adapt_writer_qos_for_reader ---

    #[test]
    fn test_adapt_writer_strips_writer_only_fields() {
        let qos = Qos {
            reliability: Some(Reliability {
                kind: ReliabilityKind::RELIABLE,
                max_blocking_time: DDS_100MS_DURATION,
            }),
            ownership_strength: Some(OwnershipStrength { value: 5 }),
            transport_priority: Some(TransportPriority { value: 1 }),
            entity_name: Some(EntityName {
                name: "writer".into(),
            }),
            ..Default::default()
        };
        let reader_qos = adapt_writer_qos_for_reader(&qos);
        assert!(reader_qos.ownership_strength.is_none());
        assert!(reader_qos.transport_priority.is_none());
        assert!(reader_qos.entity_name.is_none());
    }

    #[test]
    fn test_adapt_writer_sets_ignore_local() {
        let qos = reliable_qos();
        let reader_qos = adapt_writer_qos_for_reader(&qos);
        assert_eq!(
            reader_qos.ignore_local.unwrap().kind,
            IgnoreLocalKind::PARTICIPANT
        );
    }

    #[test]
    fn test_adapt_writer_defaults_reliability_to_best_effort() {
        let qos = Qos::default(); // no reliability field
        let reader_qos = adapt_writer_qos_for_reader(&qos);
        assert_eq!(
            reader_qos.reliability.unwrap().kind,
            ReliabilityKind::BEST_EFFORT
        );
    }

    #[test]
    fn test_adapt_writer_preserves_existing_reliability() {
        let qos = reliable_qos();
        let reader_qos = adapt_writer_qos_for_reader(&qos);
        assert_eq!(
            reader_qos.reliability.unwrap().kind,
            ReliabilityKind::RELIABLE
        );
    }

    // --- adapt_reader_qos_for_writer ---

    #[test]
    fn test_adapt_reader_sets_ignore_local() {
        let reader_qos = adapt_reader_qos_for_writer(&reliable_qos());
        assert_eq!(
            reader_qos.ignore_local.unwrap().kind,
            IgnoreLocalKind::PARTICIPANT
        );
    }

    #[test]
    fn test_adapt_reader_transient_local_adds_durability_service() {
        let qos = transient_local_qos(5);
        let writer_qos = adapt_reader_qos_for_writer(&qos);
        let ds = writer_qos
            .durability_service
            .expect("durability_service set");
        assert_eq!(ds.history_kind, HistoryKind::KEEP_LAST);
        assert_eq!(ds.history_depth, 5);
        assert_eq!(ds.max_samples, -1); // DDS_LENGTH_UNLIMITED
    }

    #[test]
    fn test_adapt_reader_volatile_no_durability_service() {
        let writer_qos = adapt_reader_qos_for_writer(&reliable_qos());
        assert!(writer_qos.durability_service.is_none());
    }

    #[test]
    fn test_adapt_reader_bumps_reliability_max_blocking() {
        let qos = Qos::default(); // no reliability
        let writer_qos = adapt_reader_qos_for_writer(&qos);
        // Should insert RELIABLE with max_blocking_time = DDS_100MS_DURATION + 1
        let r = writer_qos.reliability.expect("reliability set");
        assert_eq!(r.kind, ReliabilityKind::RELIABLE);
        assert_eq!(r.max_blocking_time, DDS_100MS_DURATION + 1);
    }

    // --- qos_mismatch_reason ---

    #[test]
    fn test_qos_best_effort_writer_reliable_reader_is_mismatch() {
        let reason = qos_mismatch_reason(&best_effort_qos(), &reliable_qos());
        assert!(reason.is_some(), "expected mismatch");
        assert!(reason.unwrap().contains("BEST_EFFORT"));
    }

    #[test]
    fn test_qos_reliable_writer_best_effort_reader_ok() {
        assert!(qos_mismatch_reason(&reliable_qos(), &best_effort_qos()).is_none());
    }

    #[test]
    fn test_qos_volatile_writer_transient_reader_is_mismatch() {
        let reason = qos_mismatch_reason(&reliable_qos(), &transient_local_qos(1));
        assert!(reason.is_some(), "expected mismatch");
        assert!(reason.unwrap().contains("VOLATILE"));
    }

    #[test]
    fn test_qos_transient_writer_volatile_reader_ok() {
        assert!(qos_mismatch_reason(&transient_local_qos(1), &reliable_qos()).is_none());
    }

    #[test]
    fn test_qos_both_same_returns_none() {
        assert!(qos_mismatch_reason(&reliable_qos(), &reliable_qos()).is_none());
        assert!(qos_mismatch_reason(&best_effort_qos(), &best_effort_qos()).is_none());
    }
}
