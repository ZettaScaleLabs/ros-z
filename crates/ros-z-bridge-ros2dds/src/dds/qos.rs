use cyclors::qos::{
    DDS_1S_DURATION, DDS_100MS_DURATION, DDS_INFINITE_TIME, DurabilityKind, DurabilityService,
    History, HistoryKind, IgnoreLocal, IgnoreLocalKind, Qos, Reliability, ReliabilityKind,
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
