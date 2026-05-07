use std::{
    sync::{Arc, Mutex as StdMutex},
    time::{Duration, Instant},
};

use anyhow::{Result, anyhow};
use parking_lot::Mutex;
use zenoh::{
    Session, Wait,
    bytes::ZBytes,
    key_expr::KeyExpr,
    pubsub::{Publisher, Subscriber},
    qos::{CongestionControl, Priority},
    sample::Locality,
};
use zenoh_ext::{
    AdvancedPublisher, AdvancedPublisherBuilderExt, AdvancedSubscriber,
    AdvancedSubscriberBuilderExt, CacheConfig, HistoryConfig,
};

use crate::dds::{
    backend::{BridgeQos, DdsParticipant, DdsWriter, DurabilityKind, HistoryKind, ReliabilityKind},
    discovery::DiscoveredEndpoint,
    names::{dds_topic_to_ros2_name, dds_type_to_ros2_type, ros2_name_to_zenoh_key},
    qos::{adapt_reader_qos_for_writer, adapt_writer_qos_for_reader, qos_mismatch_reason},
};

const TRANSIENT_LOCAL_CACHE_MULTIPLIER: usize = 10;

/// Parse a Zenoh priority from its integer discriminant (1=RealTime … 7=Background).
/// Falls back to `Priority::Data` (5) for unrecognised values.
pub fn priority_from_u8(v: u8) -> Priority {
    match v {
        1 => Priority::RealTime,
        2 => Priority::InteractiveHigh,
        3 => Priority::InteractiveLow,
        4 => Priority::DataHigh,
        5 => Priority::Data,
        6 => Priority::DataLow,
        7 => Priority::Background,
        _ => {
            tracing::warn!("Unknown publication priority {v}; defaulting to Priority::Data");
            Priority::Data
        }
    }
}

// ─── inner publisher ──────────────────────────────────────────────────────────

enum BridgePublisherInner {
    Plain(Publisher<'static>),
    Cached(Arc<AdvancedPublisher<'static>>),
}

impl BridgePublisherInner {
    fn put_wait(&self, bytes: ZBytes, attachment: Option<Vec<u8>>) -> Result<()> {
        match self {
            Self::Plain(p) => {
                if let Some(a) = attachment {
                    p.put(bytes)
                        .attachment(a)
                        .wait()
                        .map_err(|e| anyhow!("Zenoh put failed: {e}"))
                } else {
                    p.put(bytes)
                        .wait()
                        .map_err(|e| anyhow!("Zenoh put failed: {e}"))
                }
            }
            Self::Cached(p) => {
                if let Some(a) = attachment {
                    p.put(bytes)
                        .attachment(a)
                        .wait()
                        .map_err(|e| anyhow!("Zenoh put (cached) failed: {e}"))
                } else {
                    p.put(bytes)
                        .wait()
                        .map_err(|e| anyhow!("Zenoh put (cached) failed: {e}"))
                }
            }
        }
    }
}

// ─── TopicPublisherSlot ───────────────────────────────────────────────────────

/// Shared Zenoh-side publisher for a DDS topic.
///
/// Wrapped in `Arc` and stored in `Bridge::topic_publishers` keyed by
/// `(domain_id, topic_name)`. Surviving DDS writer churn preserves the
/// AdvancedPublisher history cache for TRANSIENT_LOCAL topics (#690).
pub(crate) struct TopicPublisherSlot {
    inner: BridgePublisherInner,
}

// Safety: Publisher<'static> and AdvancedPublisher<'static> are Send+Sync.
unsafe impl Send for TopicPublisherSlot {}
unsafe impl Sync for TopicPublisherSlot {}

impl TopicPublisherSlot {
    pub(crate) fn put_wait(&self, bytes: ZBytes, attachment: Option<Vec<u8>>) -> Result<()> {
        self.inner.put_wait(bytes, attachment)
    }

    async fn create(
        endpoint: &DiscoveredEndpoint,
        session: &Session,
        namespace: Option<&str>,
        reliable_routes_blocking: bool,
        priority: Priority,
        express: bool,
    ) -> Result<Self> {
        let ros2_name = dds_topic_to_ros2_name(&endpoint.topic_name)
            .ok_or_else(|| anyhow!("not a bridgeable topic: {}", endpoint.topic_name))?;

        let zenoh_key = ros2_name_to_zenoh_key(&ros2_name, namespace);
        let ke: KeyExpr<'static> = zenoh_key
            .try_into()
            .map_err(|e| anyhow!("invalid key expr: {e}"))?;

        let is_reliable = endpoint
            .qos
            .reliability
            .as_ref()
            .is_some_and(|r| r.kind == ReliabilityKind::Reliable);

        let is_transient_local = endpoint
            .qos
            .durability
            .as_ref()
            .is_some_and(|d| d.kind == DurabilityKind::TransientLocal);

        let congestion_ctrl = if reliable_routes_blocking && is_reliable {
            CongestionControl::Block
        } else {
            CongestionControl::Drop
        };

        let inner = if is_transient_local {
            let cache_size = compute_cache_size(&endpoint.qos, endpoint.keyless);
            tracing::debug!(
                "DDS→Zenoh: TRANSIENT_LOCAL topic {}, cache_size={}",
                endpoint.topic_name,
                cache_size
            );
            let adv = session
                .declare_publisher(ke.clone())
                .congestion_control(congestion_ctrl)
                .priority(priority)
                .express(express)
                .allowed_destination(Locality::Remote)
                .cache(CacheConfig::default().max_samples(cache_size))
                .publisher_detection()
                .await
                .map_err(|e| anyhow!("declare_publisher(advanced) failed: {e}"))?;
            BridgePublisherInner::Cached(Arc::new(adv))
        } else {
            let plain = session
                .declare_publisher(ke.clone())
                .congestion_control(congestion_ctrl)
                .priority(priority)
                .express(express)
                .allowed_destination(Locality::Remote)
                .await
                .map_err(|e| anyhow!("declare_publisher failed: {e}"))?;
            BridgePublisherInner::Plain(plain)
        };

        Ok(Self { inner })
    }
}

// ─── DdsToZenohRoute ─────────────────────────────────────────────────────────

/// A route from a DDS publication to a Zenoh publisher.
pub struct DdsToZenohRoute<P: DdsParticipant> {
    _reader: P::Reader,
    _publisher: Arc<TopicPublisherSlot>,
    topic_name: String,
}

impl<P: DdsParticipant> DdsToZenohRoute<P> {
    pub(crate) fn topic_name(&self) -> &str {
        &self.topic_name
    }

    pub async fn create(
        participant: &P,
        endpoint: &DiscoveredEndpoint,
        session: &Session,
        namespace: Option<&str>,
        reliable_routes_blocking: bool,
        priority: Priority,
        express: bool,
        max_publication_hz: f64,
        topic_publishers: &Arc<
            Mutex<std::collections::HashMap<(u32, String), Arc<TopicPublisherSlot>>>,
        >,
        domain_id: u32,
    ) -> Result<Self> {
        let topic_name = endpoint.topic_name.clone();
        let slot_key = (domain_id, topic_name.clone());

        let arc_pub = {
            let mut map = topic_publishers.lock();
            if let Some(existing) = map.get(&slot_key).cloned() {
                tracing::debug!(
                    "Reusing existing publisher slot for {} (arc_count={})",
                    topic_name,
                    Arc::strong_count(&existing)
                );
                existing
            } else {
                let new_pub = Arc::new(
                    TopicPublisherSlot::create(
                        endpoint,
                        session,
                        namespace,
                        reliable_routes_blocking,
                        priority,
                        express,
                    )
                    .await?,
                );
                map.insert(slot_key, Arc::clone(&new_pub));
                new_pub
            }
        };

        let qos = adapt_writer_qos_for_reader(&endpoint.qos);

        if let Some(reason) = qos_mismatch_reason(&endpoint.qos, &qos) {
            tracing::warn!("QoS mismatch on {}: {}", endpoint.topic_name, reason);
        }

        let ros2_type = dds_type_to_ros2_type(&endpoint.type_name);
        let ke_display = {
            if let Some(ros2_name) = dds_topic_to_ros2_name(&endpoint.topic_name) {
                ros2_name_to_zenoh_key(&ros2_name, namespace)
            } else {
                endpoint.topic_name.clone()
            }
        };
        let type_name = endpoint.type_name.clone();
        let keyless = endpoint.keyless;

        tracing::info!("DDS→Zenoh pub/sub route: {topic_name} → {ke_display}");

        let pub_clone = Arc::clone(&arc_pub);

        let min_interval: Option<Duration> = if max_publication_hz > 0.0 {
            Some(Duration::from_secs_f64(1.0 / max_publication_hz))
        } else {
            None
        };
        let last_pub: Arc<StdMutex<Option<Instant>>> = Arc::new(StdMutex::new(None));

        let reader = participant.create_reader(
            &topic_name,
            &type_name,
            keyless,
            qos,
            Box::new(move |bytes: Vec<u8>| {
                if let Some(interval) = min_interval {
                    let mut last = last_pub.lock().unwrap();
                    let now = Instant::now();
                    if let Some(t) = *last {
                        if now.duration_since(t) < interval {
                            return;
                        }
                    }
                    *last = Some(now);
                }
                let zbytes: ZBytes = ZBytes::from(bytes.as_slice());
                let attachment = Some(ros2_type.as_bytes().to_vec());
                if let Err(e) = pub_clone.put_wait(zbytes, attachment) {
                    tracing::warn!("Zenoh put failed on {ke_display}: {e}");
                }
            }),
        )?;

        Ok(Self {
            _reader: reader,
            _publisher: arc_pub,
            topic_name,
        })
    }
}

// ─── compute_cache_size ───────────────────────────────────────────────────────

/// Compute the AdvancedPublisher cache size from DDS QoS.
///
/// Mirrors the zenoh-plugin-ros2dds formula:
/// - KEEP_ALL → usize::MAX
/// - keyless topics → `depth × multiplier` (no instance axis)
/// - KEEP_LAST, unlimited instances → usize::MAX
/// - KEEP_LAST, N instances → `(depth × N) × multiplier`
fn compute_cache_size(qos: &BridgeQos, keyless: bool) -> usize {
    let depth = match &qos.history {
        Some(h) => match h.kind {
            HistoryKind::KeepAll => return usize::MAX,
            HistoryKind::KeepLast => h.depth as usize,
        },
        None => 1,
    };

    if keyless {
        return depth.saturating_mul(TRANSIENT_LOCAL_CACHE_MULTIPLIER);
    }

    let max_instances: Option<usize> = qos
        .durability_service
        .as_ref()
        .and_then(|ds| ds.max_instances);

    match max_instances {
        None => usize::MAX,
        Some(n) => depth
            .saturating_mul(n)
            .saturating_mul(TRANSIENT_LOCAL_CACHE_MULTIPLIER),
    }
}

// ─── ZenohToDdsRoute ─────────────────────────────────────────────────────────

#[allow(dead_code)]
enum ZenohSubscriberHandle {
    Plain(Subscriber<()>),
    Advanced(AdvancedSubscriber<()>),
}

unsafe impl Send for ZenohSubscriberHandle {}
unsafe impl Sync for ZenohSubscriberHandle {}

/// A route from a Zenoh subscriber to a DDS writer.
pub struct ZenohToDdsRoute<P: DdsParticipant> {
    _writer: Arc<P::Writer>,
    _subscriber: ZenohSubscriberHandle,
}

impl<P: DdsParticipant> ZenohToDdsRoute<P> {
    pub async fn create(
        participant: &P,
        endpoint: &DiscoveredEndpoint,
        session: &Session,
        namespace: Option<&str>,
    ) -> Result<Self> {
        let ros2_name = dds_topic_to_ros2_name(&endpoint.topic_name)
            .ok_or_else(|| anyhow!("not a bridgeable topic: {}", endpoint.topic_name))?;

        let zenoh_key = ros2_name_to_zenoh_key(&ros2_name, namespace);
        let ke: KeyExpr<'static> = zenoh_key
            .try_into()
            .map_err(|e| anyhow!("invalid key expr: {e}"))?;

        let qos = adapt_reader_qos_for_writer(&endpoint.qos);

        if let Some(reason) = qos_mismatch_reason(&qos, &endpoint.qos) {
            tracing::warn!("QoS mismatch on {}: {}", endpoint.topic_name, reason);
        }

        let writer = participant.create_writer(
            &endpoint.topic_name,
            &endpoint.type_name,
            endpoint.keyless,
            qos,
        )?;

        let ke_display = ke.to_string();
        let dds_topic = endpoint.topic_name.clone();
        let endpoint_is_transient_local = endpoint
            .qos
            .durability
            .as_ref()
            .is_some_and(|d| d.kind == DurabilityKind::TransientLocal);

        tracing::info!("Zenoh→DDS pub/sub route: {ke_display} → {dds_topic}");

        // Wrap in Arc so the writer can be shared with the subscriber callback.
        let writer = Arc::new(writer);
        let writer_cb = Arc::clone(&writer);

        let subscriber_handle = if endpoint_is_transient_local {
            tracing::debug!(
                "Zenoh→DDS: TRANSIENT_LOCAL reader on {dds_topic}, using AdvancedSubscriber"
            );
            let adv = session
                .declare_subscriber(ke.clone())
                .history(HistoryConfig::default().detect_late_publishers())
                .callback(move |sample| {
                    let bytes: Vec<u8> = sample.payload().to_bytes().into_owned();
                    if let Err(e) = writer_cb.write_cdr(bytes) {
                        tracing::warn!("DDS write failed on {ke_display}: {e}");
                    }
                })
                .await
                .map_err(|e| anyhow!("declare_subscriber(advanced) failed: {e}"))?;
            ZenohSubscriberHandle::Advanced(adv)
        } else {
            let sub = session
                .declare_subscriber(ke.clone())
                .callback(move |sample| {
                    let bytes: Vec<u8> = sample.payload().to_bytes().into_owned();
                    if let Err(e) = writer_cb.write_cdr(bytes) {
                        tracing::warn!("DDS write failed on {ke_display}: {e}");
                    }
                })
                .await
                .map_err(|e| anyhow!("declare_subscriber failed: {e}"))?;
            ZenohSubscriberHandle::Plain(sub)
        };

        Ok(Self {
            _writer: writer,
            _subscriber: subscriber_handle,
        })
    }
}

// ─── tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use crate::dds::backend::{
        BridgeQos, Durability, DurabilityKind, DurabilityService, History, HistoryKind,
    };
    use crate::dds::names::dds_type_to_ros2_type;

    use super::compute_cache_size;

    fn qos_transient_local(depth: i32) -> BridgeQos {
        BridgeQos {
            durability: Some(Durability {
                kind: DurabilityKind::TransientLocal,
            }),
            history: Some(History {
                kind: HistoryKind::KeepLast,
                depth,
            }),
            ..Default::default()
        }
    }

    fn qos_transient_local_with_instances(depth: i32, max_instances: Option<usize>) -> BridgeQos {
        BridgeQos {
            durability: Some(Durability {
                kind: DurabilityKind::TransientLocal,
            }),
            history: Some(History {
                kind: HistoryKind::KeepLast,
                depth,
            }),
            durability_service: Some(DurabilityService {
                service_cleanup_delay: None,
                history_kind: HistoryKind::KeepLast,
                history_depth: depth,
                max_samples: None,
                max_instances,
                max_samples_per_instance: None,
            }),
            ..Default::default()
        }
    }

    fn qos_volatile(depth: i32) -> BridgeQos {
        BridgeQos {
            history: Some(History {
                kind: HistoryKind::KeepLast,
                depth,
            }),
            ..Default::default()
        }
    }

    fn qos_keep_all() -> BridgeQos {
        BridgeQos {
            durability: Some(Durability {
                kind: DurabilityKind::TransientLocal,
            }),
            history: Some(History {
                kind: HistoryKind::KeepAll,
                depth: 0,
            }),
            ..Default::default()
        }
    }

    #[test]
    fn test_cache_size_keep_last_keyless() {
        assert_eq!(compute_cache_size(&qos_transient_local(1), true), 10);
        assert_eq!(compute_cache_size(&qos_transient_local(5), true), 50);
    }

    #[test]
    fn test_cache_size_keep_last_no_durability_service_is_unlimited() {
        assert_eq!(
            compute_cache_size(&qos_transient_local(5), false),
            usize::MAX
        );
    }

    #[test]
    fn test_cache_size_keep_last_unlimited_instances_is_max() {
        let qos = qos_transient_local_with_instances(5, None);
        assert_eq!(compute_cache_size(&qos, false), usize::MAX);
    }

    #[test]
    fn test_cache_size_keep_last_n_instances() {
        let qos = qos_transient_local_with_instances(2, Some(3));
        assert_eq!(compute_cache_size(&qos, false), 60);
    }

    #[test]
    fn test_cache_size_no_history_defaults_to_multiplier() {
        assert_eq!(compute_cache_size(&BridgeQos::default(), true), 10);
    }

    #[test]
    fn test_cache_size_keep_all_is_max() {
        assert_eq!(compute_cache_size(&qos_keep_all(), false), usize::MAX);
        assert_eq!(compute_cache_size(&qos_keep_all(), true), usize::MAX);
    }

    #[test]
    fn test_volatile_cache_size_keyless() {
        assert_eq!(compute_cache_size(&qos_volatile(3), true), 30);
    }

    #[test]
    fn test_type_name_attachment_string_type() {
        let dds_type = "std_msgs::msg::dds_::String_";
        let ros2_type = dds_type_to_ros2_type(dds_type);
        assert_eq!(ros2_type.as_bytes(), b"std_msgs/msg/String");
    }

    #[test]
    fn test_type_name_attachment_full_dds_type_preserved() {
        let dds_type = "example_interfaces::msg::dds_::Int64_";
        let ros2_type = dds_type_to_ros2_type(dds_type);
        assert_eq!(ros2_type, "example_interfaces/msg/Int64");
    }

    #[test]
    fn test_arc_count_increases_with_clones() {
        use std::sync::Arc;
        let val = Arc::new(42u32);
        let _clone1 = Arc::clone(&val);
        let _clone2 = Arc::clone(&val);
        assert_eq!(Arc::strong_count(&val), 3);
    }

    #[test]
    fn test_arc_count_decreases_on_drop() {
        use std::sync::Arc;
        let val = Arc::new(42u32);
        let clone = Arc::clone(&val);
        assert_eq!(Arc::strong_count(&val), 2);
        drop(clone);
        assert_eq!(Arc::strong_count(&val), 1);
    }
}
