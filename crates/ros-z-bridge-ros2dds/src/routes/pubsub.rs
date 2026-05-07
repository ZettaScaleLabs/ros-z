use std::sync::Arc;

use anyhow::{Result, anyhow};
use cyclors::{
    dds_entity_t,
    qos::{DurabilityKind, HistoryKind, ReliabilityKind},
};
use parking_lot::Mutex;
use zenoh::{
    Session, Wait,
    bytes::ZBytes,
    key_expr::KeyExpr,
    pubsub::{Publisher, Subscriber},
    qos::CongestionControl,
    sample::Locality,
};
use zenoh_ext::{
    AdvancedPublisher, AdvancedPublisherBuilderExt, AdvancedSubscriber,
    AdvancedSubscriberBuilderExt, CacheConfig, HistoryConfig,
};

use crate::dds::{
    discovery::DiscoveredEndpoint,
    entity::DdsEntity,
    names::{dds_topic_to_ros2_name, dds_type_to_ros2_type, ros2_name_to_zenoh_key},
    qos::{
        adapt_reader_qos_for_writer, adapt_writer_qos_for_reader, is_transient_local,
        qos_mismatch_reason,
    },
    reader::create_blob_reader,
    types::DDSRawSample,
    writer::{create_blob_writer, write_cdr},
};

const TRANSIENT_LOCAL_CACHE_MULTIPLIER: usize = 10;

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
            .is_some_and(|r| r.kind == ReliabilityKind::RELIABLE);

        let is_transient_local = endpoint
            .qos
            .durability
            .as_ref()
            .is_some_and(|d| d.kind == DurabilityKind::TRANSIENT_LOCAL);

        let congestion_ctrl = if reliable_routes_blocking && is_reliable {
            CongestionControl::Block
        } else {
            CongestionControl::Drop
        };

        let inner = if is_transient_local {
            let cache_size = compute_cache_size(&endpoint.qos);
            tracing::debug!(
                "DDS→Zenoh: TRANSIENT_LOCAL topic {}, cache_size={}",
                endpoint.topic_name,
                cache_size
            );
            let adv = session
                .declare_publisher(ke.clone())
                .congestion_control(congestion_ctrl)
                // Prevent routing loops when two bridge instances share a Zenoh session (#542).
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
///
/// The Zenoh publisher is held in a shared `TopicPublisherSlot` (also stored in
/// `Bridge::topic_publishers`). When multiple DDS writers for the same topic are
/// discovered, they all share one slot. Undiscovering a writer does not destroy
/// the slot immediately — a 5-second grace period in Bridge allows the slot
/// (and its AdvancedPublisher history cache) to survive rapid churn (#690).
pub struct DdsToZenohRoute {
    _reader: DdsEntity,
    /// Keeps the TopicPublisherSlot alive while this reader is active.
    _publisher: Arc<TopicPublisherSlot>,
    topic_name: String,
}

impl DdsToZenohRoute {
    pub(crate) fn topic_name(&self) -> &str {
        &self.topic_name
    }

    pub async fn create(
        dp: dds_entity_t,
        endpoint: &DiscoveredEndpoint,
        session: &Session,
        namespace: Option<&str>,
        reliable_routes_blocking: bool,
        topic_publishers: &Arc<
            Mutex<std::collections::HashMap<(u32, String), Arc<TopicPublisherSlot>>>,
        >,
        domain_id: u32,
    ) -> Result<Self> {
        let topic_name = endpoint.topic_name.clone();
        let slot_key = (domain_id, topic_name.clone());

        // Get or create the shared publisher slot for this topic.
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
                    )
                    .await?,
                );
                map.insert(slot_key, Arc::clone(&new_pub));
                new_pub
            }
        };

        let qos = adapt_writer_qos_for_reader(&endpoint.qos);

        // G3: warn on QoS incompatibility between writer and adapted reader.
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

        let reader_handle = create_blob_reader(
            dp,
            &topic_name,
            &type_name,
            keyless,
            qos,
            move |sample: DDSRawSample| {
                let bytes: ZBytes = sample.as_slice().into();
                // G4: attach the ROS 2 type name so Zenoh receivers can identify the type.
                let attachment = Some(ros2_type.as_bytes().to_vec());
                if let Err(e) = pub_clone.put_wait(bytes, attachment) {
                    tracing::warn!("Zenoh put failed on {ke_display}: {e}");
                }
            },
        )?;

        Ok(Self {
            _reader: unsafe { DdsEntity::new(reader_handle) },
            _publisher: arc_pub,
            topic_name,
        })
    }
}

// ─── compute_cache_size ───────────────────────────────────────────────────────

/// Compute the AdvancedPublisher cache size from DDS QoS.
///
/// Mirrors the zenoh-plugin-ros2dds formula: history.depth × cache_multiplier,
/// bounded by usize::MAX for KEEP_ALL or unlimited instances.
fn compute_cache_size(qos: &cyclors::qos::Qos) -> usize {
    let depth = match &qos.history {
        Some(h) => match h.kind {
            HistoryKind::KEEP_ALL => return usize::MAX,
            HistoryKind::KEEP_LAST => h.depth as usize,
        },
        None => 1,
    };
    depth.saturating_mul(TRANSIENT_LOCAL_CACHE_MULTIPLIER)
}

// ─── ZenohToDdsRoute ─────────────────────────────────────────────────────────

/// Holds a Zenoh subscriber handle (plain or advanced) for RAII lifetime management.
#[allow(dead_code)]
enum ZenohSubscriberHandle {
    Plain(Subscriber<()>),
    /// Used for TRANSIENT_LOCAL DDS readers: receives historical samples published
    /// before this subscriber was created (`detect_late_publishers()`).
    Advanced(AdvancedSubscriber<()>),
}

// Safety: Subscriber<()> and AdvancedSubscriber<()> are Send+Sync.
unsafe impl Send for ZenohSubscriberHandle {}
unsafe impl Sync for ZenohSubscriberHandle {}

/// A route from a Zenoh subscriber to a DDS writer.
///
/// Receives CDR bytes from Zenoh and forwards them verbatim to DDS.
/// For TRANSIENT_LOCAL DDS readers, uses an AdvancedSubscriber with history
/// so late-joining DDS readers receive samples published before they appeared.
pub struct ZenohToDdsRoute {
    _writer: DdsEntity,
    _subscriber: ZenohSubscriberHandle,
}

impl ZenohToDdsRoute {
    pub async fn create(
        dp: dds_entity_t,
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

        // G3: warn on QoS incompatibility between reader and adapted writer.
        if let Some(reason) = qos_mismatch_reason(&qos, &endpoint.qos) {
            tracing::warn!("QoS mismatch on {}: {}", endpoint.topic_name, reason);
        }

        let writer_handle = create_blob_writer(
            dp,
            &endpoint.topic_name,
            &endpoint.type_name,
            endpoint.keyless,
            qos,
        )?;
        let writer_entity = unsafe { DdsEntity::new(writer_handle) };
        let writer_raw = writer_entity.raw();

        let ke_display = ke.to_string();
        let dds_topic = endpoint.topic_name.clone();
        let endpoint_is_transient_local = is_transient_local(&endpoint.qos);
        tracing::info!("Zenoh→DDS pub/sub route: {ke_display} → {dds_topic}");

        let subscriber_handle = if endpoint_is_transient_local {
            tracing::debug!(
                "Zenoh→DDS: TRANSIENT_LOCAL reader on {dds_topic}, using AdvancedSubscriber"
            );
            let adv = session
                .declare_subscriber(ke.clone())
                .history(HistoryConfig::default().detect_late_publishers())
                .callback(move |sample| {
                    let bytes: Vec<u8> = sample.payload().to_bytes().into_owned();
                    if let Err(e) = write_cdr(writer_raw, bytes) {
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
                    if let Err(e) = write_cdr(writer_raw, bytes) {
                        tracing::warn!("DDS write failed on {ke_display}: {e}");
                    }
                })
                .await
                .map_err(|e| anyhow!("declare_subscriber failed: {e}"))?;
            ZenohSubscriberHandle::Plain(sub)
        };

        Ok(Self {
            _writer: writer_entity,
            _subscriber: subscriber_handle,
        })
    }
}

// ─── tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use cyclors::qos::{Durability, DurabilityKind, History, HistoryKind, Qos};

    use super::compute_cache_size;
    use crate::dds::names::dds_type_to_ros2_type;

    fn qos_transient_local(depth: i32) -> Qos {
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

    fn qos_volatile(depth: i32) -> Qos {
        Qos {
            history: Some(History {
                kind: HistoryKind::KEEP_LAST,
                depth,
            }),
            ..Default::default()
        }
    }

    fn qos_keep_all() -> Qos {
        Qos {
            durability: Some(Durability {
                kind: DurabilityKind::TRANSIENT_LOCAL,
            }),
            history: Some(History {
                kind: HistoryKind::KEEP_ALL,
                depth: 0,
            }),
            ..Default::default()
        }
    }

    #[test]
    fn test_cache_size_keep_last() {
        assert_eq!(compute_cache_size(&qos_transient_local(1)), 10);
        assert_eq!(compute_cache_size(&qos_transient_local(5)), 50);
    }

    #[test]
    fn test_cache_size_no_history_defaults_to_multiplier() {
        assert_eq!(compute_cache_size(&Qos::default()), 10);
    }

    #[test]
    fn test_cache_size_keep_all_is_max() {
        assert_eq!(compute_cache_size(&qos_keep_all()), usize::MAX);
    }

    #[test]
    fn test_volatile_cache_size_still_computed() {
        assert_eq!(compute_cache_size(&qos_volatile(3)), 30);
    }

    // G4: type name attachment

    #[test]
    fn test_type_name_attachment_string_type() {
        let dds_type = "std_msgs::msg::dds_::String_";
        let ros2_type = dds_type_to_ros2_type(dds_type);
        assert_eq!(ros2_type.as_bytes(), b"std_msgs/msg/String");
    }

    #[test]
    fn test_type_name_attachment_full_dds_type_preserved() {
        // pub/sub uses dds_type_to_ros2_type (not service variant) — full type string
        let dds_type = "example_interfaces::msg::dds_::Int64_";
        let ros2_type = dds_type_to_ros2_type(dds_type);
        assert_eq!(ros2_type, "example_interfaces/msg/Int64");
    }

    // G1: shared publisher slot (Arc ref-count logic)

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
