use std::sync::Arc;

use anyhow::{Result, anyhow};
use cyclors::{
    dds_entity_t,
    qos::{DurabilityKind, HistoryKind, ReliabilityKind},
};
use zenoh::{
    Session, Wait,
    bytes::ZBytes,
    key_expr::KeyExpr,
    pubsub::{Publisher, Subscriber},
    qos::CongestionControl,
};
use zenoh_ext::{AdvancedPublisher, AdvancedPublisherBuilderExt, CacheConfig};

use crate::dds::{
    discovery::DiscoveredEndpoint,
    entity::DdsEntity,
    names::{dds_topic_to_ros2_name, ros2_name_to_zenoh_key},
    qos::{adapt_reader_qos_for_writer, adapt_writer_qos_for_reader},
    reader::create_blob_reader,
    types::DDSRawSample,
    writer::{create_blob_writer, write_cdr},
};

const TRANSIENT_LOCAL_CACHE_MULTIPLIER: usize = 10;

/// Unified publisher handle: plain or TRANSIENT_LOCAL with history cache.
enum BridgePublisher {
    Plain(Publisher<'static>),
    Cached(Arc<AdvancedPublisher<'static>>),
}

impl BridgePublisher {
    fn put_wait(&self, bytes: ZBytes) -> Result<()> {
        match self {
            Self::Plain(p) => p
                .put(bytes)
                .wait()
                .map_err(|e| anyhow!("Zenoh put failed: {e}")),
            Self::Cached(p) => p
                .put(bytes)
                .wait()
                .map_err(|e| anyhow!("Zenoh put (cached) failed: {e}")),
        }
    }
}

// Safety: Publisher<'static> is Send; AdvancedPublisher<'static> is Send
unsafe impl Send for BridgePublisher {}

/// A route from a DDS publication to a Zenoh publisher.
///
/// Receives CDR bytes from DDS and forwards them verbatim to Zenoh.
/// When the DDS writer is TRANSIENT_LOCAL, an AdvancedPublisher with a history
/// cache is used so late-joining Zenoh subscribers receive historical samples.
pub struct DdsToZenohRoute {
    _reader: DdsEntity,
}

impl DdsToZenohRoute {
    pub async fn create(
        dp: dds_entity_t,
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

        let publisher: BridgePublisher = if is_transient_local {
            let cache_size = compute_cache_size(&endpoint.qos);
            tracing::debug!(
                "DDS→Zenoh: TRANSIENT_LOCAL topic {}, cache_size={}",
                endpoint.topic_name,
                cache_size
            );
            let adv = session
                .declare_publisher(ke.clone())
                .congestion_control(congestion_ctrl)
                .cache(CacheConfig::default().max_samples(cache_size))
                .publisher_detection()
                .await
                .map_err(|e| anyhow!("declare_publisher(advanced) failed: {e}"))?;
            BridgePublisher::Cached(Arc::new(adv))
        } else {
            let plain = session
                .declare_publisher(ke.clone())
                .congestion_control(congestion_ctrl)
                .await
                .map_err(|e| anyhow!("declare_publisher failed: {e}"))?;
            BridgePublisher::Plain(plain)
        };

        let qos = adapt_writer_qos_for_reader(&endpoint.qos);
        let topic_name = endpoint.topic_name.clone();
        let type_name = endpoint.type_name.clone();
        let keyless = endpoint.keyless;
        let ke_display = ke.to_string();

        tracing::info!("DDS→Zenoh pub/sub route: {topic_name} → {ke_display}");

        let reader_handle = create_blob_reader(
            dp,
            &topic_name,
            &type_name,
            keyless,
            qos,
            move |sample: DDSRawSample| {
                let bytes: ZBytes = sample.as_slice().into();
                if let Err(e) = publisher.put_wait(bytes) {
                    tracing::warn!("Zenoh put failed on {ke_display}: {e}");
                }
            },
        )?;

        Ok(Self {
            _reader: unsafe { DdsEntity::new(reader_handle) },
        })
    }
}

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

/// A route from a Zenoh subscriber to a DDS writer.
///
/// Receives CDR bytes from Zenoh and forwards them verbatim to DDS.
pub struct ZenohToDdsRoute {
    // Kept to extend RAII lifetime
    _writer: DdsEntity,
    _subscriber: Subscriber<()>,
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
        tracing::info!("Zenoh→DDS pub/sub route: {ke_display} → {dds_topic}");

        let subscriber = session
            .declare_subscriber(ke.clone())
            .callback(move |sample| {
                let bytes: Vec<u8> = sample.payload().to_bytes().into_owned();
                if let Err(e) = write_cdr(writer_raw, bytes) {
                    tracing::warn!("DDS write failed on {ke_display}: {e}");
                }
            })
            .await
            .map_err(|e| anyhow!("declare_subscriber failed: {e}"))?;

        Ok(Self {
            _writer: writer_entity,
            _subscriber: subscriber,
        })
    }
}

#[cfg(test)]
mod tests {
    use cyclors::qos::{Durability, DurabilityKind, History, HistoryKind, Qos};

    use super::compute_cache_size;

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
        // depth=1 → 1 × 10 = 10
        assert_eq!(compute_cache_size(&qos_transient_local(1)), 10);
        // depth=5 → 5 × 10 = 50
        assert_eq!(compute_cache_size(&qos_transient_local(5)), 50);
    }

    #[test]
    fn test_cache_size_no_history_defaults_to_multiplier() {
        let qos = Qos::default();
        // no history → depth=1, 1 × 10 = 10
        assert_eq!(compute_cache_size(&qos), 10);
    }

    #[test]
    fn test_cache_size_keep_all_is_max() {
        assert_eq!(compute_cache_size(&qos_keep_all()), usize::MAX);
    }

    #[test]
    fn test_volatile_cache_size_still_computed() {
        // compute_cache_size is called regardless of durability; bridge code gates on is_transient_local
        assert_eq!(compute_cache_size(&qos_volatile(3)), 30);
    }
}
