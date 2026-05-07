use std::time::Duration;

use anyhow::Result;

use super::discovery::DiscoveryEvent;

// ─── QoS kinds ───────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReliabilityKind {
    BestEffort,
    Reliable,
}

impl ReliabilityKind {
    /// Wire discriminant matching the DDS / zenoh-plugin-ros2dds integer encoding.
    pub fn wire_discriminant(self) -> i32 {
        match self {
            Self::BestEffort => 0,
            Self::Reliable => 1,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DurabilityKind {
    Volatile,
    TransientLocal,
    Transient,
    Persistent,
}

impl DurabilityKind {
    pub fn wire_discriminant(self) -> i32 {
        match self {
            Self::Volatile => 0,
            Self::TransientLocal => 1,
            Self::Transient => 2,
            Self::Persistent => 3,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoryKind {
    KeepLast,
    KeepAll,
}

impl HistoryKind {
    pub fn wire_discriminant(self) -> i32 {
        match self {
            Self::KeepLast => 0,
            Self::KeepAll => 1,
        }
    }
}

// ─── QoS policy structs ───────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq)]
pub struct Reliability {
    pub kind: ReliabilityKind,
    /// `None` = DDS infinite blocking time.
    pub max_blocking_time: Option<Duration>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Durability {
    pub kind: DurabilityKind,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct History {
    pub kind: HistoryKind,
    pub depth: i32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DurabilityService {
    /// `None` = DDS infinite cleanup delay.
    pub service_cleanup_delay: Option<Duration>,
    pub history_kind: HistoryKind,
    pub history_depth: i32,
    /// `None` = DDS_LENGTH_UNLIMITED.
    pub max_samples: Option<usize>,
    /// `None` = DDS_LENGTH_UNLIMITED.
    pub max_instances: Option<usize>,
    /// `None` = DDS_LENGTH_UNLIMITED.
    pub max_samples_per_instance: Option<usize>,
}

// ─── BridgeQos ────────────────────────────────────────────────────────────────

/// Backend-independent QoS representation for the bridge.
///
/// Covers the DDS-standard policies needed for pub/sub and service routing.
/// Backend-specific fields (e.g. CycloneDDS `entity_name`, `properties`,
/// `writer_batching`) are handled inside each backend's conversion layer.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BridgeQos {
    pub reliability: Option<Reliability>,
    pub durability: Option<Durability>,
    pub history: Option<History>,
    pub durability_service: Option<DurabilityService>,
    /// `None` = infinite deadline.
    pub deadline: Option<Duration>,
    pub latency_budget: Option<Duration>,
    /// `None` = infinite lifespan.
    pub lifespan: Option<Duration>,
    pub user_data: Option<Vec<u8>>,
    /// When true the reader/writer ignores traffic from other endpoints
    /// in the same DDS participant (IgnoreLocal::PARTICIPANT).
    pub ignore_local: bool,
}

// ─── Backend traits ───────────────────────────────────────────────────────────

/// A live DDS participant on a single domain that can create readers/writers
/// and stream discovery events.
///
/// Implement this trait to plug in a DDS library.  The cyclors (CycloneDDS)
/// implementation lives in `dds::cyclors`.
pub trait DdsParticipant: Send + Sync + 'static {
    /// Opaque RAII reader handle — dropped when the reader should be destroyed.
    type Reader: Send + Sync + 'static;
    /// Writer handle that exposes `write_cdr` and `instance_handle`.
    type Writer: DdsWriter;

    /// Create a participant on `domain_id`.
    fn create(domain_id: u32) -> Result<Self>
    where
        Self: Sized;

    /// Start the builtin-topic discovery loop.
    ///
    /// Endpoint events are sent on the returned channel until the participant
    /// (and therefore the channel sender) is dropped.
    fn run_discovery(&self) -> flume::Receiver<DiscoveryEvent>;

    /// Create a CDR blob reader; `callback` is called for each valid sample.
    ///
    /// The `Vec<u8>` passed to `callback` includes the 4-byte CDR representation
    /// header followed by the payload, matching the raw DDS wire format.
    fn create_reader(
        &self,
        topic: &str,
        type_name: &str,
        keyless: bool,
        qos: BridgeQos,
        callback: Box<dyn Fn(Vec<u8>) + Send + 'static>,
    ) -> Result<Self::Reader>;

    /// Create a CDR blob writer.
    fn create_writer(
        &self,
        topic: &str,
        type_name: &str,
        keyless: bool,
        qos: BridgeQos,
    ) -> Result<Self::Writer>;
}

/// A live DDS writer that can send raw CDR blobs.
pub trait DdsWriter: Send + Sync + 'static {
    /// Write raw CDR bytes (4-byte representation header + payload) to DDS.
    fn write_cdr(&self, data: Vec<u8>) -> Result<()>;

    /// Return this writer's stable instance handle (GUID fragment).
    ///
    /// Used as the `client_guid` in ROS 2 service request headers so that
    /// the DDS server routes replies back to this specific writer.
    fn instance_handle(&self) -> Result<u64>;
}
