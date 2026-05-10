//! TF2 transform listener and buffer for ros-z.
//!
//! Subscribes to `/tf` (dynamic) and `/tf_static` (TransientLocal) and provides
//! `lookup_transform` with multi-hop LCA traversal and linear/slerp interpolation.
//!
//! # Quick start
//!
//! ```rust,ignore
//! use ros_z::prelude::*;
//! use ros_z_tf::Buffer;
//!
//! #[tokio::main]
//! async fn main() -> zenoh::Result<()> {
//!     let ctx = ZContextBuilder::default()
//!         .with_connect_endpoints(["tcp/127.0.0.1:7447"])
//!         .build()?;
//!     let node = ctx.create_node("tf_listener").build()?;
//!     let buffer = Buffer::new(&node)?;
//!
//!     tokio::time::sleep(std::time::Duration::from_millis(500)).await;
//!
//!     match buffer.lookup_transform("map", "base_link", ZTime::zero()) {
//!         Ok(tf) => println!("x={}", tf.transform.translation.x),
//!         Err(e) => eprintln!("lookup failed: {e}"),
//!     }
//!     Ok(())
//! }
//! ```

use std::fmt;
use std::num::NonZeroUsize;
use std::sync::Arc;
use std::time::Duration;

use parking_lot::RwLock;
use ros_z::msg::NativeCdrSerdes;
use ros_z::node::ZNode;
use ros_z::pubsub::ZSub;
use ros_z::qos::{QosDurability, QosHistory, QosProfile, QosReliability};
use ros_z::time::ZTime;
use ros_z_msgs::geometry_msgs::TransformStamped;
use ros_z_msgs::tf2_msgs::TFMessage;
use tokio::sync::Notify;

mod broadcaster;
mod buffer;
mod lookup;
mod math;

pub use broadcaster::{StaticTransformBroadcaster, TransformBroadcaster};

use buffer::BufferInner;

type TfSub = ZSub<TFMessage, (), NativeCdrSerdes<TFMessage>>;

/// Error returned by [`Buffer::lookup_transform`].
#[derive(Debug)]
pub enum LookupError {
    /// The requested frame has no known transforms.
    UnknownFrame(String),
    /// `source` and `target` are in disconnected sub-trees.
    NoCommonAncestor { source: String, target: String },
    /// The requested timestamp is outside the stored history window.
    ExtrapolationError {
        frame: String,
        requested: ZTime,
        oldest: ZTime,
        newest: ZTime,
    },
}

impl fmt::Display for LookupError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LookupError::UnknownFrame(frame) => {
                write!(f, "frame '{frame}' has no known transforms")
            }
            LookupError::NoCommonAncestor { source, target } => {
                write!(f, "no common ancestor between '{source}' and '{target}'")
            }
            LookupError::ExtrapolationError {
                frame,
                requested,
                oldest,
                newest,
            } => {
                write!(
                    f,
                    "requested time {:?} for frame '{}' is outside buffer window [{:?}, {:?}]",
                    requested, frame, oldest, newest
                )
            }
        }
    }
}

impl std::error::Error for LookupError {}

/// Error returned by [`Buffer::wait_for_transform`].
#[derive(Debug)]
pub enum WaitError {
    /// The timeout elapsed before the transform became available.
    Timeout,
    /// The lookup failed with an error that will not resolve with more time
    /// (e.g., disconnected frame trees).
    Lookup(LookupError),
}

impl fmt::Display for WaitError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            WaitError::Timeout => write!(f, "wait_for_transform timed out"),
            WaitError::Lookup(e) => write!(f, "lookup failed permanently: {e}"),
        }
    }
}

impl std::error::Error for WaitError {}

/// TF2 transform buffer and listener.
///
/// Subscribes to `/tf` and `/tf_static` on the provided node and maintains
/// an in-memory frame tree.  Drop this value to cancel the subscriptions.
///
/// Create with [`Buffer::new`].
pub struct Buffer {
    inner: Arc<RwLock<BufferInner>>,
    notify: Arc<Notify>,
    _tf_sub: TfSub,
    _tf_static_sub: TfSub,
}

impl Buffer {
    /// Subscribe to `/tf` and `/tf_static` on `node` and return a new buffer.
    pub fn new(node: &ZNode) -> zenoh::Result<Self> {
        let inner = Arc::new(RwLock::new(BufferInner::default()));

        let dynamic_qos = QosProfile {
            reliability: QosReliability::Reliable,
            durability: QosDurability::Volatile,
            history: QosHistory::KeepLast(NonZeroUsize::new(100).unwrap()),
            ..Default::default()
        };

        let static_qos = QosProfile {
            reliability: QosReliability::Reliable,
            durability: QosDurability::TransientLocal,
            history: QosHistory::KeepLast(NonZeroUsize::new(100).unwrap()),
            ..Default::default()
        };

        let inner_dyn = Arc::clone(&inner);
        let tf_sub = node
            .create_sub::<TFMessage>("/tf")
            .with_qos(dynamic_qos)
            .build_with_callback(move |msg: TFMessage| {
                inner_dyn.write().add_message(msg, false);
            })?;

        let inner_static = Arc::clone(&inner);
        let tf_static_sub = node
            .create_sub::<TFMessage>("/tf_static")
            .with_qos(static_qos)
            .build_with_callback(move |msg: TFMessage| {
                inner_static.write().add_message(msg, true);
            })?;

        let notify = Arc::clone(&inner.read().notify);
        Ok(Buffer {
            inner,
            notify,
            _tf_sub: tf_sub,
            _tf_static_sub: tf_static_sub,
        })
    }

    /// Look up the transform from `source` frame to `target` frame at `time`.
    ///
    /// Pass [`ZTime::zero()`] to request the latest available transform.
    ///
    /// The returned `TransformStamped` maps a point expressed in `source`
    /// coordinates into `target` coordinates.
    pub fn lookup_transform(
        &self,
        target: &str,
        source: &str,
        time: ZTime,
    ) -> Result<TransformStamped, LookupError> {
        self.inner.read().lookup(target, source, time)
    }

    /// Return `true` if [`lookup_transform`](Self::lookup_transform) would
    /// succeed for the given frames and time.
    pub fn can_transform(&self, target: &str, source: &str, time: ZTime) -> bool {
        self.inner.read().lookup(target, source, time).is_ok()
    }

    /// Return all frame IDs currently known to the buffer.
    pub fn all_frames(&self) -> Vec<String> {
        self.inner.read().all_frames()
    }

    /// Look up the transform from `source` at `time` to `target` at `fixed_time`,
    /// routing through `fixed_frame`.
    ///
    /// Equivalent to:
    /// ```text
    /// T(target ← source) = T(target ← fixed_frame, fixed_time) ∘ T(fixed_frame ← source, time)
    /// ```
    ///
    /// Used when target and source are observed at different times and need to be
    /// related through a fixed reference frame (typically `"map"`).
    pub fn lookup_transform_full(
        &self,
        target: &str,
        source: &str,
        time: ZTime,
        fixed_frame: &str,
        fixed_time: ZTime,
    ) -> Result<TransformStamped, LookupError> {
        let inner = self.inner.read();
        let t1 = inner.lookup(fixed_frame, source, time)?;
        let t2 = inner.lookup(target, fixed_frame, fixed_time)?;
        Ok(crate::math::compose_stamped(t2, t1, target, source))
    }

    /// Wait asynchronously until `lookup_transform` succeeds or `timeout` elapses.
    ///
    /// Returns `Err(WaitError::Timeout)` if no transform arrives within the
    /// deadline. Returns `Err(WaitError::Lookup(...))` immediately if the
    /// frames are in disconnected trees (waiting cannot resolve the error).
    pub async fn wait_for_transform(
        &self,
        target: &str,
        source: &str,
        time: ZTime,
        timeout: Duration,
    ) -> Result<TransformStamped, WaitError> {
        let deadline = tokio::time::Instant::now() + timeout;
        loop {
            match self.inner.read().lookup(target, source, time) {
                Ok(tf) => return Ok(tf),
                Err(LookupError::NoCommonAncestor {
                    source: s,
                    target: t,
                }) => {
                    return Err(WaitError::Lookup(LookupError::NoCommonAncestor {
                        source: s,
                        target: t,
                    }));
                }
                Err(_) => {}
            }
            let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
            if remaining.is_zero() {
                return Err(WaitError::Timeout);
            }
            // Wait for new data or until the deadline, whichever comes first.
            let _ = tokio::time::timeout(remaining, self.notify.notified()).await;
        }
    }
}
