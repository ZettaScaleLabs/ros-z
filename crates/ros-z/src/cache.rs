//! Timestamp-indexed, capacity-bounded message cache.
//!
//! [`ZCache`](crate::cache::ZCache) provides the core functionality of ROS 2's
//! `message_filters::Cache<T>`: retain a sliding window of received messages
//! and query them by time.
//!
//! # Stamp strategies
//!
//! Two indexing strategies are available, selected at build time:
//!
//! - **[`ZenohStamp`](crate::cache::ZenohStamp)** (default) — indexes by the
//!   Zenoh transport timestamp (`uhlc::Timestamp` → `SystemTime`). Zero-config;
//!   works for any message type as long as timestamping is enabled in the Zenoh
//!   config (already enabled in the ros-z default config).
//! - **[`ExtractorStamp`](crate::cache::ExtractorStamp)** — indexes by a
//!   user-supplied closure that extracts a `SystemTime` from each deserialized
//!   message. Required for `header.stamp` / sensor capture time alignment.
//!
//! # Example
//!
//! ```rust,ignore
//! use ros_z::prelude::*;
//! use ros_z_msgs::sensor_msgs::Imu;
//! use std::time::{Duration, SystemTime};
//!
//! let ctx = ZContextBuilder::default().build()?;
//! let node = ctx.create_node("cache_demo").build()?;
//!
//! // Zero-config: indexed by Zenoh transport timestamp
//! let cache = node.create_cache::<Imu>("/imu/data", 200).build()?;
//!
//! let now = SystemTime::now();
//! let window = cache.get_interval(now - Duration::from_millis(100), now);
//!
//! // Application timestamp: indexed by header.stamp
//! let cache = node
//!     .create_cache::<Imu>("/imu/data", 200)
//!     .with_stamp(|msg: &Imu| {
//!         let sec = msg.header.stamp.sec as u64;
//!         let nsec = msg.header.stamp.nanosec;
//!         SystemTime::UNIX_EPOCH + Duration::new(sec, nsec)
//!     })
//!     .build()?;
//! ```

use std::collections::BTreeMap;
use std::marker::PhantomData;
use std::sync::Arc;
use std::time::SystemTime;

use parking_lot::Mutex;
use tracing::{debug, warn};
use zenoh::liveliness::LivelinessToken;
use zenoh::{Result, Wait};

use crate::Builder;

use crate::msg::{CdrSerdes, ZDeserializer, ZMessage};
use crate::pubsub::ZSubBuilder;
use crate::topic_name;

// ---------------------------------------------------------------------------
// Stamp strategy markers
// ---------------------------------------------------------------------------

/// Index by the Zenoh transport timestamp (`uhlc::Timestamp` → `SystemTime`).
///
/// This is the default stamp strategy. It works for any message type without
/// any configuration. If the incoming [`zenoh::sample::Sample`] carries no
/// timestamp (timestamping disabled on the peer), the cache falls back to
/// `SystemTime::now()` at receive time and logs a one-time warning.
pub struct ZenohStamp;

/// Index by an application-supplied extractor closure.
///
/// The closure receives a reference to the deserialized message and returns a
/// `SystemTime` representing its logical timestamp (e.g. `header.stamp`).
pub struct ExtractorStamp<T, F: Fn(&T) -> SystemTime>(pub(crate) F, pub(crate) PhantomData<T>);

// ---------------------------------------------------------------------------
// CacheInner — shared mutable state
// ---------------------------------------------------------------------------

struct CacheInner<T> {
    entries: BTreeMap<SystemTime, T>,
    capacity: usize,
    /// Guards against logging the missing-timestamp warning more than once.
    warned_no_ts: bool,
}

impl<T> CacheInner<T> {
    fn new(capacity: usize) -> Self {
        Self {
            entries: BTreeMap::new(),
            capacity,
            warned_no_ts: false,
        }
    }

    fn insert(&mut self, stamp: SystemTime, msg: T) {
        self.entries.insert(stamp, msg);
        // Evict the oldest entry when over capacity.
        while self.entries.len() > self.capacity {
            self.entries.pop_first();
        }
    }
}

// ---------------------------------------------------------------------------
// ZCache
// ---------------------------------------------------------------------------

/// A timestamp-indexed, capacity-bounded sliding-window cache of received
/// messages.
///
/// Built via [`ZCacheBuilder`], created through
/// [`ZNode::create_cache`](crate::node::ZNode::create_cache).
///
/// Dropping `ZCache` automatically deregisters the underlying Zenoh subscriber.
pub struct ZCache<T: ZMessage> {
    inner: Arc<Mutex<CacheInner<T>>>,
    _sub: zenoh::pubsub::Subscriber<()>,
    _lv_token: LivelinessToken,
}

impl<T: ZMessage + Clone> ZCache<T> {
    /// All messages with timestamp in `[t_start, t_end]`, inclusive, ordered
    /// by timestamp ascending.
    ///
    /// Returns clones of the stored messages. If `t_start > t_end` the result
    /// is always empty (no panic).
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let window = cache.get_interval(
    ///     SystemTime::now() - Duration::from_millis(100),
    ///     SystemTime::now(),
    /// );
    /// ```
    pub fn get_interval(&self, t_start: SystemTime, t_end: SystemTime) -> Vec<T> {
        if t_start > t_end {
            return Vec::new();
        }
        let inner = self.inner.lock();
        inner
            .entries
            .range(t_start..=t_end)
            .map(|(_, v)| v.clone())
            .collect()
    }

    /// The most recent message with timestamp ≤ `t`, or `None` if the cache is
    /// empty or all messages are strictly after `t`.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let latest = cache.get_before(SystemTime::now());
    /// ```
    pub fn get_before(&self, t: SystemTime) -> Option<T> {
        let inner = self.inner.lock();
        inner
            .entries
            .range(..=t)
            .next_back()
            .map(|(_, v)| v.clone())
    }

    /// The earliest message with timestamp ≥ `t`, or `None` if the cache is
    /// empty or all messages are strictly before `t`.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let next = cache.get_after(camera_timestamp);
    /// ```
    pub fn get_after(&self, t: SystemTime) -> Option<T> {
        let inner = self.inner.lock();
        inner.entries.range(t..).next().map(|(_, v)| v.clone())
    }

    /// The message whose timestamp is nearest to `t` (either side).
    ///
    /// When two messages are equidistant, the one with the earlier (before)
    /// timestamp is returned.
    ///
    /// Returns `None` if the cache is empty.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let nearest_imu = cache.get_nearest(camera_stamp);
    /// ```
    pub fn get_nearest(&self, t: SystemTime) -> Option<T> {
        let inner = self.inner.lock();
        if inner.entries.is_empty() {
            return None;
        }

        let before = inner.entries.range(..=t).next_back().map(|(k, v)| (*k, v));
        let after = inner.entries.range(t..).next().map(|(k, v)| (*k, v));

        match (before, after) {
            (None, Some((_, v))) => Some(v.clone()),
            (Some((_, v)), None) => Some(v.clone()),
            (Some((kb, vb)), Some((ka, va))) => {
                // Compute distances. SystemTime subtraction can return Duration
                // only if the result is non-negative.
                let dist_before = t.duration_since(kb).unwrap_or_default();
                let dist_after = ka.duration_since(t).unwrap_or_default();
                // On a tie prefer earlier (before) timestamp.
                if dist_after < dist_before {
                    Some(va.clone())
                } else {
                    Some(vb.clone())
                }
            }
            (None, None) => None,
        }
    }

    /// Timestamp of the oldest cached message, or `None` if empty.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// if let Some(oldest) = cache.oldest_stamp() {
    ///     println!("Cache starts at {:?}", oldest);
    /// }
    /// ```
    pub fn oldest_stamp(&self) -> Option<SystemTime> {
        self.inner.lock().entries.keys().next().copied()
    }

    /// Timestamp of the newest cached message, or `None` if empty.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// if let Some(newest) = cache.newest_stamp() {
    ///     println!("Cache ends at {:?}", newest);
    /// }
    /// ```
    pub fn newest_stamp(&self) -> Option<SystemTime> {
        self.inner.lock().entries.keys().next_back().copied()
    }

    /// Number of messages currently in the cache.
    pub fn len(&self) -> usize {
        self.inner.lock().entries.len()
    }

    /// `true` if the cache holds no messages.
    pub fn is_empty(&self) -> bool {
        self.inner.lock().entries.is_empty()
    }

    /// Remove all messages from the cache.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// cache.clear();
    /// assert!(cache.is_empty());
    /// ```
    pub fn clear(&self) {
        self.inner.lock().entries.clear();
    }
}

// ---------------------------------------------------------------------------
// ZCacheBuilder
// ---------------------------------------------------------------------------

/// Builder for [`ZCache<T>`].
///
/// Created by [`ZNode::create_cache`](crate::node::ZNode::create_cache).
/// Use [`with_stamp`](ZCacheBuilder::with_stamp) to switch from the default
/// Zenoh transport timestamp to an application-level extractor.
pub struct ZCacheBuilder<T, S = CdrSerdes<T>, Stamp = ZenohStamp> {
    pub(crate) sub_builder: ZSubBuilder<T, S>,
    capacity: usize,
    stamp: Stamp,
}

impl<T: ZMessage, S> ZCacheBuilder<T, S, ZenohStamp> {
    pub(crate) fn new(sub_builder: ZSubBuilder<T, S>, capacity: usize) -> Self {
        Self {
            sub_builder,
            capacity,
            stamp: ZenohStamp,
        }
    }

    /// Switch to application-level timestamp extraction.
    ///
    /// The extractor receives a reference to the deserialized message and
    /// returns a `SystemTime` representing its logical timestamp (e.g.
    /// `header.stamp`).
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// use ros_z_msgs::sensor_msgs::Imu;
    /// use std::time::{Duration, SystemTime};
    ///
    /// let cache = node
    ///     .create_cache::<Imu>("/imu/data", 200)
    ///     .with_stamp(|msg: &Imu| {
    ///         SystemTime::UNIX_EPOCH
    ///             + Duration::new(msg.header.stamp.sec as u64, msg.header.stamp.nanosec)
    ///     })
    ///     .build()?;
    /// ```
    pub fn with_stamp<F>(self, extractor: F) -> ZCacheBuilder<T, S, ExtractorStamp<T, F>>
    where
        F: Fn(&T) -> SystemTime + Send + Sync + 'static,
    {
        ZCacheBuilder {
            sub_builder: self.sub_builder,
            capacity: self.capacity,
            stamp: ExtractorStamp(extractor, PhantomData),
        }
    }

    /// Maximum number of messages to retain. Oldest are evicted when full.
    ///
    /// Defaults to the value passed to
    /// [`ZNode::create_cache`](crate::node::ZNode::create_cache).
    pub fn with_capacity(mut self, capacity: usize) -> Self {
        self.capacity = capacity;
        self
    }

    /// Apply a QoS profile to the underlying subscriber.
    pub fn with_qos(mut self, qos: crate::qos::QosProfile) -> Self {
        self.sub_builder = self.sub_builder.with_qos(qos);
        self
    }
}

impl<T: ZMessage, S, F> ZCacheBuilder<T, S, ExtractorStamp<T, F>>
where
    F: Fn(&T) -> SystemTime + Send + Sync + 'static,
{
    /// Maximum number of messages to retain. Oldest are evicted when full.
    pub fn with_capacity(mut self, capacity: usize) -> Self {
        self.capacity = capacity;
        self
    }

    /// Apply a QoS profile to the underlying subscriber.
    pub fn with_qos(mut self, qos: crate::qos::QosProfile) -> Self {
        self.sub_builder = self.sub_builder.with_qos(qos);
        self
    }
}

// ---------------------------------------------------------------------------
// Builder impl — ZenohStamp variant
// ---------------------------------------------------------------------------

impl<T, S> Builder for ZCacheBuilder<T, S, ZenohStamp>
where
    T: ZMessage + Send + Sync + 'static,
    S: for<'a> ZDeserializer<Input<'a> = &'a [u8], Output = T> + 'static,
{
    type Output = ZCache<T>;

    fn build(self) -> Result<ZCache<T>> {
        let ZCacheBuilder {
            sub_builder,
            capacity,
            ..
        } = self;

        build_cache_from_sub::<T, S, _>(
            sub_builder,
            capacity,
            |sample: &zenoh::sample::Sample, inner: &mut CacheInner<T>, msg: T| {
                let stamp = match sample.timestamp() {
                    Some(ts) => ts.get_time().to_system_time(),
                    None => {
                        if !inner.warned_no_ts {
                            warn!(
                                "[CACHE] Incoming sample has no Zenoh timestamp; \
                                 falling back to SystemTime::now(). \
                                 Enable timestamping in the Zenoh config to avoid this."
                            );
                            inner.warned_no_ts = true;
                        }
                        SystemTime::now()
                    }
                };
                inner.insert(stamp, msg);
            },
        )
    }
}

// ---------------------------------------------------------------------------
// Builder impl — ExtractorStamp variant
// ---------------------------------------------------------------------------

impl<T, S, F> Builder for ZCacheBuilder<T, S, ExtractorStamp<T, F>>
where
    T: ZMessage + Send + Sync + 'static,
    S: for<'a> ZDeserializer<Input<'a> = &'a [u8], Output = T> + 'static,
    F: Fn(&T) -> SystemTime + Send + Sync + 'static,
{
    type Output = ZCache<T>;

    fn build(self) -> Result<ZCache<T>> {
        let ZCacheBuilder {
            sub_builder,
            capacity,
            stamp: ExtractorStamp(extractor, _),
        } = self;

        build_cache_from_sub::<T, S, _>(
            sub_builder,
            capacity,
            move |_sample: &zenoh::sample::Sample, inner: &mut CacheInner<T>, msg: T| {
                let stamp = extractor(&msg);
                inner.insert(stamp, msg);
            },
        )
    }
}

// ---------------------------------------------------------------------------
// Shared build helper
// ---------------------------------------------------------------------------

/// Common construction path shared by both stamp variants.
///
/// `insert_fn` receives the raw `Sample`, the mutable `CacheInner`, and the
/// deserialized `T`. It is responsible for extracting the timestamp and calling
/// `inner.insert(stamp, msg)`.
fn build_cache_from_sub<T, S, InsertFn>(
    mut sub_builder: ZSubBuilder<T, S>,
    capacity: usize,
    insert_fn: InsertFn,
) -> Result<ZCache<T>>
where
    T: ZMessage + Send + Sync + 'static,
    S: for<'a> ZDeserializer<Input<'a> = &'a [u8], Output = T> + 'static,
    InsertFn: Fn(&zenoh::sample::Sample, &mut CacheInner<T>, T) + Send + Sync + 'static,
{
    // Qualify the topic name (same logic as ZSubBuilder::build_internal).
    let qualified_topic = topic_name::qualify_topic_name(
        &sub_builder.entity.topic,
        &sub_builder.entity.node.namespace,
        &sub_builder.entity.node.name,
    )
    .map_err(|e| zenoh::Error::from(format!("Failed to qualify topic: {}", e)))?;

    sub_builder.entity.topic = qualified_topic.clone();
    debug!("[CACHE] Qualified topic: {}", qualified_topic);

    let session = sub_builder.session.clone();
    let keyexpr_format = sub_builder.keyexpr_format;
    let entity = sub_builder.entity.clone();

    let topic_ke = keyexpr_format.topic_key_expr(&entity)?;
    let key_expr = (*topic_ke).clone();
    debug!("[CACHE] Key expression: {}", key_expr);

    let inner = Arc::new(Mutex::new(CacheInner::<T>::new(capacity)));
    let inner_cb = Arc::clone(&inner);

    let cb_sub = session
        .declare_subscriber(key_expr)
        .callback(move |sample: zenoh::sample::Sample| {
            let payload = sample.payload().to_bytes();
            match S::deserialize(&payload) {
                Ok(out) => {
                    let msg = out;
                    let mut guard = inner_cb.lock();
                    insert_fn(&sample, &mut guard, msg);
                }
                Err(e) => {
                    tracing::error!("[CACHE] Failed to deserialize message: {}", e);
                }
            }
        })
        .wait()?;

    let lv_ke = keyexpr_format.liveliness_key_expr(&entity, &session.zid())?;
    let lv_token = session
        .liveliness()
        .declare_token((*lv_ke).clone())
        .wait()?;

    debug!("[CACHE] Cache subscriber ready: topic={}", qualified_topic);

    Ok(ZCache {
        inner,
        _sub: cb_sub,
        _lv_token: lv_token,
    })
}
