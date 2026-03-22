//! Message forwarding between Humble and Jazzy Zenoh sessions.
//!
//! # Pub/Sub forwarding
//! Raw CDR bytes are forwarded verbatim between sessions with the key expression
//! hash segment rewritten for the target distro.
//!
//! # Service forwarding
//! Zenoh services use queryables (server) and get() (client). The bridge
//! declares a queryable on one side and re-issues queries on the other.

use anyhow::Result;
use std::sync::Arc;
use zenoh::{Session, Wait, bytes::ZBytes};

/// A running pub/sub forwarder for a single topic pair.
///
/// Dropping this handle stops the forwarder (subscriber is dropped).
pub struct ForwarderHandle {
    /// Keep the subscriber alive.
    _subscriber: zenoh::pubsub::Subscriber<()>,
}

/// Start forwarding messages from `src_session` key expression `src_ke`
/// to `dst_session` using key expression `dst_ke`.
///
/// The payload bytes are forwarded verbatim (CDR serialisation is identical
/// between Humble and Jazzy for the same message type).
pub fn start_forwarder(
    src_session: Arc<Session>,
    src_ke: String,
    dst_session: Arc<Session>,
    dst_ke: String,
) -> Result<ForwarderHandle> {
    let publisher = Arc::new(
        dst_session
            .declare_publisher(dst_ke.clone())
            .wait()
            .map_err(|e| anyhow::anyhow!("declare_publisher {dst_ke}: {e}"))?,
    );

    let sub = src_session
        .declare_subscriber(src_ke.clone())
        .callback({
            let publisher = publisher.clone();
            let src_ke_log = src_ke.clone();
            let dst_ke_log = dst_ke.clone();
            move |sample| {
                let payload: ZBytes = sample.payload().clone();
                let pub_clone = publisher.clone();
                let src = src_ke_log.clone();
                let dst = dst_ke_log.clone();
                tokio::spawn(async move {
                    if let Err(e) = pub_clone.put(payload).await {
                        tracing::warn!("forwarder {src} → {dst}: put failed: {e}");
                    }
                });
            }
        })
        .wait()
        .map_err(|e| anyhow::anyhow!("declare_subscriber {src_ke}: {e}"))?;

    Ok(ForwarderHandle { _subscriber: sub })
}

/// A running service forwarder.
///
/// Intercepts queries on `proxy_ke` and re-issues them on `target_ke` in the
/// other session, then forwards the reply back to the original querier.
///
/// Dropping this handle stops the forwarder.
pub struct ServiceForwarderHandle {
    _queryable: zenoh::query::Queryable<()>,
}

/// Start a service forwarder: declare a queryable on `proxy_session` at
/// `proxy_ke`, and for each query re-issue it via `target_session` at
/// `target_ke`, forwarding the first reply back.
pub fn start_service_forwarder(
    proxy_session: Arc<Session>,
    proxy_ke: String,
    target_session: Arc<Session>,
    target_ke: String,
) -> Result<ServiceForwarderHandle> {
    let queryable = proxy_session
        .declare_queryable(proxy_ke.clone())
        .complete(true)
        .callback({
            let target_session = target_session.clone();
            let target_ke = target_ke.clone();
            let proxy_ke_log = proxy_ke.clone();
            move |query| {
                let target_session = target_session.clone();
                let target_ke = target_ke.clone();
                let proxy_ke_log = proxy_ke_log.clone();
                // Extract payload from the query (service request body).
                let payload: Option<ZBytes> = query.payload().cloned();
                tokio::spawn(async move {
                    let mut get = target_session.get(&target_ke);
                    if let Some(p) = payload {
                        get = get.payload(p);
                    }
                    let replies = match get.await {
                        Ok(r) => r,
                        Err(e) => {
                            tracing::warn!(
                                "service forwarder {proxy_ke_log} → {target_ke}: get failed: {e}"
                            );
                            return;
                        }
                    };
                    // Forward the first reply back to the original querier.
                    while let Ok(reply) = replies.recv_async().await {
                        match reply.result() {
                            Ok(sample) => {
                                let payload = sample.payload().clone();
                                if let Err(e) = query.reply(&target_ke, payload).await {
                                    tracing::warn!(
                                        "service forwarder reply failed: {e}"
                                    );
                                }
                                break;
                            }
                            Err(e) => {
                                tracing::warn!(
                                    "service forwarder {proxy_ke_log} → {target_ke}: reply error: {e:?}"
                                );
                            }
                        }
                    }
                });
            }
        })
        .wait()
        .map_err(|e| anyhow::anyhow!("declare_queryable {proxy_ke}: {e}"))?;

    Ok(ServiceForwarderHandle {
        _queryable: queryable,
    })
}
