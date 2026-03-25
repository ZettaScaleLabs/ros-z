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

    // Use Arc<str> so the closure captures a reference-counted pointer rather
    // than cloning the full String on every message.
    let src_ke_log: Arc<str> = src_ke.as_str().into();
    let dst_ke_log: Arc<str> = dst_ke.as_str().into();

    let sub = src_session
        .declare_subscriber(src_ke.clone())
        .callback({
            let publisher = publisher.clone();
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
    let target_ke_arc: Arc<str> = target_ke.as_str().into();
    let proxy_ke_log: Arc<str> = proxy_ke.as_str().into();

    let queryable = proxy_session
        .declare_queryable(proxy_ke.clone())
        .complete(true)
        .callback({
            let target_session = target_session.clone();
            move |query| {
                let target_session = target_session.clone();
                let target_ke = target_ke_arc.clone();
                let proxy_ke_log = proxy_ke_log.clone();
                // Extract payload and attachment from the query.
                // The attachment carries the QueryKey (sequence number, writer GUID)
                // required by ros-z service.rs — it must be forwarded verbatim.
                let payload: Option<ZBytes> = query.payload().cloned();
                let attachment: Option<ZBytes> = query.attachment().cloned();
                tokio::spawn(async move {
                    let mut get = target_session.get(target_ke.as_ref());
                    if let Some(p) = payload {
                        get = get.payload(p);
                    }
                    if let Some(a) = attachment {
                        get = get.attachment(a);
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
                    // Use the query's own KE (Jazzy KE) so the reply routes
                    // back correctly — Zenoh requires the reply KE to intersect
                    // the original queried KE, not the target KE.
                    let reply_ke = query.key_expr().to_string();
                    while let Ok(reply) = replies.recv_async().await {
                        match reply.result() {
                            Ok(sample) => {
                                let payload = sample.payload().clone();
                                if let Err(e) = query.reply(&reply_ke, payload).await {
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
