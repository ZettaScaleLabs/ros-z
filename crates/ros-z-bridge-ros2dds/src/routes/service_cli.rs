use std::time::Duration;

use anyhow::{Result, anyhow};
use cyclors::dds_entity_t;
use zenoh::{Session, bytes::ZBytes, key_expr::KeyExpr};

use crate::dds::{
    discovery::DiscoveredEndpoint,
    entity::DdsEntity,
    names::{
        dds_topic_to_ros2_name, dds_type_to_ros2_service_type, is_action_get_result_topic,
        ros2_name_to_zenoh_key,
    },
    qos::service_default_qos,
    reader::create_blob_reader,
    types::DDSRawSample,
    writer::{create_blob_writer, write_cdr},
};

const CDR_HEADER_LE: [u8; 4] = [0, 1, 0, 0];

/// Regular service calls must complete within this timeout.
const SERVICE_TIMEOUT_SECS: u64 = 10;

/// Action `get_result` calls block until the goal completes — allow up to 300 s.
/// This matches the zenoh-plugin-ros2dds DEFAULT_ACTION_GET_RESULT_TIMEOUT.
const ACTION_GET_RESULT_TIMEOUT_SECS: u64 = 300;

/// Request forwarded from the DDS reader callback to the async dispatch task.
struct PendingRequest {
    /// Original 16-byte request header from the DDS client (echoed back in the reply).
    hdr: [u8; 16],
    /// CDR payload without the request header (what the Zenoh server expects).
    payload: Vec<u8>,
    /// Raw DDS writer handle for sending the reply back to the DDS client.
    rep_writer: dds_entity_t,
}

/// A route that proxies a DDS service CLIENT through a Zenoh queryable (server).
///
/// When the bridge discovers a DDS publication on `rq/<name>Request` (a DDS client
/// is calling a service), this route:
/// 1. Reads each CDR request from DDS
/// 2. Forwards it as a Zenoh `get()` to the matching queryable
/// 3. Writes the CDR reply back on `rr/<name>Reply` to the DDS client
///
/// Action `get_result` requests use a 300 s timeout instead of 10 s because the
/// Zenoh server blocks until the goal finishes executing.
///
/// This is the reverse direction of `ServiceRoute` (which handles DDS servers).
pub struct ServiceCliRoute {
    _req_reader: DdsEntity,
    _rep_writer: DdsEntity,
}

impl ServiceCliRoute {
    pub async fn create(
        dp: dds_entity_t,
        endpoint: &DiscoveredEndpoint,
        session: &Session,
        namespace: Option<&str>,
    ) -> Result<Self> {
        let ros2_name = dds_topic_to_ros2_name(&endpoint.topic_name)
            .ok_or_else(|| anyhow!("not a bridgeable topic: {}", endpoint.topic_name))?;
        let ros2_type = dds_type_to_ros2_service_type(&endpoint.type_name);

        let zenoh_key = ros2_name_to_zenoh_key(&ros2_name, namespace);
        let ke: KeyExpr<'static> = zenoh_key
            .try_into()
            .map_err(|e| anyhow!("invalid key expr: {e}"))?;

        // Action get_result blocks for the full goal duration; use a much longer timeout.
        let timeout = if is_action_get_result_topic(&endpoint.topic_name) {
            tracing::debug!(
                "Action get_result topic detected ({}): using {}s timeout",
                endpoint.topic_name,
                ACTION_GET_RESULT_TIMEOUT_SECS
            );
            Duration::from_secs(ACTION_GET_RESULT_TIMEOUT_SECS)
        } else {
            Duration::from_secs(SERVICE_TIMEOUT_SECS)
        };

        tracing::info!("Service client route (DDS client → Zenoh querier): {ros2_name} ↔ {ke}");

        let dds_base = ros2_type.replace('/', "::");
        let req_type = format!("{dds_base}_Request_");
        let rep_type = format!("{dds_base}_Response_");

        let rep_topic = format!("rr{ros2_name}Reply");
        let req_topic = endpoint.topic_name.clone();

        let qos = service_default_qos();

        let rep_writer_h = create_blob_writer(dp, &rep_topic, &rep_type, true, qos.clone())?;
        let rep_writer = unsafe { DdsEntity::new(rep_writer_h) };
        let rep_writer_raw = rep_writer_h;

        // Channel: DDS callback → async task
        let (tx, rx) = flume::bounded::<PendingRequest>(64);

        // Async task: receive requests, do Zenoh get(), write DDS replies
        let querier = session
            .declare_querier(ke.clone())
            .timeout(timeout)
            .await
            .map_err(|e| anyhow!("declare_querier failed: {e}"))?;

        tokio::spawn(async move {
            while let Ok(req) = rx.recv_async().await {
                let zbytes: ZBytes = req.payload.into();
                let replies = match querier.get().payload(zbytes).await {
                    Ok(r) => r,
                    Err(e) => {
                        tracing::warn!("Zenoh get() failed: {e}");
                        continue;
                    }
                };

                for reply in replies {
                    let reply_bytes: Vec<u8> = match reply.result() {
                        Ok(sample) => sample.payload().to_bytes().into_owned(),
                        Err(e) => {
                            tracing::warn!("Service reply error: {e}");
                            continue;
                        }
                    };

                    // Build DDS reply: CDR_LE + original 16-byte request header + reply payload
                    let reply_body = if reply_bytes.len() >= 4 {
                        &reply_bytes[4..]
                    } else {
                        reply_bytes.as_slice()
                    };
                    let mut dds_reply = Vec::with_capacity(4 + 16 + reply_body.len());
                    dds_reply.extend_from_slice(&CDR_HEADER_LE);
                    dds_reply.extend_from_slice(&req.hdr);
                    dds_reply.extend_from_slice(reply_body);

                    if let Err(e) = write_cdr(req.rep_writer, dds_reply) {
                        tracing::warn!("DDS reply write failed: {e}");
                    }
                    break;
                }
            }
        });

        let req_reader_h = create_blob_reader(
            dp,
            &req_topic,
            &req_type,
            true,
            qos,
            move |sample: DDSRawSample| {
                let raw = sample.as_slice();
                // raw = 4-byte CDR header + 16-byte request header + payload
                if raw.len() < 20 {
                    tracing::warn!("Service request too short ({} bytes)", raw.len());
                    return;
                }
                let mut hdr = [0u8; 16];
                hdr.copy_from_slice(&raw[4..20]);

                // Zenoh payload: CDR_LE + body (without request header)
                let mut payload = Vec::with_capacity(4 + raw.len() - 20);
                payload.extend_from_slice(&CDR_HEADER_LE);
                payload.extend_from_slice(&raw[20..]);

                let _ = tx.try_send(PendingRequest {
                    hdr,
                    payload,
                    rep_writer: rep_writer_raw,
                });
            },
        )?;
        let req_reader = unsafe { DdsEntity::new(req_reader_h) };

        Ok(Self {
            _req_reader: req_reader,
            _rep_writer: rep_writer,
        })
    }
}
