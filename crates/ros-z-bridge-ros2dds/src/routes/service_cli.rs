use std::{sync::Arc, time::Duration};

use anyhow::{Result, anyhow};
use zenoh::{Session, bytes::ZBytes, key_expr::KeyExpr};

use crate::dds::{
    backend::{DdsParticipant, DdsWriter},
    discovery::DiscoveredEndpoint,
    names::{
        dds_topic_to_ros2_name, dds_type_to_ros2_service_type, is_action_get_result_topic,
        ros2_name_to_zenoh_key,
    },
    qos::{qos_mismatch_reason, service_default_bridge_qos},
};

const CDR_HEADER_LE: [u8; 4] = [0, 1, 0, 0];
const CDR_HEADER_BE: [u8; 4] = [0, 0, 0, 0];

fn cdr_header_matching(payload: &[u8]) -> [u8; 4] {
    if payload.get(1).copied().unwrap_or(1) == 1 {
        CDR_HEADER_LE
    } else {
        CDR_HEADER_BE
    }
}

const SERVICE_TIMEOUT_SECS: u64 = 10;
const ACTION_GET_RESULT_TIMEOUT_SECS: u64 = 300;

/// Request forwarded from the DDS reader callback to the async dispatch task.
struct PendingRequest {
    /// Original 16-byte request header from the DDS client (echoed back in the reply).
    hdr: [u8; 16],
    /// CDR payload without the request header (what the Zenoh server expects).
    payload: Vec<u8>,
}

/// A route that proxies a DDS service CLIENT through a Zenoh queryable (server).
pub struct ServiceCliRoute<P: DdsParticipant> {
    _req_reader: P::Reader,
    _rep_writer: Arc<P::Writer>,
}

impl<P: DdsParticipant> ServiceCliRoute<P> {
    pub async fn create(
        participant: &P,
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

        let qos = service_default_bridge_qos();

        if let Some(reason) = qos_mismatch_reason(&endpoint.qos, &qos) {
            tracing::warn!("QoS mismatch on {}: {}", endpoint.topic_name, reason);
        }

        let rep_writer =
            Arc::new(participant.create_writer(&rep_topic, &rep_type, true, qos.clone())?);

        let (tx, rx) = flume::bounded::<PendingRequest>(64);

        let querier = session
            .declare_querier(ke.clone())
            .timeout(timeout)
            .await
            .map_err(|e| anyhow!("declare_querier failed: {e}"))?;

        let writer_task = Arc::clone(&rep_writer);
        tokio::spawn(async move {
            while let Ok(req) = rx.recv_async().await {
                let zbytes: ZBytes = req.payload.into();
                let replies = match querier
                    .get()
                    .payload(zbytes)
                    .attachment(req.hdr.to_vec())
                    .await
                {
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

                    let reply_body = if reply_bytes.len() >= 4 {
                        &reply_bytes[4..]
                    } else {
                        reply_bytes.as_slice()
                    };
                    let mut dds_reply = Vec::with_capacity(4 + 16 + reply_body.len());
                    dds_reply.extend_from_slice(&cdr_header_matching(&reply_bytes));
                    dds_reply.extend_from_slice(&req.hdr);
                    dds_reply.extend_from_slice(reply_body);

                    if let Err(e) = writer_task.write_cdr(dds_reply) {
                        tracing::warn!("DDS reply write failed: {e}");
                    }
                    break;
                }
            }
        });

        let req_reader = participant.create_reader(
            &req_topic,
            &req_type,
            true,
            qos,
            Box::new(move |bytes: Vec<u8>| {
                let raw = bytes.as_slice();
                if raw.len() < 20 {
                    tracing::warn!("Service request too short ({} bytes)", raw.len());
                    return;
                }
                let mut hdr = [0u8; 16];
                hdr.copy_from_slice(&raw[4..20]);

                let mut payload = Vec::with_capacity(4 + raw.len() - 20);
                payload.extend_from_slice(&cdr_header_matching(raw));
                payload.extend_from_slice(&raw[20..]);

                let _ = tx.try_send(PendingRequest { hdr, payload });
            }),
        )?;

        Ok(Self {
            _req_reader: req_reader,
            _rep_writer: rep_writer,
        })
    }
}
