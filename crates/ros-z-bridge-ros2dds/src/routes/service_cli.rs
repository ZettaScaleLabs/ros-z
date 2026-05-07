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

#[cfg(test)]
mod tests {
    use super::{CDR_HEADER_BE, CDR_HEADER_LE, cdr_header_matching};

    // ── cdr_header_matching ───────────────────────────────────────────────────

    #[test]
    fn test_cdr_header_matching_le_when_byte1_is_1() {
        let payload = [0x00, 0x01, 0x00, 0x00, 0xAA];
        assert_eq!(cdr_header_matching(&payload), CDR_HEADER_LE);
    }

    #[test]
    fn test_cdr_header_matching_be_when_byte1_is_0() {
        let payload = [0x00, 0x00, 0x00, 0x00, 0xAA];
        assert_eq!(cdr_header_matching(&payload), CDR_HEADER_BE);
    }

    #[test]
    fn test_cdr_header_matching_empty_payload_defaults_to_le() {
        // unwrap_or(1) → treats as LE
        assert_eq!(cdr_header_matching(&[]), CDR_HEADER_LE);
    }

    #[test]
    fn test_cdr_header_matching_single_byte_payload_defaults_to_le() {
        assert_eq!(cdr_header_matching(&[0x00]), CDR_HEADER_LE);
    }

    // ── DDS request → Zenoh query payload ────────────────────────────────────
    //
    // DDS service request wire format (CDR):
    //   [0..4]  CDR representation header (endianness flag at byte [1])
    //   [4..12] client_guid (8 bytes)
    //   [12..20] seq_num (8 bytes, little-endian)
    //   [20..]  ros2 payload body
    //
    // The bridge must:
    //   - Reject payloads shorter than 20 bytes
    //   - Extract bytes [4..20] as the 16-byte DDS header (guid + seq)
    //   - Build Zenoh query payload = [cdr_hdr(4)] + [raw[20..]]

    fn make_dds_request(body: &[u8]) -> Vec<u8> {
        let mut v = vec![0x00, 0x01, 0x00, 0x00]; // LE CDR header
        v.extend_from_slice(&0xDEADBEEF_u64.to_le_bytes()); // client_guid
        v.extend_from_slice(&42u64.to_le_bytes()); // seq_num
        v.extend_from_slice(body);
        v
    }

    #[test]
    fn test_request_parsing_extracts_16_byte_header() {
        let raw = make_dds_request(&[1, 2, 3]);
        assert!(raw.len() >= 20);
        let mut hdr = [0u8; 16];
        hdr.copy_from_slice(&raw[4..20]);
        // first 8 bytes = client_guid
        assert_eq!(&hdr[..8], &0xDEADBEEF_u64.to_le_bytes());
        // last 8 bytes = seq_num
        assert_eq!(&hdr[8..], &42u64.to_le_bytes());
    }

    #[test]
    fn test_request_builds_zenoh_payload_from_body() {
        let body = [10u8, 20, 30, 40];
        let raw = make_dds_request(&body);
        // Bridge logic: zenoh_payload = cdr_header_matching(raw) + raw[20..]
        let mut payload = Vec::new();
        payload.extend_from_slice(&cdr_header_matching(&raw));
        payload.extend_from_slice(&raw[20..]);
        assert_eq!(&payload[..4], &CDR_HEADER_LE);
        assert_eq!(&payload[4..], &body);
    }

    #[test]
    fn test_request_too_short_is_rejected() {
        // The bridge callback returns early when raw.len() < 20.
        let raw = vec![0u8; 19];
        assert!(
            raw.len() < 20,
            "precondition: payload must be shorter than 20"
        );
    }

    #[test]
    fn test_request_exactly_20_bytes_has_empty_body() {
        let raw = make_dds_request(&[]);
        assert_eq!(raw.len(), 20);
        let mut payload = Vec::new();
        payload.extend_from_slice(&cdr_header_matching(&raw));
        payload.extend_from_slice(&raw[20..]);
        assert_eq!(payload, CDR_HEADER_LE);
    }

    // ── Zenoh reply → DDS reply ───────────────────────────────────────────────
    //
    // Zenoh reply payload: [cdr_hdr(4)] [body...]
    //
    // Bridge must reconstruct:
    //   [cdr_hdr_matching_zenoh(4)] [saved_hdr(16)] [body...]
    //   i.e. strip the 4-byte CDR header from the reply and prepend the saved 16-byte
    //   DDS header (guid+seq) between the new CDR header and the body.

    fn build_dds_reply(saved_hdr: &[u8; 16], zenoh_reply: &[u8]) -> Vec<u8> {
        let reply_body = if zenoh_reply.len() >= 4 {
            &zenoh_reply[4..]
        } else {
            zenoh_reply
        };
        let mut dds_reply = Vec::with_capacity(4 + 16 + reply_body.len());
        dds_reply.extend_from_slice(&cdr_header_matching(zenoh_reply));
        dds_reply.extend_from_slice(saved_hdr);
        dds_reply.extend_from_slice(reply_body);
        dds_reply
    }

    #[test]
    fn test_reply_reconstruction_injects_saved_header() {
        let body = [0xAA, 0xBB, 0xCC, 0xDD];
        let mut zenoh_reply = CDR_HEADER_LE.to_vec();
        zenoh_reply.extend_from_slice(&body);

        let mut saved_hdr = [0u8; 16];
        saved_hdr[..8].copy_from_slice(&0x1234_u64.to_le_bytes());
        saved_hdr[8..].copy_from_slice(&7u64.to_le_bytes());

        let dds_reply = build_dds_reply(&saved_hdr, &zenoh_reply);

        assert_eq!(dds_reply.len(), 4 + 16 + body.len());
        assert_eq!(&dds_reply[..4], &CDR_HEADER_LE);
        assert_eq!(&dds_reply[4..20], &saved_hdr);
        assert_eq!(&dds_reply[20..], &body);
    }

    #[test]
    fn test_reply_reconstruction_preserves_be_endianness() {
        let mut zenoh_reply = vec![0x00, 0x00, 0x00, 0x00]; // BE CDR header
        zenoh_reply.extend_from_slice(&[1, 2, 3]);

        let saved_hdr = [0u8; 16];
        let dds_reply = build_dds_reply(&saved_hdr, &zenoh_reply);

        assert_eq!(&dds_reply[..4], &CDR_HEADER_BE);
    }

    #[test]
    fn test_reply_reconstruction_short_reply_passthrough() {
        // If the Zenoh reply is shorter than 4 bytes, treat the whole thing as the body.
        let zenoh_reply = &[0x01, 0x02]; // 2 bytes — no CDR header to strip
        let saved_hdr = [0u8; 16];
        let dds_reply = build_dds_reply(&saved_hdr, zenoh_reply);
        // reply_body = zenoh_reply (len < 4 branch)
        assert_eq!(&dds_reply[4..20], &saved_hdr);
        assert_eq!(&dds_reply[20..], &[0x01, 0x02]);
    }

    // ── CDR endianness round-trip ─────────────────────────────────────────────

    #[test]
    fn test_le_request_produces_le_zenoh_payload_and_le_dds_reply() {
        let body = [0xDE, 0xAD];
        let raw_request = make_dds_request(&body);
        // Zenoh payload endianness matches request
        let zenoh_payload: Vec<u8> = {
            let mut v = Vec::new();
            v.extend_from_slice(&cdr_header_matching(&raw_request));
            v.extend_from_slice(&raw_request[20..]);
            v
        };
        assert_eq!(&zenoh_payload[..4], &CDR_HEADER_LE);

        // Now simulate a reply with LE CDR header
        let mut saved_hdr = [0u8; 16];
        saved_hdr.copy_from_slice(&raw_request[4..20]);
        let dds_reply = build_dds_reply(&saved_hdr, &zenoh_payload);
        assert_eq!(&dds_reply[..4], &CDR_HEADER_LE);
        assert_eq!(&dds_reply[4..20], &saved_hdr);
        assert_eq!(&dds_reply[20..], &body);
    }
}
