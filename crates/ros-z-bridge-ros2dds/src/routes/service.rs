use std::{
    collections::HashMap,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
};

use anyhow::{Result, anyhow};
use cyclors::dds_entity_t;
use parking_lot::Mutex;
use zenoh::{
    Session, Wait,
    bytes::ZBytes,
    key_expr::KeyExpr,
    query::{Query, Queryable},
};

use crate::dds::{
    discovery::DiscoveredEndpoint,
    entity::DdsEntity,
    names::{dds_topic_to_ros2_name, dds_type_to_ros2_service_type, ros2_name_to_zenoh_key},
    participant::get_instance_handle,
    qos::{qos_mismatch_reason, service_default_qos},
    reader::create_blob_reader,
    types::DDSRawSample,
    writer::{create_blob_writer, write_cdr},
};

const CDR_HEADER_LE: [u8; 4] = [0, 1, 0, 0];

/// Sequence number embedded in every ROS 2 CDR service payload.
///
/// Layout: [4-byte CDR header] [8-byte client_guid] [8-byte seq_num LE] [payload]
fn extract_sequence_number(raw: &[u8]) -> Option<u64> {
    if raw.len() < 20 {
        return None;
    }
    Some(u64::from_le_bytes(raw[12..20].try_into().unwrap()))
}

/// A route that exposes a DDS service server as a Zenoh queryable.
///
/// Translates Zenoh `get()` queries into DDS requests and matches replies back
/// using the sequence number from the CddsRequestHeader.
///
/// The queryable is declared with `.complete(true)` so Zenoh clients across a
/// router topology see this bridge as a complete service provider (#642).
pub struct ServiceRoute {
    _req_writer: DdsEntity,
    _rep_reader: DdsEntity,
    _queryable: Queryable<()>,
}

impl ServiceRoute {
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

        tracing::info!(
            "Service route (DDS server → Zenoh queryable): {ros2_name} ↔ {}",
            ke
        );

        let seq_counter = Arc::new(AtomicU64::new(0));
        let in_flight: Arc<Mutex<HashMap<u64, Query>>> = Arc::new(Mutex::new(HashMap::new()));

        let req_topic = format!("rq{ros2_name}Request");
        let rep_topic = format!("rr{ros2_name}Reply");

        let dds_base = ros2_type.replace('/', "::");
        let req_type = format!("{dds_base}_Request_");
        let rep_type = format!("{dds_base}_Response_");

        let qos = service_default_qos();

        // G3: warn on QoS incompatibility between discovered endpoint and the service QoS.
        if let Some(reason) = qos_mismatch_reason(&endpoint.qos, &qos) {
            tracing::warn!("QoS mismatch on {}: {}", endpoint.topic_name, reason);
        }

        let req_writer_h = create_blob_writer(dp, &req_topic, &req_type, true, qos.clone())?;
        let req_writer = unsafe { DdsEntity::new(req_writer_h) };
        let req_writer_raw = req_writer_h;

        // Derive client_guid from the request writer's instance handle (#647).
        // CycloneDDS echoes this handle back as the client_guid in the reply header,
        // so replies are routed to this specific writer — not the participant at large.
        let client_guid = get_instance_handle(req_writer_h)?;

        let in_flight_rep = in_flight.clone();
        let rep_reader_h = create_blob_reader(
            dp,
            &rep_topic,
            &rep_type,
            true,
            qos.clone(),
            move |sample: DDSRawSample| {
                let raw = sample.as_slice();
                let seq = match extract_sequence_number(raw) {
                    Some(s) => s,
                    None => {
                        tracing::warn!("Service reply too short ({} bytes)", raw.len());
                        return;
                    }
                };
                if let Some(query) = in_flight_rep.lock().remove(&seq) {
                    // Strip the 16-byte CddsRequestHeader before forwarding to Zenoh (#647).
                    // The Zenoh querier sent [CDR_HDR + payload] and expects the same shape back,
                    // not the DDS-level [CDR_HDR + guid + seq + payload].
                    let zenoh_payload = if raw.len() >= 20 {
                        let mut v = Vec::with_capacity(4 + (raw.len() - 20));
                        v.extend_from_slice(&CDR_HEADER_LE);
                        v.extend_from_slice(&raw[20..]);
                        v
                    } else {
                        raw.to_vec()
                    };
                    let zbytes: ZBytes = zenoh_payload.into();
                    let ke = query.key_expr().clone();
                    if let Err(e) = query.reply(ke, zbytes).wait() {
                        tracing::warn!("Service reply send failed: {e}");
                    }
                } else {
                    tracing::debug!("No in-flight query for seq={seq}");
                }
            },
        )?;
        let rep_reader = unsafe { DdsEntity::new(rep_reader_h) };

        let in_flight_q = in_flight.clone();
        let seq_counter_q = seq_counter.clone();

        let queryable = session
            .declare_queryable(ke.clone())
            // complete(true): signals to Zenoh routers that this queryable handles the full
            // key space, enabling cross-router service calls to succeed (#642).
            .complete(true)
            .callback(move |query: Query| {
                let seq = seq_counter_q.fetch_add(1, Ordering::Relaxed);

                let query_payload: Vec<u8> = match query.payload() {
                    Some(p) => p.to_bytes().into_owned(),
                    None => vec![],
                };

                let payload_body = if query_payload.len() >= 4 {
                    &query_payload[4..]
                } else {
                    query_payload.as_slice()
                };

                // Build DDS payload: CDR_LE + client_guid (8 bytes LE) + seq (8 bytes LE) + body
                let mut dds_payload = Vec::with_capacity(4 + 16 + payload_body.len());
                dds_payload.extend_from_slice(&CDR_HEADER_LE);
                dds_payload.extend_from_slice(&client_guid.to_le_bytes());
                dds_payload.extend_from_slice(&seq.to_le_bytes());
                dds_payload.extend_from_slice(payload_body);

                in_flight_q.lock().insert(seq, query);

                if let Err(e) = write_cdr(req_writer_raw, dds_payload) {
                    tracing::warn!("DDS request write failed: {e}");
                    in_flight_q.lock().remove(&seq);
                }
            })
            .await
            .map_err(|e| anyhow!("declare_queryable failed: {e}"))?;

        Ok(Self {
            _req_writer: req_writer,
            _rep_reader: rep_reader,
            _queryable: queryable,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_extract_sequence_number_valid() {
        let mut raw = vec![0u8; 24];
        raw[12..20].copy_from_slice(&42u64.to_le_bytes());
        assert_eq!(extract_sequence_number(&raw), Some(42));
    }

    #[test]
    fn test_extract_sequence_number_too_short() {
        assert_eq!(extract_sequence_number(&[0u8; 19]), None);
        assert_eq!(extract_sequence_number(&[]), None);
    }

    #[test]
    fn test_extract_sequence_number_exact_boundary() {
        let raw = vec![0u8; 20];
        assert_eq!(extract_sequence_number(&raw), Some(0));
    }

    #[test]
    fn test_reply_header_stripping_removes_16_byte_dds_header() {
        // DDS reply: [4-byte CDR header] [8-byte guid] [8-byte seq] [4-byte payload]
        let mut dds_reply = vec![0u8; 24];
        dds_reply[0] = 0;
        dds_reply[1] = 1; // CDR LE
        dds_reply[4..12].copy_from_slice(&0xdeadbeef_u64.to_le_bytes());
        dds_reply[12..20].copy_from_slice(&7u64.to_le_bytes());
        dds_reply[20..24].copy_from_slice(&[1, 2, 3, 4]);

        let raw = dds_reply.as_slice();
        let zenoh_payload: Vec<u8> = if raw.len() >= 20 {
            let mut v = Vec::with_capacity(4 + (raw.len() - 20));
            v.extend_from_slice(&CDR_HEADER_LE);
            v.extend_from_slice(&raw[20..]);
            v
        } else {
            raw.to_vec()
        };

        assert_eq!(zenoh_payload.len(), 8);
        assert_eq!(&zenoh_payload[..4], &CDR_HEADER_LE);
        assert_eq!(&zenoh_payload[4..], &[1, 2, 3, 4]);
    }

    #[test]
    fn test_reply_header_stripping_short_payload_passthrough() {
        // Raw < 20 bytes → passthrough unchanged (defensive fallback)
        let raw = &[0u8; 15];
        let zenoh_payload: Vec<u8> = if raw.len() >= 20 {
            let mut v = Vec::with_capacity(4 + (raw.len() - 20));
            v.extend_from_slice(&CDR_HEADER_LE);
            v.extend_from_slice(&raw[20..]);
            v
        } else {
            raw.to_vec()
        };
        assert_eq!(zenoh_payload, raw.to_vec());
    }

    #[test]
    fn test_request_payload_construction() {
        let client_guid = 0xaabbccdd_u64;
        let seq = 5u64;
        let payload_body = &[10u8, 20, 30];

        let mut dds_payload = Vec::with_capacity(4 + 16 + payload_body.len());
        dds_payload.extend_from_slice(&CDR_HEADER_LE);
        dds_payload.extend_from_slice(&client_guid.to_le_bytes());
        dds_payload.extend_from_slice(&seq.to_le_bytes());
        dds_payload.extend_from_slice(payload_body);

        assert_eq!(&dds_payload[..4], &CDR_HEADER_LE);
        assert_eq!(&dds_payload[4..12], &client_guid.to_le_bytes());
        assert_eq!(&dds_payload[12..20], &seq.to_le_bytes());
        assert_eq!(&dds_payload[20..], payload_body);
    }
}
