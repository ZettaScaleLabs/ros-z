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
    qos::service_default_qos,
    reader::create_blob_reader,
    types::DDSRawSample,
    writer::{create_blob_writer, write_cdr},
};

const CDR_HEADER_LE: [u8; 4] = [0, 1, 0, 0];

/// Sequence number embedded in every ROS 2 CDR service payload (offset 8..16).
fn extract_sequence_number(raw: &[u8]) -> Option<u64> {
    // raw = 4-byte CDR header + 16-byte request header + actual payload
    // Bytes [4..12] = client_guid, bytes [12..20] = sequence_number (LE)
    if raw.len() < 20 {
        return None;
    }
    Some(u64::from_le_bytes(raw[12..20].try_into().unwrap()))
}

/// A route that exposes a DDS service server as a Zenoh queryable.
///
/// Translates Zenoh `get()` queries into DDS requests and matches replies back
/// using the sequence number from the CddsRequestHeader (fix for #647).
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
        participant_client_guid: u64,
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

        // Convert "pkg/srv/Name" → "pkg::srv::dds_::Name"
        let dds_base = ros2_type.replace('/', "::");
        let req_type = format!("{dds_base}_Request_");
        let rep_type = format!("{dds_base}_Response_");

        let qos = service_default_qos();

        let req_writer_h = create_blob_writer(dp, &req_topic, &req_type, true, qos.clone())?;
        let req_writer = unsafe { DdsEntity::new(req_writer_h) };
        let req_writer_raw = req_writer_h;

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
                    let zbytes: ZBytes = raw.into();
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
            .callback(move |query: Query| {
                let seq = seq_counter_q.fetch_add(1, Ordering::Relaxed);

                let query_payload: Vec<u8> = match query.payload() {
                    Some(p) => p.to_bytes().into_owned(),
                    None => vec![],
                };

                // Strip CDR header from Zenoh payload if present; we'll re-add our own
                let payload_body = if query_payload.len() >= 4 {
                    &query_payload[4..]
                } else {
                    query_payload.as_slice()
                };

                // Build DDS payload: CDR_LE + client_guid (8 bytes LE) + seq (8 bytes LE) + body
                let mut dds_payload = Vec::with_capacity(4 + 16 + payload_body.len());
                dds_payload.extend_from_slice(&CDR_HEADER_LE);
                dds_payload.extend_from_slice(&participant_client_guid.to_le_bytes());
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
