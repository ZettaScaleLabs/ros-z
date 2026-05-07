use anyhow::{Result, anyhow};
use cyclors::dds_entity_t;
use zenoh::{Session, Wait, bytes::ZBytes, key_expr::KeyExpr, pubsub::Subscriber};

use crate::dds::{
    discovery::DiscoveredEndpoint,
    entity::DdsEntity,
    names::{dds_topic_to_ros2_name, ros2_name_to_zenoh_key},
    qos::{adapt_reader_qos_for_writer, adapt_writer_qos_for_reader},
    reader::create_blob_reader,
    types::DDSRawSample,
    writer::{create_blob_writer, write_cdr},
};

/// A route from a DDS publication to a Zenoh publisher.
///
/// Receives CDR bytes from DDS and forwards them verbatim to Zenoh.
pub struct DdsToZenohRoute {
    _reader: DdsEntity,
}

impl DdsToZenohRoute {
    pub async fn create(
        dp: dds_entity_t,
        endpoint: &DiscoveredEndpoint,
        session: &Session,
        namespace: Option<&str>,
    ) -> Result<Self> {
        let ros2_name = dds_topic_to_ros2_name(&endpoint.topic_name)
            .ok_or_else(|| anyhow!("not a bridgeable topic: {}", endpoint.topic_name))?;

        let zenoh_key = ros2_name_to_zenoh_key(&ros2_name, namespace);
        let ke: KeyExpr<'static> = zenoh_key
            .try_into()
            .map_err(|e| anyhow!("invalid key expr: {e}"))?;

        let publisher = session
            .declare_publisher(ke.clone())
            .await
            .map_err(|e| anyhow!("declare_publisher failed: {e}"))?;

        let qos = adapt_writer_qos_for_reader(&endpoint.qos);
        let topic_name = endpoint.topic_name.clone();
        let type_name = endpoint.type_name.clone();
        let keyless = endpoint.keyless;
        let ke_display = ke.to_string();

        tracing::info!("DDS→Zenoh pub/sub route: {topic_name} → {ke_display}");

        let reader_handle = create_blob_reader(
            dp,
            &topic_name,
            &type_name,
            keyless,
            qos,
            move |sample: DDSRawSample| {
                let bytes: ZBytes = sample.as_slice().into();
                if let Err(e) = publisher.put(bytes).wait() {
                    tracing::warn!("Zenoh put failed on {ke_display}: {e}");
                }
            },
        )?;

        Ok(Self {
            _reader: unsafe { DdsEntity::new(reader_handle) },
        })
    }
}

/// A route from a Zenoh subscriber to a DDS writer.
///
/// Receives CDR bytes from Zenoh and forwards them verbatim to DDS.
pub struct ZenohToDdsRoute {
    // Kept to extend RAII lifetime
    _writer: DdsEntity,
    _subscriber: Subscriber<()>,
}

impl ZenohToDdsRoute {
    pub async fn create(
        dp: dds_entity_t,
        endpoint: &DiscoveredEndpoint,
        session: &Session,
        namespace: Option<&str>,
    ) -> Result<Self> {
        let ros2_name = dds_topic_to_ros2_name(&endpoint.topic_name)
            .ok_or_else(|| anyhow!("not a bridgeable topic: {}", endpoint.topic_name))?;

        let zenoh_key = ros2_name_to_zenoh_key(&ros2_name, namespace);
        let ke: KeyExpr<'static> = zenoh_key
            .try_into()
            .map_err(|e| anyhow!("invalid key expr: {e}"))?;

        let qos = adapt_reader_qos_for_writer(&endpoint.qos);
        let writer_handle = create_blob_writer(
            dp,
            &endpoint.topic_name,
            &endpoint.type_name,
            endpoint.keyless,
            qos,
        )?;
        let writer_entity = unsafe { DdsEntity::new(writer_handle) };
        let writer_raw = writer_entity.raw();

        let ke_display = ke.to_string();
        let dds_topic = endpoint.topic_name.clone();
        tracing::info!("Zenoh→DDS pub/sub route: {ke_display} → {dds_topic}");

        let subscriber = session
            .declare_subscriber(ke.clone())
            .callback(move |sample| {
                let bytes: Vec<u8> = sample.payload().to_bytes().into_owned();
                if let Err(e) = write_cdr(writer_raw, bytes) {
                    tracing::warn!("DDS write failed on {ke_display}: {e}");
                }
            })
            .await
            .map_err(|e| anyhow!("declare_subscriber failed: {e}"))?;

        Ok(Self {
            _writer: writer_entity,
            _subscriber: subscriber,
        })
    }
}
