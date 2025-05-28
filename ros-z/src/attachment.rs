use zenoh::bytes::ZBytes;
use zenoh_ext::{ZDeserializer, ZSerializer};

const RMW_GID_STORAGE_SIZE: usize = 16;

pub type GidArray = [u8; RMW_GID_STORAGE_SIZE];

#[derive(Hash)]
pub struct Attachment {
    pub sequence_number: i64,
    pub source_timestamp: i64,
    pub source_gid: GidArray,
}

impl Attachment {
    pub fn new(sn: i64, gid: GidArray) -> Self {
        Self {
            sequence_number: sn,
            source_timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |v| v.as_nanos() as _),
            source_gid: gid,
        }
    }
}

impl TryFrom<&ZBytes> for Attachment {
    type Error = zenoh::Error;
    fn try_from(value: &ZBytes) -> Result<Self, Self::Error> {
        let mut des = ZDeserializer::new(&value);
        let sequence_number = des.deserialize::<i64>()?;
        let source_timestamp = des.deserialize::<i64>()?;
        let source_gid = des.deserialize::<GidArray>()?;
        Ok(Attachment {
            sequence_number,
            source_timestamp,
            source_gid,
        })
    }
}

impl From<Attachment> for ZBytes {
    fn from(value: Attachment) -> Self {
        let mut ser = ZSerializer::new();
        ser.serialize(value.sequence_number);
        ser.serialize(value.source_timestamp);
        ser.serialize(&value.source_gid);
        ser.finish()
    }
}
