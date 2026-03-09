use crate::type_support::MessageTypeSupport;
use ros_z::entity::{TypeHash, TypeInfo};
use ros_z::msg::{ZDeserializer, ZMessage, ZSerdes, ZSerializer, ZService};
use ros_z::ros_msg::WithTypeInfo;
use ros_z::{MessageTypeInfo, ServiceTypeInfo};

pub struct RosMessage {
    msg: *const crate::c_void,
    ts: MessageTypeSupport,
}

impl RosMessage {
    pub fn new(msg: *const crate::c_void, ts: MessageTypeSupport) -> Self {
        RosMessage { msg, ts }
    }
}

pub struct RosSerdes;

impl ZDeserializer for RosSerdes {
    type Input<'a> = (&'a Vec<u8>, &'a mut RosMessage);
    type Output = bool;
    type Error = std::io::Error;
    fn deserialize<'a>(input: Self::Input<'a>) -> Result<Self::Output, Self::Error> {
        let (bytes, msg) = input;
        unsafe { msg.ts.deserialize_message(bytes, msg.msg as *mut _) };
        Ok(true)
    }
}

impl ZSerializer for RosSerdes {
    type Input<'a> = &'a RosMessage;

    fn serialize_to_zbuf(input: Self::Input<'_>) -> zenoh_buffers::ZBuf {
        let RosMessage { msg, ts } = input;
        let bytes = unsafe { ts.serialize_message(*msg) };
        zenoh_buffers::ZBuf::from(bytes)
    }

    fn serialize_to_zbuf_with_hint(
        input: Self::Input<'_>,
        _capacity_hint: usize,
    ) -> zenoh_buffers::ZBuf {
        // RosMessage uses C FFI serialization which produces Vec<u8>
        // We can't pre-allocate based on hint, so just use the regular path
        Self::serialize_to_zbuf(input)
    }

    fn serialize_to_shm(
        input: Self::Input<'_>,
        _estimated_size: usize,
        provider: &zenoh::shm::ShmProvider<zenoh::shm::PosixShmProviderBackend>,
    ) -> zenoh::Result<(zenoh_buffers::ZBuf, usize)> {
        // RosMessage uses C FFI serialization, which produces Vec<u8>
        // So we serialize to Vec first, then copy to SHM
        let RosMessage { msg, ts } = input;
        let data = unsafe { ts.serialize_message(*msg) };
        let actual_size = data.len();

        use zenoh::Wait;
        use zenoh::shm::{BlockOn, GarbageCollect};

        let mut shm_buf = provider
            .alloc(actual_size)
            .with_policy::<BlockOn<GarbageCollect>>()
            .wait()
            .map_err(|e| zenoh::Error::from(format!("SHM allocation failed: {}", e)))?;

        shm_buf[0..actual_size].copy_from_slice(&data);

        Ok((zenoh_buffers::ZBuf::from(shm_buf), actual_size))
    }

    fn serialize_to_buf(input: Self::Input<'_>, buffer: &mut Vec<u8>) {
        let RosMessage { msg, ts } = input;
        buffer.clear();
        let bytes = unsafe { ts.serialize_message(*msg) };
        buffer.extend_from_slice(&bytes);
    }
}

/// `ZSerdes<RosMessage>` implementation for `RosSerdes`.
///
/// Serialization uses the C FFI path (`ts.serialize_message`).
/// Deserialization via the `ZSerdes` interface is not supported — the RMW path
/// deserializes directly into a pre-allocated C++ message via `ts.deserialize_message`
/// in `SubscriptionImpl::take`. Calling `deserialize` on this impl returns an error.
impl ZSerdes<RosMessage> for RosSerdes {
    type Error = std::io::Error;

    fn serialize(msg: &RosMessage) -> zenoh_buffers::ZBuf {
        let RosMessage { msg: ptr, ts } = msg;
        let bytes = unsafe { ts.serialize_message(*ptr) };
        zenoh_buffers::ZBuf::from(bytes)
    }

    fn serialize_with_hint(msg: &RosMessage, _capacity_hint: usize) -> zenoh_buffers::ZBuf {
        Self::serialize(msg)
    }

    fn serialize_to_vec(msg: &RosMessage) -> Vec<u8> {
        let RosMessage { msg: ptr, ts } = msg;
        unsafe { ts.serialize_message(*ptr) }
    }

    fn serialize_to_shm(
        msg: &RosMessage,
        _estimated_size: usize,
        provider: &zenoh::shm::ShmProvider<zenoh::shm::PosixShmProviderBackend>,
    ) -> zenoh::Result<(zenoh_buffers::ZBuf, usize)> {
        let data = Self::serialize_to_vec(msg);
        let actual_size = data.len();

        use zenoh::Wait;
        use zenoh::shm::{BlockOn, GarbageCollect};

        let mut shm_buf = provider
            .alloc(actual_size)
            .with_policy::<BlockOn<GarbageCollect>>()
            .wait()
            .map_err(|e| zenoh::Error::from(format!("SHM allocation failed: {}", e)))?;

        shm_buf[0..actual_size].copy_from_slice(&data);
        Ok((zenoh_buffers::ZBuf::from(shm_buf), actual_size))
    }

    fn deserialize(_buf: &[u8]) -> Result<RosMessage, std::io::Error> {
        Err(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "RosMessage deserialization requires a pre-allocated target; use SubscriptionImpl::take instead",
        ))
    }
}

impl ZMessage for RosMessage {}

impl MessageTypeInfo for RosMessage {
    // Static methods return placeholder values since RosMessage is a generic wrapper
    // The actual type info varies per instance
    fn type_name() -> &'static str {
        "rcl_z::RosMessage"
    }

    fn type_hash() -> TypeHash {
        // Placeholder - actual hash is instance-specific
        TypeHash::zero()
    }

    // Dynamic methods fetch real type info from C++ type support
    fn type_name_dyn(&self) -> String {
        self.ts.get_type_prefix()
    }

    fn type_hash_dyn(&self) -> TypeHash {
        self.ts.get_type_hash()
    }

    fn type_info_dyn(&self) -> TypeInfo {
        self.ts.get_type_info()
    }
}

impl WithTypeInfo for RosMessage {}

unsafe impl Sync for RosMessage {}
unsafe impl Send for RosMessage {}

pub struct RosService;

impl ZService for RosService {
    type Response = RosMessage;
    type Request = RosMessage;
}

impl ServiceTypeInfo for RosService {
    // Static method returns placeholder value since RosService is a generic wrapper
    fn service_type_info() -> TypeInfo {
        TypeInfo::new("rcl_z::RosService", TypeHash::zero())
    }

    // Note: Dynamic method would need to be implemented differently for services
    // since RosService doesn't hold type support. This may require refactoring
    // to store ServiceTypeSupport in RosService instances.
}
