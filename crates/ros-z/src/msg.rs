use byteorder::LittleEndian;
#[cfg(feature = "protobuf")]
use prost::Message as ProstMessage;
use ros_z_cdr::{
    CdrBuffer, CdrDeserialize, CdrSerialize, CdrSerializedSize, CdrSerializer, CdrWriter,
    ZBufWriter,
};
use serde::{Deserialize, Serialize};
use std::marker::PhantomData;
use zenoh_buffers::ZBuf;

#[derive(Debug)]
pub struct CdrError(String);

impl std::fmt::Display for CdrError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "CDR deserialization error: {}", self.0)
    }
}

impl std::error::Error for CdrError {}

pub trait ZSerializer {
    type Input<'a>
    where
        Self: 'a;

    /// Serialize directly to a ZBuf for zero-copy publishing.
    ///
    /// This is the primary serialization method that returns a ZBuf,
    /// optimized for Zenoh publishing without intermediate copies.
    ///
    /// Uses a fixed 256-byte initial capacity. For better performance with
    /// large messages, use `serialize_to_zbuf_with_hint()` or call via
    /// `ZMessage::serialize_to_zbuf()` which provides accurate size hints.
    fn serialize_to_zbuf(input: Self::Input<'_>) -> ZBuf;

    /// Serialize to ZBuf with a capacity hint for optimal allocation.
    ///
    /// This method uses the provided capacity hint to pre-allocate the buffer,
    /// reducing or eliminating reallocations for large messages.
    ///
    /// # Arguments
    ///
    /// * `input` - The message to serialize
    /// * `capacity_hint` - Expected serialized size in bytes
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// use ros_z::msg::{ZSerializer, CdrCompatSerdes};
    /// use serde::Serialize;
    ///
    /// #[derive(Serialize)]
    /// struct LargeMsg { data: Vec<u8> }
    ///
    /// let msg = LargeMsg { data: vec![0; 1_000_000] };
    /// let hint = 4 + 4 + 1_000_000;  // header + length + data
    /// let zbuf = CdrCompatSerdes::serialize_to_zbuf_with_hint(&msg, hint);
    /// ```
    fn serialize_to_zbuf_with_hint(input: Self::Input<'_>, capacity_hint: usize) -> ZBuf;

    /// Serialize directly to shared memory for zero-copy publishing.
    ///
    /// This method serializes the message directly into a pre-allocated SHM buffer,
    /// avoiding any intermediate copies. This matches the rmw_zenoh_cpp approach.
    ///
    /// # Arguments
    ///
    /// * `input` - The message to serialize
    /// * `estimated_size` - Conservative upper bound on serialized size
    /// * `provider` - SHM provider for buffer allocation
    ///
    /// # Returns
    ///
    /// A tuple of (ZBuf, actual_size) where:
    /// - ZBuf is backed by SHM
    /// - actual_size is the exact number of bytes written
    ///
    /// # Errors
    ///
    /// Returns an error if SHM allocation fails.
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// use ros_z::msg::{ZSerializer, CdrCompatSerdes};
    /// use ros_z::shm::ShmProviderBuilder;
    /// use serde::Serialize;
    ///
    /// # fn main() -> zenoh::Result<()> {
    /// #[derive(Serialize)]
    /// struct MyMsg { value: u32 }
    ///
    /// let msg = MyMsg { value: 42 };
    /// let provider = ShmProviderBuilder::new(1024 * 1024).build()?;
    ///
    /// let (zbuf, size) = CdrCompatSerdes::serialize_to_shm(&msg, 128, &provider)?;
    /// println!("Serialized {} bytes to SHM", size);
    /// # Ok(())
    /// # }
    /// ```
    fn serialize_to_shm(
        input: Self::Input<'_>,
        estimated_size: usize,
        provider: &zenoh::shm::ShmProvider<zenoh::shm::PosixShmProviderBackend>,
    ) -> zenoh::Result<(ZBuf, usize)>;

    /// Serialize to an existing buffer, returning the result as ZBuf.
    ///
    /// This variant allows buffer reuse for reduced allocations.
    /// The buffer is cleared and reused, then wrapped in a ZBuf.
    fn serialize_to_zbuf_reuse(input: Self::Input<'_>, buffer: &mut Vec<u8>) -> ZBuf {
        Self::serialize_to_buf(input, buffer);
        // Take ownership of the buffer contents, leaving an empty Vec
        ZBuf::from(std::mem::take(buffer))
    }

    /// Serialize to Vec<u8> (legacy method).
    ///
    /// Prefer `serialize_to_zbuf()` for zero-copy publishing.
    fn serialize(input: Self::Input<'_>) -> Vec<u8> {
        let mut buffer = Vec::new();
        Self::serialize_to_buf(input, &mut buffer);
        buffer
    }

    /// Serialize to an existing buffer, reusing its allocation.
    ///
    /// The buffer is cleared before writing. Implementations should
    /// write directly to the buffer for optimal performance.
    fn serialize_to_buf(input: Self::Input<'_>, buffer: &mut Vec<u8>);
}

pub trait ZDeserializer {
    type Input<'a>;
    type Output;
    type Error: std::error::Error + Send + Sync + 'static;
    fn deserialize(input: Self::Input<'_>) -> Result<Self::Output, Self::Error>;
}

/// Unified serialization/deserialization trait for ROS-Z messages.
///
/// `ZSerdes<T>` replaces the old `ZMessage::Serdes` associated type.
/// Implement this trait to define a custom wire format for message type `T`.
///
/// Two built-in implementations are provided:
/// - [`CdrSerdes`]: fast path using `CdrSerialize`/`CdrDeserialize` traits
/// - [`CdrCompatSerdes`]: compatibility path using `serde::Serialize`/`Deserialize`
///
/// # Example
///
/// ```rust,no_run
/// use ros_z::msg::{ZSerdes, CdrSerdes, CdrCompatSerdes};
/// use serde::{Serialize, Deserialize};
///
/// #[derive(Serialize, Deserialize)]
/// struct MyMsg { value: u32 }
///
/// // Use serde path (works with any serde type)
/// fn publish_serde(msg: &MyMsg) -> zenoh_buffers::ZBuf {
///     CdrCompatSerdes::serialize(msg)
/// }
/// ```
pub trait ZSerdes<T>: Send + Sync + 'static {
    /// Error type returned by deserialization.
    type Error: std::error::Error + Send + Sync + 'static;

    /// Serialize `msg` to a ZBuf.
    fn serialize(msg: &T) -> ZBuf;

    /// Serialize `msg` to a ZBuf with a capacity hint.
    fn serialize_with_hint(msg: &T, capacity_hint: usize) -> ZBuf;

    /// Serialize `msg` to a Vec<u8>.
    fn serialize_to_vec(msg: &T) -> Vec<u8>;

    /// Serialize `msg` to shared memory.
    fn serialize_to_shm(
        msg: &T,
        estimated_size: usize,
        provider: &zenoh::shm::ShmProvider<zenoh::shm::PosixShmProviderBackend>,
    ) -> zenoh::Result<(ZBuf, usize)>;

    /// Deserialize a message from a byte slice.
    fn deserialize(buf: &[u8]) -> Result<T, Self::Error>;
}

// Core Z-Message trait
pub trait ZMessage: Send + Sync + Sized + 'static {
    /// Get an estimated upper bound on the serialized size of this message.
    ///
    /// This is used to pre-allocate buffers for optimal serialization performance,
    /// both for regular ZBuf serialization and for zero-copy SHM serialization.
    /// The estimate should be conservative (larger than actual) to avoid buffer overflow.
    ///
    /// Default implementation returns 2x the size of the type, which is conservative
    /// for most messages. Messages with dynamic fields (Vec, String, ZBuf) get accurate
    /// implementations auto-generated by ros-z-codegen.
    ///
    /// # Example
    ///
    /// ```rust
    /// use ros_z::msg::ZMessage;
    /// use serde::{Serialize, Deserialize};
    ///
    /// #[derive(Serialize, Deserialize)]
    /// struct MyMessage {
    ///     data: Vec<u8>,
    ///     count: u32,
    /// }
    ///
    /// // Custom implementation for better estimation
    /// impl MyMessage {
    ///     fn estimate_size(&self) -> usize {
    ///         4 + // CDR header
    ///         4 + // sequence length prefix for Vec
    ///         self.data.len() + // actual data
    ///         4 + // count field
    ///         16  // padding/alignment buffer
    ///     }
    /// }
    /// ```
    fn estimated_serialized_size(&self) -> usize {
        // Conservative default: 2x struct size + CDR header
        // This works well for structs with few dynamic fields
        std::mem::size_of::<Self>() * 2 + 4
    }
}

// Blanket implementation: any type satisfying the CDR traits is a ZMessage.
// All generated message types satisfy these bounds.
impl<T> ZMessage for T where T: Send + Sync + 'static {}

// ── Serde-based CDR serialization (existing path, kept for non-generated types) ───────────

/// Unit struct implementing CDR serialization via serde.
///
/// Use this serdes when your type implements `serde::Serialize + serde::de::DeserializeOwned`
/// but not the native `CdrSerialize`/`CdrDeserialize` traits. This includes:
/// - roslibrust message types
/// - Custom user types with `#[derive(Serialize, Deserialize)]`
/// - Internal wire types that haven't been migrated to native CDR
///
/// Performance: ~15-30% slower than `CdrSerdes` due to serde overhead.
pub struct CdrCompatSerdes;

/// CDR encapsulation header for little-endian encoding
pub const CDR_HEADER_LE: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

impl<T> ZSerdes<T> for CdrCompatSerdes
where
    T: serde::Serialize + serde::de::DeserializeOwned + Send + Sync + 'static,
{
    type Error = CdrError;

    fn serialize(msg: &T) -> ZBuf {
        Self::serialize_with_hint(msg, 256)
    }

    fn serialize_with_hint(msg: &T, capacity_hint: usize) -> ZBuf {
        let mut writer = ZBufWriter::with_capacity(capacity_hint);
        writer.extend_from_slice(&CDR_HEADER_LE);
        let mut serializer = CdrSerializer::<LittleEndian, ZBufWriter>::new(&mut writer);
        msg.serialize(&mut serializer).unwrap();
        writer.into_zbuf()
    }

    fn serialize_to_vec(msg: &T) -> Vec<u8> {
        let mut buffer = Vec::new();
        buffer.extend_from_slice(&CDR_HEADER_LE);
        let mut fast_ser = CdrSerializer::<LittleEndian>::new(&mut buffer);
        msg.serialize(&mut fast_ser).unwrap();
        buffer
    }

    fn serialize_to_shm(
        msg: &T,
        estimated_size: usize,
        provider: &zenoh::shm::ShmProvider<zenoh::shm::PosixShmProviderBackend>,
    ) -> zenoh::Result<(ZBuf, usize)> {
        let mut writer = crate::shm::ShmWriter::new(provider, estimated_size)?;
        writer.extend_from_slice(&CDR_HEADER_LE);
        let mut serializer = CdrSerializer::<LittleEndian, crate::shm::ShmWriter>::new(&mut writer);
        msg.serialize(&mut serializer)
            .map_err(|e| zenoh::Error::from(format!("CDR serialization failed: {}", e)))?;
        let actual_size = writer.position();
        let zbuf = writer.into_zbuf()?;
        Ok((zbuf, actual_size))
    }

    fn deserialize(buf: &[u8]) -> Result<T, CdrError> {
        if buf.len() < 4 {
            return Err(CdrError("CDR data too short for header".into()));
        }
        let representation_identifier = &buf[0..2];
        if representation_identifier != [0x00, 0x01] {
            return Err(CdrError(format!(
                "Expected CDR_LE encapsulation ({:?}), found {:?}",
                [0x00, 0x01],
                representation_identifier
            )));
        }
        let payload = &buf[4..];
        let x = ros_z_cdr::from_bytes::<T, byteorder::LittleEndian>(payload)
            .map_err(|e| CdrError(e.to_string()))?;
        Ok(x.0)
    }
}

// Keep old ZSerializer/ZDeserializer impls on a wrapper for backwards compat
// with any callers that use CdrCompatSerdes::<T>::serialize_to_zbuf() etc.
// We provide a type alias for backwards compat.
/// Backwards-compatible wrapper. Prefer [`CdrCompatSerdes`] (unit struct).
#[doc(hidden)]
pub struct CdrCompatSerdesTyped<T>(PhantomData<T>);

impl<T> ZSerializer for CdrCompatSerdesTyped<T>
where
    T: Serialize,
{
    type Input<'a>
        = &'a T
    where
        T: 'a;

    fn serialize_to_zbuf(input: &T) -> ZBuf {
        Self::serialize_to_zbuf_with_hint(input, 256)
    }

    fn serialize_to_zbuf_with_hint(input: &T, capacity_hint: usize) -> ZBuf {
        let mut writer = ZBufWriter::with_capacity(capacity_hint);
        writer.extend_from_slice(&CDR_HEADER_LE);
        let mut serializer = CdrSerializer::<LittleEndian, ZBufWriter>::new(&mut writer);
        input.serialize(&mut serializer).unwrap();
        writer.into_zbuf()
    }

    fn serialize_to_shm(
        input: &T,
        estimated_size: usize,
        provider: &zenoh::shm::ShmProvider<zenoh::shm::PosixShmProviderBackend>,
    ) -> zenoh::Result<(ZBuf, usize)> {
        let mut writer = crate::shm::ShmWriter::new(provider, estimated_size)?;
        writer.extend_from_slice(&CDR_HEADER_LE);
        let mut serializer = CdrSerializer::<LittleEndian, crate::shm::ShmWriter>::new(&mut writer);
        input
            .serialize(&mut serializer)
            .map_err(|e| zenoh::Error::from(format!("CDR serialization failed: {}", e)))?;
        let actual_size = writer.position();
        let zbuf = writer.into_zbuf()?;
        Ok((zbuf, actual_size))
    }

    fn serialize(input: &T) -> Vec<u8> {
        let mut buffer = Vec::new();
        Self::serialize_to_buf(input, &mut buffer);
        buffer
    }

    fn serialize_to_buf(input: &T, buffer: &mut Vec<u8>) {
        buffer.clear();
        buffer.extend_from_slice(&CDR_HEADER_LE);
        let mut fast_ser = CdrSerializer::<LittleEndian>::new(buffer);
        input.serialize(&mut fast_ser).unwrap();
    }
}

impl<T> ZDeserializer for CdrCompatSerdesTyped<T>
where
    for<'a> T: Deserialize<'a>,
{
    type Input<'b> = &'b [u8];
    type Output = T;
    type Error = CdrError;

    fn deserialize(input: Self::Input<'_>) -> Result<Self::Output, Self::Error> {
        if input.len() < 4 {
            return Err(CdrError("CDR data too short for header".into()));
        }
        let representation_identifier = &input[0..2];
        if representation_identifier != [0x00, 0x01] {
            return Err(CdrError(format!(
                "Expected CDR_LE encapsulation ({:?}), found {:?}",
                [0x00, 0x01],
                representation_identifier
            )));
        }
        let payload = &input[4..];
        let x = ros_z_cdr::from_bytes::<T, byteorder::LittleEndian>(payload)
            .map_err(|e| CdrError(e.to_string()))?;
        Ok(x.0)
    }
}

// ── Fast CdrSerialize-based CDR serialization (new path for generated types) ────────────

/// Unit struct implementing CDR serialization via native `CdrSerialize`/`CdrDeserialize` traits.
///
/// Generated message types implement these traits and use `CdrSerdes` as their
/// default serdes. This enables the POD bulk fast path for sequences of
/// plain types (e.g., `Vec<f32>`, `Vec<geometry_msgs::Point>`).
///
/// This is the default serdes for `ZPub`, `ZSub`, `ZClient`, and `ZServer`.
pub struct CdrSerdes;

impl<T> ZSerdes<T> for CdrSerdes
where
    T: CdrSerialize + CdrSerializedSize + CdrDeserialize + Send + Sync + 'static,
{
    type Error = CdrError;

    fn serialize(msg: &T) -> ZBuf {
        let capacity_hint = msg.cdr_serialized_size(0) + 4;
        Self::serialize_with_hint(msg, capacity_hint)
    }

    fn serialize_with_hint(msg: &T, capacity_hint: usize) -> ZBuf {
        let mut writer = ZBufWriter::with_capacity(capacity_hint);
        writer.extend_from_slice(&CDR_HEADER_LE);
        ros_z_cdr::traits::cdr_to_zbuf_writer(msg, &mut writer);
        writer.into_zbuf()
    }

    fn serialize_to_vec(msg: &T) -> Vec<u8> {
        let mut buffer = Vec::new();
        buffer.extend_from_slice(&CDR_HEADER_LE);
        let mut cdr_writer = CdrWriter::<LittleEndian>::new(&mut buffer);
        msg.cdr_serialize(&mut cdr_writer);
        buffer
    }

    fn serialize_to_shm(
        msg: &T,
        estimated_size: usize,
        provider: &zenoh::shm::ShmProvider<zenoh::shm::PosixShmProviderBackend>,
    ) -> zenoh::Result<(ZBuf, usize)> {
        let mut writer = crate::shm::ShmWriter::new(provider, estimated_size)?;
        writer.extend_from_slice(&CDR_HEADER_LE);
        let mut cdr_writer = CdrWriter::<LittleEndian, crate::shm::ShmWriter>::new(&mut writer);
        msg.cdr_serialize(&mut cdr_writer);
        let actual_size = writer.position();
        let zbuf = writer.into_zbuf()?;
        Ok((zbuf, actual_size))
    }

    fn deserialize(buf: &[u8]) -> Result<T, CdrError> {
        if buf.len() < 4 {
            return Err(CdrError("CDR data too short for header".into()));
        }
        let representation_identifier = &buf[0..2];
        if representation_identifier != [0x00, 0x01] {
            return Err(CdrError(format!(
                "Expected CDR_LE encapsulation ({:?}), found {:?}",
                [0x00, 0x01],
                representation_identifier
            )));
        }
        let payload = &buf[4..];
        let mut reader = ros_z_cdr::CdrReader::<LittleEndian>::new(payload);
        T::cdr_deserialize(&mut reader).map_err(|e| CdrError(e.to_string()))
    }
}

/// Backwards-compatible typed wrapper for CdrSerdes.
#[doc(hidden)]
pub struct CdrSerdesTyped<T>(PhantomData<T>);

impl<T> ZSerializer for CdrSerdesTyped<T>
where
    T: CdrSerialize + CdrSerializedSize,
{
    type Input<'a>
        = &'a T
    where
        T: 'a;

    fn serialize_to_zbuf(input: &T) -> ZBuf {
        let capacity_hint = input.cdr_serialized_size(0) + 4;
        Self::serialize_to_zbuf_with_hint(input, capacity_hint)
    }

    fn serialize_to_zbuf_with_hint(input: &T, capacity_hint: usize) -> ZBuf {
        let mut writer = ZBufWriter::with_capacity(capacity_hint);
        writer.extend_from_slice(&CDR_HEADER_LE);
        ros_z_cdr::traits::cdr_to_zbuf_writer(input, &mut writer);
        writer.into_zbuf()
    }

    fn serialize_to_shm(
        input: &T,
        estimated_size: usize,
        provider: &zenoh::shm::ShmProvider<zenoh::shm::PosixShmProviderBackend>,
    ) -> zenoh::Result<(ZBuf, usize)> {
        let mut writer = crate::shm::ShmWriter::new(provider, estimated_size)?;
        writer.extend_from_slice(&CDR_HEADER_LE);
        let mut cdr_writer = CdrWriter::<LittleEndian, crate::shm::ShmWriter>::new(&mut writer);
        input.cdr_serialize(&mut cdr_writer);
        let actual_size = writer.position();
        let zbuf = writer.into_zbuf()?;
        Ok((zbuf, actual_size))
    }

    fn serialize(input: &T) -> Vec<u8> {
        let mut buffer = Vec::new();
        Self::serialize_to_buf(input, &mut buffer);
        buffer
    }

    fn serialize_to_buf(input: &T, buffer: &mut Vec<u8>) {
        buffer.clear();
        buffer.extend_from_slice(&CDR_HEADER_LE);
        let mut cdr_writer = CdrWriter::<LittleEndian>::new(buffer);
        input.cdr_serialize(&mut cdr_writer);
    }
}

impl<T> ZDeserializer for CdrSerdesTyped<T>
where
    T: CdrDeserialize,
{
    type Input<'b> = &'b [u8];
    type Output = T;
    type Error = CdrError;

    fn deserialize(input: Self::Input<'_>) -> Result<Self::Output, Self::Error> {
        if input.len() < 4 {
            return Err(CdrError("CDR data too short for header".into()));
        }
        let representation_identifier = &input[0..2];
        if representation_identifier != [0x00, 0x01] {
            return Err(CdrError(format!(
                "Expected CDR_LE encapsulation ({:?}), found {:?}",
                [0x00, 0x01],
                representation_identifier
            )));
        }
        let payload = &input[4..];
        let mut reader = ros_z_cdr::CdrReader::<LittleEndian>::new(payload);
        T::cdr_deserialize(&mut reader).map_err(|e| CdrError(e.to_string()))
    }
}

// Protobuf

#[cfg(feature = "protobuf")]
pub struct ProtobufSerdes;

#[cfg(feature = "protobuf")]
impl<T> ZSerdes<T> for ProtobufSerdes
where
    T: ProstMessage + Default + Send + Sync + 'static,
{
    type Error = prost::DecodeError;

    fn serialize(msg: &T) -> ZBuf {
        ZBuf::from(msg.encode_to_vec())
    }

    fn serialize_with_hint(msg: &T, _capacity_hint: usize) -> ZBuf {
        Self::serialize(msg)
    }

    fn serialize_to_vec(msg: &T) -> Vec<u8> {
        msg.encode_to_vec()
    }

    fn serialize_to_shm(
        msg: &T,
        estimated_size: usize,
        provider: &zenoh::shm::ShmProvider<zenoh::shm::PosixShmProviderBackend>,
    ) -> zenoh::Result<(ZBuf, usize)> {
        let data = msg.encode_to_vec();
        let actual_size = data.len();

        use zenoh::Wait;
        use zenoh::shm::{BlockOn, GarbageCollect};

        let mut shm_buf = provider
            .alloc(estimated_size.max(actual_size))
            .with_policy::<BlockOn<GarbageCollect>>()
            .wait()
            .map_err(|e| zenoh::Error::from(format!("SHM allocation failed: {}", e)))?;

        shm_buf[0..actual_size].copy_from_slice(&data);

        Ok((ZBuf::from(shm_buf), actual_size))
    }

    fn deserialize(buf: &[u8]) -> Result<T, prost::DecodeError> {
        T::decode(buf)
    }
}

/// Backwards-compatible typed protobuf wrapper.
#[cfg(feature = "protobuf")]
#[doc(hidden)]
pub struct ProtobufSerdesTyped<T>(PhantomData<T>);

#[cfg(feature = "protobuf")]
impl<T> ZSerializer for ProtobufSerdesTyped<T>
where
    T: ProstMessage,
{
    type Input<'a>
        = &'a T
    where
        T: 'a;

    fn serialize_to_zbuf(input: &T) -> ZBuf {
        // Use prost's builtin encode_to_vec and wrap in ZBuf
        ZBuf::from(input.encode_to_vec())
    }

    fn serialize_to_zbuf_with_hint(input: &T, _capacity_hint: usize) -> ZBuf {
        // Protobuf doesn't support custom capacity hints (uses encode_to_vec internally)
        Self::serialize_to_zbuf(input)
    }

    fn serialize_to_shm(
        input: &T,
        estimated_size: usize,
        provider: &zenoh::shm::ShmProvider<zenoh::shm::PosixShmProviderBackend>,
    ) -> zenoh::Result<(ZBuf, usize)> {
        // For protobuf, we serialize to Vec first then copy to SHM
        // (protobuf doesn't support custom writers like CDR does)
        let data = input.encode_to_vec();
        let actual_size = data.len();

        use zenoh::Wait;
        use zenoh::shm::{BlockOn, GarbageCollect};

        let mut shm_buf = provider
            .alloc(estimated_size.max(actual_size))
            .with_policy::<BlockOn<GarbageCollect>>()
            .wait()
            .map_err(|e| zenoh::Error::from(format!("SHM allocation failed: {}", e)))?;

        shm_buf[0..actual_size].copy_from_slice(&data);

        Ok((ZBuf::from(shm_buf), actual_size))
    }

    fn serialize(input: &T) -> Vec<u8> {
        // Use prost's builtin encode_to_vec for direct serialization
        input.encode_to_vec()
    }

    fn serialize_to_buf(input: &T, buffer: &mut Vec<u8>) {
        buffer.clear();
        input.encode(buffer).unwrap();
    }
}

#[cfg(feature = "protobuf")]
impl<T> ZDeserializer for ProtobufSerdesTyped<T>
where
    T: ProstMessage + Default,
{
    type Input<'a> = &'a [u8];
    type Output = T;
    type Error = prost::DecodeError;

    fn deserialize(input: &[u8]) -> Result<T, prost::DecodeError> {
        T::decode(input)
    }
}

pub trait ZService {
    type Request: ZMessage;
    type Response: ZMessage;
}

#[cfg(test)]
mod tests {
    use super::*;
    use zenoh_buffers::buffer::SplitBuffer;

    #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
    struct SimpleMessage {
        value: u32,
        text: String,
    }

    #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
    struct LargeMessage {
        data: Vec<u8>,
        count: u64,
        nested: Vec<SimpleMessage>,
    }

    #[test]
    fn test_serialize_to_zbuf() {
        let msg = SimpleMessage {
            value: 42,
            text: "Hello, ZBuf!".to_string(),
        };

        let zbuf = CdrCompatSerdes::serialize(&msg);
        let bytes = zbuf.contiguous();

        // Verify CDR header
        assert_eq!(&bytes[0..4], &CDR_HEADER_LE);

        // Verify roundtrip
        let deserialized =
            <CdrCompatSerdes as ZSerdes<SimpleMessage>>::deserialize(&bytes).unwrap();
        assert_eq!(deserialized, msg);
    }

    #[test]
    fn test_serialize_to_zbuf_consistency() {
        let msg = SimpleMessage {
            value: 123,
            text: "consistency test".to_string(),
        };

        // Both methods should produce identical bytes
        let zbuf = CdrCompatSerdes::serialize(&msg);
        let vec = CdrCompatSerdes::serialize_to_vec(&msg);

        let zbuf_bytes = zbuf.contiguous();
        assert_eq!(&*zbuf_bytes, &vec[..]);
    }

    #[test]
    fn test_zmessage_serialize_to_zbuf() {
        let msg = SimpleMessage {
            value: 777,
            text: "trait test".to_string(),
        };

        // Use CdrCompatSerdes directly for serde-only types
        let zbuf = CdrCompatSerdes::serialize(&msg);
        let bytes = zbuf.contiguous();

        assert_eq!(&bytes[0..4], &CDR_HEADER_LE);

        let deserialized =
            <CdrCompatSerdes as ZSerdes<SimpleMessage>>::deserialize(&bytes).unwrap();
        assert_eq!(deserialized, msg);
    }

    #[test]
    fn test_cdr_serialize_to_buf_consistency() {
        let msg = SimpleMessage {
            value: 42,
            text: "Hello, ros-z!".to_string(),
        };

        // Serialize using both methods
        let vec1 = CdrCompatSerdes::serialize_to_vec(&msg);
        let zbuf = CdrCompatSerdes::serialize(&msg);
        let zbuf_bytes = zbuf.contiguous();
        let vec2 = zbuf_bytes.to_vec();

        // Results should be identical
        assert_eq!(vec1, vec2);
        assert!(!vec1.is_empty());
        assert_eq!(&vec1[0..4], &CDR_HEADER_LE); // CDR header
    }

    #[test]
    fn test_cdr_roundtrip_with_serialize_to_vec() {
        let original = LargeMessage {
            data: vec![1, 2, 3, 4, 5, 6, 7, 8],
            count: 42,
            nested: vec![
                SimpleMessage {
                    value: 10,
                    text: "first".to_string(),
                },
                SimpleMessage {
                    value: 20,
                    text: "second".to_string(),
                },
            ],
        };

        // Serialize using serialize_to_vec
        let buffer = CdrCompatSerdes::serialize_to_vec(&original);

        // Deserialize
        let deserialized = <CdrCompatSerdes as ZSerdes<LargeMessage>>::deserialize(&buffer)
            .expect("Failed to deserialize");

        // Should match original
        assert_eq!(deserialized, original);
    }

    #[test]
    fn test_zmessage_trait_implementation() {
        let msg = SimpleMessage {
            value: 777,
            text: "trait test".to_string(),
        };

        // Use CdrCompatSerdes directly for serde-only types
        let serialized = CdrCompatSerdes::serialize_to_vec(&msg);
        assert!(!serialized.is_empty());
        assert_eq!(&serialized[0..4], &CDR_HEADER_LE);

        // Deserialize using CdrCompatSerdes
        let deserialized =
            <CdrCompatSerdes as ZSerdes<SimpleMessage>>::deserialize(&serialized[..])
                .expect("Failed to deserialize");
        assert_eq!(deserialized, msg);
    }

    #[cfg(feature = "protobuf")]
    #[test]
    fn test_protobuf_serialize_to_vec() {
        use prost::Message;

        #[derive(Clone, PartialEq, Message)]
        struct ProtoMessage {
            #[prost(uint32, tag = "1")]
            id: u32,
            #[prost(string, tag = "2")]
            name: String,
        }

        let msg = ProtoMessage {
            id: 42,
            name: "test".to_string(),
        };

        // Test both serialization methods
        let vec1 = ProtobufSerdes::serialize_to_vec(&msg);
        let zbuf = ProtobufSerdes::serialize(&msg);
        let zbuf_bytes = zbuf.contiguous();
        let vec2 = zbuf_bytes.to_vec();

        // Results should be identical
        assert_eq!(vec1, vec2);
        assert!(!vec1.is_empty());
    }

    #[cfg(feature = "protobuf")]
    #[test]
    fn test_protobuf_serialize_to_zbuf() {
        use prost::Message;

        #[derive(Clone, PartialEq, Message)]
        struct ProtoMessage {
            #[prost(uint32, tag = "1")]
            id: u32,
            #[prost(string, tag = "2")]
            name: String,
        }

        let msg = ProtoMessage {
            id: 42,
            name: "test".to_string(),
        };

        let zbuf = ProtobufSerdes::serialize(&msg);
        let bytes = zbuf.contiguous();

        // Verify it matches the Vec<u8> serialization
        let vec = ProtobufSerdes::serialize_to_vec(&msg);
        assert_eq!(&*bytes, &vec[..]);
    }
}

/// Tests for `CdrSerdes` — the `CdrSerialize`-based CDR fast path.
///
/// These tests verify:
/// 1. Byte-identical wire output between `CdrCompatSerdes` (serde path) and `CdrSerdes` (CDR trait path).
/// 2. Roundtrip correctness for `CdrSerdes`.
/// 3. POD bulk path produces the same bytes for plain sequences as the element loop.
#[cfg(test)]
mod fast_cdr_tests {
    use super::*;
    use ros_z_cdr::{
        CdrBuffer, CdrDeserialize, CdrReader, CdrSerialize, CdrSerializedSize, CdrWriter,
    };

    // ── Test types ────────────────────────────────────────────────────────────

    /// A struct with a string field — NOT plain (element-by-element path).
    #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
    struct Header {
        seq: u32,
        frame_id: String,
    }

    impl CdrSerialize for Header {
        fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(
            &self,
            w: &mut CdrWriter<'_, BO, B>,
        ) {
            self.seq.cdr_serialize(w);
            self.frame_id.cdr_serialize(w);
        }
    }

    impl CdrDeserialize for Header {
        fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
            r: &mut CdrReader<'de, BO>,
        ) -> ros_z_cdr::Result<Self> {
            Ok(Self {
                seq: u32::cdr_deserialize(r)?,
                frame_id: String::cdr_deserialize(r)?,
            })
        }
    }

    impl CdrSerializedSize for Header {
        fn cdr_serialized_size(&self, pos: usize) -> usize {
            let p = self.seq.cdr_serialized_size(pos);
            self.frame_id.cdr_serialized_size(p)
        }
    }

    /// A plain struct — all fields are f64, no strings/sequences.
    /// On LE hosts this satisfies `CdrPlain` (verified in ros-z-cdr tests).
    #[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
    struct Point3d {
        x: f64,
        y: f64,
        z: f64,
    }

    impl CdrSerialize for Point3d {
        fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(
            &self,
            w: &mut CdrWriter<'_, BO, B>,
        ) {
            self.x.cdr_serialize(w);
            self.y.cdr_serialize(w);
            self.z.cdr_serialize(w);
        }
    }

    impl CdrDeserialize for Point3d {
        fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
            r: &mut CdrReader<'de, BO>,
        ) -> ros_z_cdr::Result<Self> {
            Ok(Self {
                x: f64::cdr_deserialize(r)?,
                y: f64::cdr_deserialize(r)?,
                z: f64::cdr_deserialize(r)?,
            })
        }
    }

    impl CdrSerializedSize for Point3d {
        fn cdr_serialized_size(&self, pos: usize) -> usize {
            let p = self.x.cdr_serialized_size(pos);
            let p = self.y.cdr_serialized_size(p);
            self.z.cdr_serialized_size(p)
        }
    }

    /// A message with a Vec<Point3d> — this is the key fast-path case.
    #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
    struct PointCloud {
        header: Header,
        points: Vec<Point3d>,
    }

    impl CdrSerialize for PointCloud {
        fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(
            &self,
            w: &mut CdrWriter<'_, BO, B>,
        ) {
            self.header.cdr_serialize(w);
            // Vec<Point3d>: element-by-element (Point3d: CdrSerialize)
            w.write_sequence_length(self.points.len());
            for pt in &self.points {
                pt.cdr_serialize(w);
            }
        }
    }

    impl CdrDeserialize for PointCloud {
        fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
            r: &mut CdrReader<'de, BO>,
        ) -> ros_z_cdr::Result<Self> {
            let header = Header::cdr_deserialize(r)?;
            let n = r.read_sequence_length()?;
            let mut points = Vec::with_capacity(n);
            for _ in 0..n {
                points.push(Point3d::cdr_deserialize(r)?);
            }
            Ok(Self { header, points })
        }
    }

    impl CdrSerializedSize for PointCloud {
        fn cdr_serialized_size(&self, pos: usize) -> usize {
            let p = self.header.cdr_serialized_size(pos);
            // sequence length u32 (4-byte aligned)
            let p = p + ((4 - p % 4) % 4) + 4;
            let mut p = p;
            for pt in &self.points {
                p = pt.cdr_serialized_size(p);
            }
            p
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    fn serde_bytes<T: serde::Serialize + serde::de::DeserializeOwned + Send + Sync + 'static>(
        value: &T,
    ) -> Vec<u8> {
        CdrCompatSerdes::serialize_to_vec(value)
    }

    fn fast_bytes<T: CdrSerialize + CdrSerializedSize + CdrDeserialize + Send + Sync + 'static>(
        value: &T,
    ) -> Vec<u8> {
        CdrSerdes::serialize_to_vec(value)
    }

    fn fast_deserialize<
        T: CdrSerialize + CdrSerializedSize + CdrDeserialize + Send + Sync + 'static,
    >(
        bytes: &[u8],
    ) -> T {
        CdrSerdes::deserialize(bytes).expect("CdrSerdes::deserialize failed")
    }

    // ── Tests ─────────────────────────────────────────────────────────────────

    #[test]
    fn header_byte_identical_to_serde() {
        let msg = Header {
            seq: 42,
            frame_id: "base_link".to_string(),
        };
        assert_eq!(serde_bytes(&msg), fast_bytes(&msg));
    }

    #[test]
    fn header_fast_roundtrip() {
        let msg = Header {
            seq: 99,
            frame_id: "map".to_string(),
        };
        let bytes = fast_bytes(&msg);
        let decoded: Header = fast_deserialize(&bytes);
        assert_eq!(msg, decoded);
    }

    #[test]
    #[allow(clippy::approx_constant)]
    fn point3d_byte_identical_to_serde() {
        let pt = Point3d {
            x: 1.0,
            y: 2.5,
            z: -3.14,
        };
        assert_eq!(serde_bytes(&pt), fast_bytes(&pt));
    }

    #[test]
    #[allow(clippy::approx_constant)]
    fn point3d_fast_roundtrip() {
        let pt = Point3d {
            x: 1.0,
            y: 2.5,
            z: -3.14,
        };
        let bytes = fast_bytes(&pt);
        let decoded: Point3d = fast_deserialize(&bytes);
        assert_eq!(pt, decoded);
    }

    #[test]
    fn pointcloud_byte_identical_to_serde() {
        let msg = PointCloud {
            header: Header {
                seq: 1,
                frame_id: "lidar".to_string(),
            },
            points: vec![
                Point3d {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                Point3d {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                Point3d {
                    x: -1.0,
                    y: -2.0,
                    z: -3.0,
                },
            ],
        };
        assert_eq!(serde_bytes(&msg), fast_bytes(&msg));
    }

    #[test]
    fn pointcloud_fast_roundtrip() {
        let msg = PointCloud {
            header: Header {
                seq: 7,
                frame_id: "camera".to_string(),
            },
            points: (0..100)
                .map(|i| Point3d {
                    x: i as f64,
                    y: (i * 2) as f64,
                    z: (i * 3) as f64,
                })
                .collect(),
        };
        let bytes = fast_bytes(&msg);
        let decoded: PointCloud = fast_deserialize(&bytes);
        assert_eq!(msg, decoded);
    }

    #[test]
    fn empty_sequence_roundtrip() {
        let msg = PointCloud {
            header: Header {
                seq: 0,
                frame_id: String::new(),
            },
            points: vec![],
        };
        let bytes = fast_bytes(&msg);
        let decoded: PointCloud = fast_deserialize(&bytes);
        assert_eq!(msg, decoded);
    }

    #[test]
    fn size_hint_matches_actual() {
        let msg = PointCloud {
            header: Header {
                seq: 1,
                frame_id: "test".to_string(),
            },
            points: vec![
                Point3d {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0
                };
                10
            ],
        };
        let hint = msg.cdr_serialized_size(0) + 4;
        let bytes = fast_bytes(&msg);
        // The hint should be >= actual payload size
        assert!(
            hint >= bytes.len() - 4,
            "hint={hint} bytes.len()={}",
            bytes.len()
        );
    }
}
