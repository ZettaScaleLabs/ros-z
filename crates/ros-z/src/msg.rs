use byteorder::LittleEndian;
#[cfg(feature = "protobuf")]
use prost::Message as ProstMessage;
use ros_z_cdr::{CdrBuffer, CdrSerializer, ZBufWriter};
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
    /// use ros_z::msg::{ZSerializer, CdrSerdes};
    /// use serde::Serialize;
    ///
    /// #[derive(Serialize)]
    /// struct LargeMsg { data: Vec<u8> }
    ///
    /// let msg = LargeMsg { data: vec![0; 1_000_000] };
    /// let hint = 4 + 4 + 1_000_000;  // header + length + data
    /// let zbuf = CdrSerdes::<LargeMsg>::serialize_to_zbuf_with_hint(&msg, hint);
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
    /// use ros_z::msg::{ZSerializer, CdrSerdes};
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
    /// let (zbuf, size) = CdrSerdes::<MyMsg>::serialize_to_shm(&msg, 128, &provider)?;
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

// Core Z-Message trait
pub trait ZMessage: Send + Sync + Sized + 'static {
    type Serdes: for<'a> ZSerializer<Input<'a> = &'a Self> + ZDeserializer;

    fn serialize(&self) -> Vec<u8> {
        Self::Serdes::serialize(self)
    }

    fn serialize_to_zbuf(&self) -> ZBuf {
        // Use accurate size estimation for optimal buffer allocation
        Self::Serdes::serialize_to_zbuf_with_hint(self, self.estimated_serialized_size())
    }

    fn deserialize(
        input: <Self::Serdes as ZDeserializer>::Input<'_>,
    ) -> Result<Self, <Self::Serdes as ZDeserializer>::Error>
    where
        Self::Serdes: ZDeserializer<Output = Self>,
    {
        Self::Serdes::deserialize(input)
    }

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

// Blanket implementation for serde-compatible types using CDR
impl<T> ZMessage for T
where
    T: Send + Sync + Serialize + for<'a> Deserialize<'a> + 'static,
{
    type Serdes = CdrSerdes<T>;
}

// CDR

pub struct CdrSerdes<T>(PhantomData<T>);

/// CDR encapsulation header for little-endian encoding
const CDR_HEADER_LE: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

impl<T> ZSerializer for CdrSerdes<T>
where
    T: Serialize,
{
    type Input<'a>
        = &'a T
    where
        T: 'a;

    fn serialize_to_zbuf(input: &T) -> ZBuf {
        // Use fixed 256-byte capacity for backward compatibility
        Self::serialize_to_zbuf_with_hint(input, 256)
    }

    fn serialize_to_zbuf_with_hint(input: &T, capacity_hint: usize) -> ZBuf {
        // Create ZBufWriter with provided capacity hint for optimal allocation
        let mut writer = ZBufWriter::with_capacity(capacity_hint);

        // Write CDR header
        writer.extend_from_slice(&CDR_HEADER_LE);

        // Serialize payload directly to ZBufWriter
        let mut serializer = CdrSerializer::<LittleEndian, ZBufWriter>::new(&mut writer);
        input.serialize(&mut serializer).unwrap();

        // Convert to ZBuf (transfers ownership, no copy)
        writer.into_zbuf()
    }

    fn serialize_to_shm(
        input: &T,
        estimated_size: usize,
        provider: &zenoh::shm::ShmProvider<zenoh::shm::PosixShmProviderBackend>,
    ) -> zenoh::Result<(ZBuf, usize)> {
        // Create SHM writer with estimated size
        let mut writer = crate::shm::ShmWriter::new(provider, estimated_size)?;

        // Write CDR header
        writer.extend_from_slice(&CDR_HEADER_LE);

        // Serialize payload directly to SHM buffer
        let mut serializer = CdrSerializer::<LittleEndian, crate::shm::ShmWriter>::new(&mut writer);
        input
            .serialize(&mut serializer)
            .map_err(|e| zenoh::Error::from(format!("CDR serialization failed: {}", e)))?;

        // Get actual serialized size
        let actual_size = writer.position();

        // Convert to ZBuf (SHM-backed, zero-copy!)
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

        // STEP 1: Write CDR header FIRST (4 bytes)
        // This avoids the O(n) memmove that would be needed if we prepended it later
        buffer.extend_from_slice(&CDR_HEADER_LE);

        // STEP 2: Serialize payload using CdrSerializer
        // Zero buffer swaps - the serializer works directly on our buffer!
        let mut fast_ser = CdrSerializer::<LittleEndian>::new(buffer);
        input.serialize(&mut fast_ser).unwrap();
        // Buffer is automatically updated through the mutable reference

        // Done! Header is at position 0, payload follows immediately after
    }
}

impl<T> ZDeserializer for CdrSerdes<T>
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
        let header = &input[0..4];
        let representation_identifier = &header[0..2];
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

// Protobuf

#[cfg(feature = "protobuf")]
pub struct ProtobufSerdes<T>(PhantomData<T>);

#[cfg(feature = "protobuf")]
impl<T> ZSerializer for ProtobufSerdes<T>
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
impl<T> ZDeserializer for ProtobufSerdes<T>
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

        let zbuf = CdrSerdes::<SimpleMessage>::serialize_to_zbuf(&msg);
        let bytes = zbuf.contiguous();

        // Verify CDR header
        assert_eq!(&bytes[0..4], &CDR_HEADER_LE);

        // Verify roundtrip
        let deserialized = CdrSerdes::<SimpleMessage>::deserialize(&bytes).unwrap();
        assert_eq!(deserialized, msg);
    }

    #[test]
    fn test_serialize_to_zbuf_consistency() {
        let msg = SimpleMessage {
            value: 123,
            text: "consistency test".to_string(),
        };

        // Both methods should produce identical bytes
        let zbuf = CdrSerdes::<SimpleMessage>::serialize_to_zbuf(&msg);
        let vec = CdrSerdes::<SimpleMessage>::serialize(&msg);

        let zbuf_bytes = zbuf.contiguous();
        assert_eq!(&*zbuf_bytes, &vec[..]);
    }

    #[test]
    fn test_serialize_to_zbuf_reuse() {
        let msg1 = SimpleMessage {
            value: 1,
            text: "first".to_string(),
        };
        let msg2 = SimpleMessage {
            value: 2,
            text: "second".to_string(),
        };

        let mut buffer = Vec::with_capacity(1024);

        // First serialization
        let zbuf1 = CdrSerdes::<SimpleMessage>::serialize_to_zbuf_reuse(&msg1, &mut buffer);
        let bytes1 = zbuf1.contiguous();

        // Buffer should be empty after take
        assert!(buffer.is_empty());

        // Second serialization (buffer will be reallocated)
        let zbuf2 = CdrSerdes::<SimpleMessage>::serialize_to_zbuf_reuse(&msg2, &mut buffer);
        let bytes2 = zbuf2.contiguous();

        // Verify roundtrips
        let decoded1 = CdrSerdes::<SimpleMessage>::deserialize(&bytes1).unwrap();
        let decoded2 = CdrSerdes::<SimpleMessage>::deserialize(&bytes2).unwrap();

        assert_eq!(decoded1, msg1);
        assert_eq!(decoded2, msg2);
    }

    #[test]
    fn test_zmessage_serialize_to_zbuf() {
        let msg = SimpleMessage {
            value: 777,
            text: "trait test".to_string(),
        };

        // ZMessage trait provides serialize_to_zbuf method
        let zbuf = msg.serialize_to_zbuf();
        let bytes = zbuf.contiguous();

        assert_eq!(&bytes[0..4], &CDR_HEADER_LE);

        let deserialized = <SimpleMessage as ZMessage>::deserialize(&bytes).unwrap();
        assert_eq!(deserialized, msg);
    }

    #[test]
    fn test_cdr_serialize_to_buf_consistency() {
        let msg = SimpleMessage {
            value: 42,
            text: "Hello, ros-z!".to_string(),
        };

        // Serialize using both methods
        let vec1 = CdrSerdes::<SimpleMessage>::serialize(&msg);
        let mut vec2 = Vec::new();
        CdrSerdes::<SimpleMessage>::serialize_to_buf(&msg, &mut vec2);

        // Results should be identical
        assert_eq!(vec1, vec2);
        assert!(!vec1.is_empty());
        assert_eq!(&vec1[0..4], &CDR_HEADER_LE); // CDR header
    }

    #[test]
    fn test_cdr_serialize_to_buf_reuses_capacity() {
        let msg = SimpleMessage {
            value: 123,
            text: "test".to_string(),
        };

        let mut buffer = Vec::with_capacity(1024);
        CdrSerdes::<SimpleMessage>::serialize_to_buf(&msg, &mut buffer);

        let capacity_after_first = buffer.capacity();
        assert_eq!(capacity_after_first, 1024);

        // Serialize again - should reuse capacity
        CdrSerdes::<SimpleMessage>::serialize_to_buf(&msg, &mut buffer);
        assert_eq!(buffer.capacity(), capacity_after_first);
    }

    #[test]
    fn test_cdr_serialize_to_buf_clears_previous_data() {
        let msg1 = LargeMessage {
            data: vec![1; 1000],
            count: 100,
            nested: vec![],
        };

        let msg2 = SimpleMessage {
            value: 1,
            text: "x".to_string(),
        };

        let mut buffer = Vec::new();

        // Serialize large message
        CdrSerdes::<LargeMessage>::serialize_to_buf(&msg1, &mut buffer);
        let len1 = buffer.len();
        assert!(len1 > 100);

        // Serialize small message - should clear buffer first
        CdrSerdes::<SimpleMessage>::serialize_to_buf(&msg2, &mut buffer);
        let len2 = buffer.len();
        assert!(len2 < len1);

        // Verify content is correct (not mixed)
        assert_eq!(&buffer[0..4], &CDR_HEADER_LE); // CDR header
    }

    #[test]
    fn test_cdr_roundtrip_with_serialize_to_buf() {
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

        // Serialize using serialize_to_buf
        let mut buffer = Vec::new();
        CdrSerdes::<LargeMessage>::serialize_to_buf(&original, &mut buffer);

        // Deserialize
        let deserialized =
            CdrSerdes::<LargeMessage>::deserialize(&buffer).expect("Failed to deserialize");

        // Should match original
        assert_eq!(deserialized, original);
    }

    #[test]
    fn test_serialize_to_buf_with_empty_buffer() {
        let msg = SimpleMessage {
            value: 99,
            text: "empty buffer test".to_string(),
        };

        let mut buffer = Vec::new();
        assert_eq!(buffer.capacity(), 0);

        CdrSerdes::<SimpleMessage>::serialize_to_buf(&msg, &mut buffer);

        assert!(!buffer.is_empty());
        assert!(buffer.capacity() > 0);
        assert_eq!(&buffer[0..4], &CDR_HEADER_LE); // CDR header
    }

    #[test]
    fn test_serialize_to_buf_multiple_messages() {
        let messages = vec![
            SimpleMessage {
                value: 1,
                text: "one".to_string(),
            },
            SimpleMessage {
                value: 2,
                text: "two".to_string(),
            },
            SimpleMessage {
                value: 3,
                text: "three".to_string(),
            },
        ];

        let mut buffer = Vec::new();
        let mut all_serialized = Vec::new();

        for msg in &messages {
            CdrSerdes::<SimpleMessage>::serialize_to_buf(msg, &mut buffer);
            all_serialized.push(buffer.clone());

            // Verify each serialization is correct
            let deserialized =
                CdrSerdes::<SimpleMessage>::deserialize(&buffer).expect("Failed to deserialize");
            assert_eq!(&deserialized, msg);
        }

        // Verify all serializations are different
        assert_ne!(all_serialized[0], all_serialized[1]);
        assert_ne!(all_serialized[1], all_serialized[2]);
    }

    #[test]
    fn test_zmessage_trait_implementation() {
        let msg = SimpleMessage {
            value: 777,
            text: "trait test".to_string(),
        };

        // ZMessage trait provides serialize method
        let serialized = ZMessage::serialize(&msg);
        assert!(!serialized.is_empty());
        assert_eq!(&serialized[0..4], &CDR_HEADER_LE);

        // ZMessage trait provides deserialize method
        let deserialized = <SimpleMessage as ZMessage>::deserialize(&serialized[..])
            .expect("Failed to deserialize");
        assert_eq!(deserialized, msg);
    }

    #[cfg(feature = "protobuf")]
    #[test]
    fn test_protobuf_serialize_to_buf() {
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
        let vec1 = ProtobufSerdes::<ProtoMessage>::serialize(&msg);
        let mut vec2 = Vec::new();
        ProtobufSerdes::<ProtoMessage>::serialize_to_buf(&msg, &mut vec2);

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

        let zbuf = ProtobufSerdes::<ProtoMessage>::serialize_to_zbuf(&msg);
        let bytes = zbuf.contiguous();

        // Verify it matches the Vec<u8> serialization
        let vec = ProtobufSerdes::<ProtoMessage>::serialize(&msg);
        assert_eq!(&*bytes, &vec[..]);
    }
}
