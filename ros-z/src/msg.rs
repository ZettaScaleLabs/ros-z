use byteorder::LittleEndian;
#[cfg(feature = "protobuf")]
use prost::Message as ProstMessage;
use serde::{Deserialize, Serialize};
use std::marker::PhantomData;

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
    fn serialize(input: Self::Input<'_>) -> Vec<u8>;
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

    fn deserialize(input: <Self::Serdes as ZDeserializer>::Input<'_>) -> Result<Self, <Self::Serdes as ZDeserializer>::Error>
    where
        Self::Serdes: ZDeserializer<Output = Self>,
    {
        Self::Serdes::deserialize(input)
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

impl<T> ZSerializer for CdrSerdes<T>
where
    T: Serialize,
{
    type Input<'a>
        = &'a T
    where
        T: 'a;

    fn serialize(input: &T) -> Vec<u8> {
        let mut buffer = cdr_encoding::to_vec::<T, LittleEndian>(input).unwrap();
        // Prepend CDR encapsulation header (0x00 0x01 0x00 0x00 = CDR LE)
        let mut result = vec![0x00, 0x01, 0x00, 0x00];
        result.append(&mut buffer);
        result
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
        // Skip the first four bytes (CDR encapsulation header)
        let x = cdr_encoding::from_bytes::<T, byteorder::LittleEndian>(&input[4..])
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

    fn serialize(input: &T) -> Vec<u8> {
        // Use prost's builtin encode_to_vec for direct serialization
        input.encode_to_vec()
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
