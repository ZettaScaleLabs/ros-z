use byteorder::LittleEndian;
#[cfg(feature = "protobuf")]
use prost::Message as ProstMessage;
use serde::{Deserialize, Serialize};
use std::marker::PhantomData;

pub trait ZSerializer {
    type Input<'a>
    where
        Self: 'a;
    fn serialize(input: Self::Input<'_>) -> Vec<u8>;
}

pub trait ZDeserializer {
    type Input<'a>;
    type Output;
    fn deserialize(input: Self::Input<'_>) -> Self::Output;
}

// Core Z-Message trait
pub trait ZMessage: Sized {
    type Serdes: for<'a> ZSerializer<Input<'a> = &'a Self> + ZDeserializer;

    fn serialize(&self) -> Vec<u8> {
        Self::Serdes::serialize(self)
    }

    fn deserialize(input: <Self::Serdes as ZDeserializer>::Input<'_>) -> Self
    where
        Self::Serdes: ZDeserializer<Output = Self>,
    {
        Self::Serdes::deserialize(input)
    }
}

// Blanket implementation for serde-compatible types using CDR
impl<T> ZMessage for T
where
    T: Serialize + for<'a> Deserialize<'a> + 'static,
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
        cdr_encoding::to_vec::<T, LittleEndian>(input).unwrap()
    }
}

impl<T> ZDeserializer for CdrSerdes<T>
where
    for<'a> T: Deserialize<'a>,
{
    type Input<'b> = &'b [u8];
    type Output = T;

    fn deserialize(input: Self::Input<'_>) -> T {
        cdr_encoding::from_bytes::<T, LittleEndian>(input)
            .unwrap()
            .0
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

    fn deserialize(input: &[u8]) -> T {
        T::decode(input).unwrap_or_default()
    }
}

pub trait ZService {
    type Request: ZMessage;
    type Response: ZMessage;
}
