use std::marker::PhantomData;

use prost::Message;
use serde::{Deserialize, Serialize};

pub trait ZDeserializer {
    type Input<'a>;
    type Output;
    fn deserialize(input: Self::Input<'_>) -> Self::Output;
}

pub trait ZSerializer {
    type Input<'a>;
    fn serialize(input: Self::Input<'_>) -> Vec<u8>;
}

pub trait ZMessage {
    type Serdes: for<'a> ZSerializer<Input<'a> = &'a Self> + ZDeserializer;
    fn serialize(&self) -> Vec<u8> {
        Self::Serdes::serialize(self)
    }

    fn deserialize(
        input: <Self::Serdes as ZDeserializer>::Input<'_>,
    ) -> <Self::Serdes as ZDeserializer>::Output {
        Self::Serdes::deserialize(input)
    }
}

pub struct CdrSerdes<T>(PhantomData<T>);

impl<T> ZDeserializer for CdrSerdes<T>
where
    for<'a> T: Deserialize<'a>,
{
    type Input<'a> = &'a [u8];
    // FIXME: Use a result type
    type Output = T;
    fn deserialize(input: Self::Input<'_>) -> Self::Output {
        // Skip the first four bytes
        let x = cdr_encoding::from_bytes::<T, byteorder::LittleEndian>(&input[4..]).unwrap();
        x.0
    }
}

impl<T> ZSerializer for CdrSerdes<T>
where
    for<'a> T: Serialize + 'a,
{
    type Input<'a> = &'a T;
    fn serialize(input: &T) -> Vec<u8> {
        // Allocate with enough space: 4 bytes header + payload
        let mut buffer = Vec::with_capacity(std::mem::size_of_val(input) * 2 + 4);
        // Write the encapsulation header (CDR LE = 0x00 01 00 00)
        buffer.extend_from_slice(&[0x00, 0x01, 0x00, 0x00]);
        // Serialize directly into the same buffer (appends after the header)
        cdr_encoding::to_writer::<T, byteorder::LittleEndian, _>(&mut buffer, input).unwrap();

        buffer
    }
}

pub struct ProtobufSerdes<T>(PhantomData<T>);

impl<T> ZDeserializer for ProtobufSerdes<T>
where
    T: Message + Default,
{
    type Input<'a> = &'a [u8];
    type Output = T;
    fn deserialize(input: Self::Input<'_>) -> Self::Output {
        T::decode(input).unwrap()
    }
}

impl<T> ZSerializer for ProtobufSerdes<T>
where
    T: Message + 'static,
{
    type Input<'a>
        = &'a T
    where
        T: 'a;
    fn serialize(input: &T) -> Vec<u8> {
        input.encode_to_vec()
    }
}

// Default implementation for compatibility
impl<T> ZMessage for T
where
    for<'a> T: Serialize + Deserialize<'a> + 'a,
{
    type Serdes = CdrSerdes<T>;
}

pub trait ZService {
    type Request: ZMessage;
    type Response: ZMessage;
}
