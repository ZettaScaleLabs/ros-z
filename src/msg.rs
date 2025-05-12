use cdr::{CdrLe, Infinite};
use serde::{Deserialize, Serialize};
use std::marker::PhantomData;

pub trait Deserializer {
    type Input<'a>;
    type Output;
}

pub trait ZDeserializer {
    type Input<'a>;
    type Output;
    fn deserialize<'a>(input: Self::Input<'a>) -> Self::Output;
}

pub trait ZSerializer {
    type Input<'a>;
    fn serialize<'a>(input: Self::Input<'a>) -> Vec<u8>;
}

pub trait ZMessage {
    type Serder: for<'a> ZSerializer<Input<'a> = &'a Self> + ZDeserializer;
    fn serialize(&self) -> Vec<u8> {
        Self::Serder::serialize(self)
    }

    fn deserialize<'a>(
        input: <Self::Serder as ZDeserializer>::Input<'a>,
    ) -> <Self::Serder as ZDeserializer>::Output {
        Self::Serder::deserialize(input)
    }
}

pub trait Message {
    type De: Deserializer;
    fn deserialize<'a>(
        input: <Self::De as Deserializer>::Input<'a>,
    ) -> <Self::De as Deserializer>::Output;
    fn serialize(&self) -> Vec<u8>;
}

impl<'b, T> Message for T
where
    T: Serialize + Deserialize<'b>,
{
    type De = CdrDeserializer<T>;
    fn deserialize<'a>(bytes: &'a [u8]) -> T {
        Self::De::de(bytes)
    }
    fn serialize(&self) -> Vec<u8> {
        Self::De::se(self)
    }
}

pub struct CdrDeserializer<T>(PhantomData<T>);

impl<'a, T> CdrDeserializer<T>
where
    T: Serialize + Deserialize<'a>,
{
    fn se(msg: &T) -> Vec<u8> {
        cdr::serialize::<_, _, CdrLe>(&msg, Infinite)
            .unwrap()
            .into()
    }

    fn de(bytes: &[u8]) -> T {
        cdr::deserialize::<T>(bytes).unwrap()
    }
}

impl<T> Deserializer for CdrDeserializer<T> {
    type Output = T;
    type Input<'a> = &'a [u8];
}

pub struct CdrSerder<T>(PhantomData<T>);

impl<T> ZDeserializer for CdrSerder<T>
where
    for<'a> T: Deserialize<'a>,
{
    type Input<'a> = &'a [u8];
    type Output = T;
    fn deserialize<'a>(input: Self::Input<'a>) -> Self::Output {
        cdr::deserialize::<T>(input).unwrap()
    }
}

impl<T> ZSerializer for CdrSerder<T>
where
    for<'a> T: Serialize + 'a,
{
    type Input<'a> = &'a T;
    fn serialize<'a>(bytes: &'a T) -> Vec<u8> {
        cdr::serialize::<_, _, CdrLe>(bytes, Infinite)
            .unwrap()
            .into()
    }
}

impl<T> ZMessage for T
where
    for<'a> T: Serialize + Deserialize<'a> + 'a,
{
    type Serder = CdrSerder<T>;
}
