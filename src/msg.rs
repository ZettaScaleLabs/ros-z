use std::marker::PhantomData;

use cdr::{CdrLe, Infinite};
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
    type Output = T;
    fn deserialize(input: Self::Input<'_>) -> Self::Output {
        cdr::deserialize::<T>(input).unwrap()
    }
}

impl<T> ZSerializer for CdrSerdes<T>
where
    for<'a> T: Serialize + 'a,
{
    type Input<'a> = &'a T;
    fn serialize(input: &T) -> Vec<u8> {
        cdr::serialize::<_, _, CdrLe>(input, Infinite).unwrap()
    }
}

impl<T> ZMessage for T
where
    for<'a> T: Serialize + Deserialize<'a> + 'a,
{
    type Serdes = CdrSerdes<T>;
}
