//! CDR Deserializer for ROS-Z messages.

use std::marker::PhantomData;

use byteorder::{ByteOrder, ReadBytesExt};
use serde::de::{
    self, DeserializeSeed, EnumAccess, IntoDeserializer, MapAccess, SeqAccess, VariantAccess,
    Visitor,
};

use crate::error::{Error, Result};

/// Deserializer type for converting CDR data stream to Rust objects.
///
/// `CdrDeserializer` is about three machine words of data, so fairly cheap to create.
pub struct CdrDeserializer<'i, BO> {
    phantom: PhantomData<BO>,
    input: &'i [u8],
    serialized_data_count: usize,
}

impl<'de, BO> CdrDeserializer<'de, BO>
where
    BO: ByteOrder,
{
    /// Create a new deserializer from input bytes.
    #[inline]
    pub fn new(input: &'de [u8]) -> CdrDeserializer<'de, BO> {
        CdrDeserializer::<BO> {
            phantom: PhantomData,
            input,
            serialized_data_count: 0,
        }
    }

    /// How many bytes of input stream have been consumed.
    #[inline]
    pub fn bytes_consumed(&self) -> usize {
        self.serialized_data_count
    }

    /// Read the first bytes in the input.
    ///
    /// Returns a slice with lifetime `'de` (the input data lifetime),
    /// enabling zero-copy borrowed deserialization.
    #[inline]
    fn next_bytes(&mut self, count: usize) -> Result<&'de [u8]> {
        if count <= self.input.len() {
            let (head, tail) = self.input.split_at(count);
            self.input = tail;
            self.serialized_data_count += count;
            Ok(head)
        } else {
            Err(Error::Eof)
        }
    }

    /// Consume and discard bytes (for padding).
    #[inline]
    fn remove_bytes_from_input(&mut self, count: usize) -> Result<()> {
        let _pad = self.next_bytes(count)?;
        Ok(())
    }

    /// Calculate and remove alignment padding.
    #[inline]
    fn align(&mut self, type_octet_alignment: usize) -> Result<()> {
        let modulo = self.serialized_data_count % type_octet_alignment;
        if modulo == 0 {
            Ok(())
        } else {
            let padding = type_octet_alignment - modulo;
            self.remove_bytes_from_input(padding)
        }
    }
}

/// Deserialize an object from `&[u8]` based on a [`serde::Deserialize`] implementation.
///
/// Returns deserialized object + count of bytes consumed.
///
/// For zero-copy deserialization of borrowed types (like `&str`), the input
/// bytes must outlive the deserialized value.
#[inline]
pub fn from_bytes<'de, T, BO>(input_bytes: &'de [u8]) -> Result<(T, usize)>
where
    T: serde::Deserialize<'de>,
    BO: ByteOrder,
{
    from_bytes_with::<PhantomData<T>, BO>(input_bytes, PhantomData)
}

/// Deserialize type based on a [`serde::Deserialize`] implementation.
///
/// Returns deserialized object + count of bytes consumed.
#[inline]
pub fn from_bytes_with<'de, S, BO>(input_bytes: &'de [u8], decoder: S) -> Result<(S::Value, usize)>
where
    S: DeserializeSeed<'de>,
    BO: ByteOrder,
{
    let mut deserializer = CdrDeserializer::<BO>::new(input_bytes);
    let t = decoder.deserialize(&mut deserializer)?;
    Ok((t, deserializer.serialized_data_count))
}

impl<'de, BO> de::Deserializer<'de> for &mut CdrDeserializer<'de, BO>
where
    BO: ByteOrder,
{
    type Error = Error;

    /// CDR serialization is not a self-describing data format.
    fn deserialize_any<V>(self, _visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        Err(Error::NotSelfDescribingFormat(
            "CDR cannot deserialize \"any\" type.".to_string(),
        ))
    }

    /// Boolean values are encoded as single octets (0 or 1).
    fn deserialize_bool<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        match self.next_bytes(1)?.first().unwrap() {
            0 => visitor.visit_bool(false),
            1 => visitor.visit_bool(true),
            x => Err(Error::BadBoolean(*x)),
        }
    }

    fn deserialize_i8<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        visitor.visit_i8(self.next_bytes(1)?.read_i8().unwrap())
    }

    fn deserialize_u8<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        visitor.visit_u8(self.next_bytes(1)?.read_u8().unwrap())
    }

    fn deserialize_i16<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.align(2)?;
        visitor.visit_i16(self.next_bytes(2)?.read_i16::<BO>().unwrap())
    }

    fn deserialize_u16<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.align(2)?;
        visitor.visit_u16(self.next_bytes(2)?.read_u16::<BO>().unwrap())
    }

    fn deserialize_i32<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.align(4)?;
        visitor.visit_i32(self.next_bytes(4)?.read_i32::<BO>().unwrap())
    }

    fn deserialize_u32<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.align(4)?;
        visitor.visit_u32(self.next_bytes(4)?.read_u32::<BO>().unwrap())
    }

    fn deserialize_i64<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.align(8)?;
        visitor.visit_i64(self.next_bytes(8)?.read_i64::<BO>().unwrap())
    }

    fn deserialize_u64<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.align(8)?;
        visitor.visit_u64(self.next_bytes(8)?.read_u64::<BO>().unwrap())
    }

    fn deserialize_f32<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.align(4)?;
        visitor.visit_f32(self.next_bytes(4)?.read_f32::<BO>().unwrap())
    }

    fn deserialize_f64<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.align(8)?;
        visitor.visit_f64(self.next_bytes(8)?.read_f64::<BO>().unwrap())
    }

    /// Since this is Rust, a char is 32-bit Unicode codepoint.
    fn deserialize_char<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.align(4)?;
        let codepoint = self.next_bytes(4)?.read_u32::<BO>().unwrap();
        match char::from_u32(codepoint) {
            Some(c) => visitor.visit_char(c),
            None => Err(Error::BadChar(codepoint)),
        }
    }

    fn deserialize_str<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        // Align and read string length (includes null terminator)
        self.align(4)?;
        let bytes_len = self.next_bytes(4)?.read_u32::<BO>().unwrap() as usize;
        let bytes = self.next_bytes(bytes_len)?;

        // Remove the null terminating character
        let bytes_without_null = match bytes.split_last() {
            None => bytes, // Empty string edge case
            Some((null_char, contents)) => {
                if *null_char != 0 {
                    // Warn but continue - some implementations may not null-terminate
                }
                contents
            }
        };

        std::str::from_utf8(bytes_without_null)
            .map_err(Error::BadUTF8)
            .and_then(|s| visitor.visit_borrowed_str(s))
    }

    fn deserialize_string<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        // For owned strings, still use borrowed path - serde will copy if needed
        self.deserialize_str(visitor)
    }

    /// OPTIMIZED: Read bytes efficiently in bulk instead of element-by-element.
    /// This is critical for large byte arrays (images, point clouds, etc.).
    /// Uses zero-copy borrowed bytes when possible.
    fn deserialize_bytes<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        // Align to 4 bytes (for the length prefix)
        self.align(4)?;
        // Read length prefix
        let len = self.next_bytes(4)?.read_u32::<BO>().unwrap() as usize;
        // Read the entire buffer at once - use borrowed slice (zero-copy)
        let bytes = self.next_bytes(len)?;
        visitor.visit_borrowed_bytes(bytes)
    }

    fn deserialize_byte_buf<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        // Align to 4 bytes
        self.align(4)?;
        // Length prefix
        let len = self.next_bytes(4)?.read_u32::<BO>().unwrap() as usize;
        // Read the entire buffer at once
        let buf = self.next_bytes(len)?.to_vec();
        visitor.visit_byte_buf(buf)
    }

    #[inline]
    fn deserialize_option<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.align(4)?;
        let enum_tag = self.next_bytes(4)?.read_u32::<BO>().unwrap();
        match enum_tag {
            0 => visitor.visit_none(),
            1 => visitor.visit_some(self),
            wtf => Err(Error::BadOption(wtf)),
        }
    }

    #[inline]
    fn deserialize_unit<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        // Unit data is not put on wire
        visitor.visit_unit()
    }

    #[inline]
    fn deserialize_unit_struct<V>(self, _name: &'static str, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.deserialize_unit(visitor)
    }

    #[inline]
    fn deserialize_newtype_struct<V>(self, _name: &'static str, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        visitor.visit_newtype_struct(self)
    }

    /// Sequences are encoded as an unsigned long value, followed by the elements.
    #[inline]
    fn deserialize_seq<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.align(4)?;
        let element_count = self.next_bytes(4)?.read_u32::<BO>().unwrap() as usize;
        visitor.visit_seq(SequenceHelper::new(self, element_count))
    }

    /// Fixed length array - number of elements is not included.
    #[inline]
    fn deserialize_tuple<V>(self, len: usize, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        visitor.visit_seq(SequenceHelper::new(self, len))
    }

    #[inline]
    fn deserialize_tuple_struct<V>(
        self,
        _name: &'static str,
        len: usize,
        visitor: V,
    ) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        visitor.visit_seq(SequenceHelper::new(self, len))
    }

    #[inline]
    fn deserialize_map<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.align(4)?;
        let element_count = self.next_bytes(4)?.read_u32::<BO>().unwrap() as usize;
        visitor.visit_map(SequenceHelper::new(self, element_count))
    }

    #[inline]
    fn deserialize_struct<V>(
        self,
        _name: &'static str,
        fields: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        visitor.visit_seq(SequenceHelper::new(self, fields.len()))
    }

    /// Enum values are encoded as unsigned longs (u32).
    #[inline]
    fn deserialize_enum<V>(
        self,
        _name: &'static str,
        _variants: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.align(4)?;
        visitor.visit_enum(EnumerationHelper::<BO>::new(self))
    }

    #[inline]
    fn deserialize_identifier<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.deserialize_u32(visitor)
    }

    #[inline]
    fn deserialize_ignored_any<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.deserialize_any(visitor)
    }

    #[inline]
    fn is_human_readable(&self) -> bool {
        false
    }
}

// ----------------------------------------------------------

struct EnumerationHelper<'a, 'de, BO> {
    de: &'a mut CdrDeserializer<'de, BO>,
}

impl<'a, 'de, BO> EnumerationHelper<'a, 'de, BO>
where
    BO: ByteOrder,
{
    #[inline]
    fn new(de: &'a mut CdrDeserializer<'de, BO>) -> Self {
        EnumerationHelper::<BO> { de }
    }
}

impl<'de, 'a, BO> EnumAccess<'de> for EnumerationHelper<'a, 'de, BO>
where
    BO: ByteOrder,
{
    type Error = Error;
    type Variant = Self;

    #[inline]
    fn variant_seed<V>(self, seed: V) -> Result<(V::Value, Self::Variant)>
    where
        V: DeserializeSeed<'de>,
    {
        // preceding deserialize_enum aligned to 4
        let enum_tag = self.de.next_bytes(4)?.read_u32::<BO>().unwrap();
        let val: Result<_> = seed.deserialize(enum_tag.into_deserializer());
        Ok((val?, self))
    }
}

impl<'de, 'a, BO> VariantAccess<'de> for EnumerationHelper<'a, 'de, BO>
where
    BO: ByteOrder,
{
    type Error = Error;

    #[inline]
    fn unit_variant(self) -> Result<()> {
        Ok(())
    }

    #[inline]
    fn newtype_variant_seed<T>(self, seed: T) -> Result<T::Value>
    where
        T: DeserializeSeed<'de>,
    {
        seed.deserialize(self.de)
    }

    #[inline]
    fn tuple_variant<V>(self, len: usize, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        de::Deserializer::deserialize_tuple(self.de, len, visitor)
    }

    #[inline]
    fn struct_variant<V>(self, fields: &'static [&'static str], visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        de::Deserializer::deserialize_tuple(self.de, fields.len(), visitor)
    }
}

// ----------------------------------------------------------

struct SequenceHelper<'a, 'de, BO> {
    de: &'a mut CdrDeserializer<'de, BO>,
    element_counter: usize,
    expected_count: usize,
}

impl<'a, 'de, BO> SequenceHelper<'a, 'de, BO> {
    #[inline]
    fn new(de: &'a mut CdrDeserializer<'de, BO>, expected_count: usize) -> Self {
        SequenceHelper {
            de,
            element_counter: 0,
            expected_count,
        }
    }
}

impl<'a, 'de, BO> SeqAccess<'de> for SequenceHelper<'a, 'de, BO>
where
    BO: ByteOrder,
{
    type Error = Error;

    #[inline]
    fn next_element_seed<T>(&mut self, seed: T) -> Result<Option<T::Value>>
    where
        T: DeserializeSeed<'de>,
    {
        if self.element_counter == self.expected_count {
            Ok(None)
        } else {
            self.element_counter += 1;
            seed.deserialize(&mut *self.de).map(Some)
        }
    }
}

impl<'de, 'a, BO> MapAccess<'de> for SequenceHelper<'a, 'de, BO>
where
    BO: ByteOrder,
{
    type Error = Error;

    #[inline]
    fn next_key_seed<K>(&mut self, seed: K) -> Result<Option<K::Value>>
    where
        K: DeserializeSeed<'de>,
    {
        if self.element_counter == self.expected_count {
            Ok(None)
        } else {
            self.element_counter += 1;
            seed.deserialize(&mut *self.de).map(Some)
        }
    }

    #[inline]
    fn next_value_seed<V>(&mut self, seed: V) -> Result<V::Value>
    where
        V: DeserializeSeed<'de>,
    {
        seed.deserialize(&mut *self.de)
    }
}
