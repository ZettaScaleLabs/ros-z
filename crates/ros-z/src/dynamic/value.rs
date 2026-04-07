//! Runtime representation of ROS 2 values.
//!
//! This module provides the `DynamicValue` enum for representing any ROS 2
//! value at runtime, along with conversion traits.

use std::sync::Arc;

use super::message::DynamicMessage;
use super::schema::{EnumPayloadSchema, EnumSchema, FieldType};

/// Runtime representation of any ROS 2 value.
#[derive(Clone, Debug, PartialEq)]
pub enum DynamicValue {
    // Primitives
    Bool(bool),
    Int8(i8),
    Int16(i16),
    Int32(i32),
    Int64(i64),
    Uint8(u8),
    Uint16(u16),
    Uint32(u32),
    Uint64(u64),
    Float32(f32),
    Float64(f64),
    String(String),

    /// Optimized byte array (for uint8[] / byte[])
    Bytes(Vec<u8>),

    /// Nested message
    Message(Box<DynamicMessage>),
    /// Optional value encoded with a `u32` presence tag.
    Optional(Option<Box<DynamicValue>>),
    /// Tagged enum encoded with a `u32` variant index.
    Enum(EnumValue),

    /// Collections (homogeneous)
    Array(Vec<DynamicValue>),
}

/// Runtime representation of a serde enum value.
#[derive(Clone, Debug, PartialEq)]
pub struct EnumValue {
    pub variant_index: u32,
    pub variant_name: String,
    pub payload: EnumPayloadValue,
}

impl EnumValue {
    /// Create a new enum value.
    pub fn new(
        variant_index: u32,
        variant_name: impl Into<String>,
        payload: EnumPayloadValue,
    ) -> Self {
        Self {
            variant_index,
            variant_name: variant_name.into(),
            payload,
        }
    }
}

/// Runtime payload value for a serde enum variant.
#[derive(Clone, Debug, PartialEq)]
pub enum EnumPayloadValue {
    Unit,
    Newtype(Box<DynamicValue>),
    Tuple(Vec<DynamicValue>),
    Struct(Vec<DynamicNamedValue>),
}

/// Named field value used by struct enum variants.
#[derive(Clone, Debug, PartialEq)]
pub struct DynamicNamedValue {
    pub name: String,
    pub value: DynamicValue,
}

/// Macro to generate accessor methods for primitive types.
macro_rules! impl_primitive_accessors {
    ($($method:ident -> $variant:ident : $ty:ty),* $(,)?) => {
        impl DynamicValue {
            $(
                #[doc = concat!("Try to extract as ", stringify!($ty), ".")]
                pub fn $method(&self) -> Option<$ty> {
                    match self {
                        DynamicValue::$variant(v) => Some(*v),
                        _ => None,
                    }
                }
            )*
        }
    };
}

impl_primitive_accessors! {
    as_bool -> Bool: bool,
    as_i8 -> Int8: i8,
    as_i16 -> Int16: i16,
    as_i32 -> Int32: i32,
    as_i64 -> Int64: i64,
    as_u8 -> Uint8: u8,
    as_u16 -> Uint16: u16,
    as_u32 -> Uint32: u32,
    as_u64 -> Uint64: u64,
    as_f32 -> Float32: f32,
    as_f64 -> Float64: f64,
}

impl DynamicValue {
    /// Try to extract as a string reference.
    pub fn as_str(&self) -> Option<&str> {
        match self {
            DynamicValue::String(v) => Some(v),
            _ => None,
        }
    }

    /// Try to extract as a byte slice.
    pub fn as_bytes(&self) -> Option<&[u8]> {
        match self {
            DynamicValue::Bytes(v) => Some(v),
            _ => None,
        }
    }

    /// Try to extract as a message reference.
    pub fn as_message(&self) -> Option<&DynamicMessage> {
        match self {
            DynamicValue::Message(v) => Some(v),
            _ => None,
        }
    }

    /// Try to extract as a mutable message reference.
    pub fn as_message_mut(&mut self) -> Option<&mut DynamicMessage> {
        match self {
            DynamicValue::Message(v) => Some(v),
            _ => None,
        }
    }

    /// Try to extract as an optional reference.
    pub fn as_optional(&self) -> Option<Option<&DynamicValue>> {
        match self {
            DynamicValue::Optional(Some(value)) => Some(Some(value.as_ref())),
            DynamicValue::Optional(None) => Some(None),
            _ => None,
        }
    }

    /// Try to extract as an enum reference.
    pub fn as_enum(&self) -> Option<&EnumValue> {
        match self {
            DynamicValue::Enum(value) => Some(value),
            _ => None,
        }
    }

    /// Try to extract as an array reference.
    pub fn as_array(&self) -> Option<&[DynamicValue]> {
        match self {
            DynamicValue::Array(v) => Some(v),
            _ => None,
        }
    }

    /// Try to extract as a mutable array reference.
    pub fn as_array_mut(&mut self) -> Option<&mut Vec<DynamicValue>> {
        match self {
            DynamicValue::Array(v) => Some(v),
            _ => None,
        }
    }

    /// Check if this value is a primitive type.
    pub fn is_primitive(&self) -> bool {
        matches!(
            self,
            DynamicValue::Bool(_)
                | DynamicValue::Int8(_)
                | DynamicValue::Int16(_)
                | DynamicValue::Int32(_)
                | DynamicValue::Int64(_)
                | DynamicValue::Uint8(_)
                | DynamicValue::Uint16(_)
                | DynamicValue::Uint32(_)
                | DynamicValue::Uint64(_)
                | DynamicValue::Float32(_)
                | DynamicValue::Float64(_)
                | DynamicValue::String(_)
        )
    }
}

/// Trait for types that can be converted to DynamicValue.
pub trait IntoDynamic {
    fn into_dynamic(self) -> DynamicValue;
}

/// Trait for types that can be extracted from DynamicValue.
pub trait FromDynamic: Sized {
    fn from_dynamic(value: &DynamicValue) -> Option<Self>;
}

/// Macro to implement IntoDynamic and FromDynamic for primitive types.
macro_rules! impl_dynamic_conversions {
    ($($ty:ty => $variant:ident, $accessor:ident);* $(;)?) => {
        $(
            impl IntoDynamic for $ty {
                fn into_dynamic(self) -> DynamicValue {
                    DynamicValue::$variant(self)
                }
            }

            impl FromDynamic for $ty {
                fn from_dynamic(v: &DynamicValue) -> Option<Self> {
                    v.$accessor()
                }
            }
        )*
    };
}

impl_dynamic_conversions! {
    bool => Bool, as_bool;
    i8 => Int8, as_i8;
    i16 => Int16, as_i16;
    i32 => Int32, as_i32;
    i64 => Int64, as_i64;
    u8 => Uint8, as_u8;
    u16 => Uint16, as_u16;
    u32 => Uint32, as_u32;
    u64 => Uint64, as_u64;
    f32 => Float32, as_f32;
    f64 => Float64, as_f64;
}

impl IntoDynamic for String {
    fn into_dynamic(self) -> DynamicValue {
        DynamicValue::String(self)
    }
}

impl IntoDynamic for &str {
    fn into_dynamic(self) -> DynamicValue {
        DynamicValue::String(self.to_string())
    }
}

impl FromDynamic for String {
    fn from_dynamic(v: &DynamicValue) -> Option<Self> {
        v.as_str().map(|s| s.to_string())
    }
}

// Note: Vec<u8> uses DynamicValue::Bytes via the special-cased serialization,
// but for generic Vec<T> we use DynamicValue::Array. The Bytes variant is for
// optimized byte array handling in serialization.

impl IntoDynamic for DynamicMessage {
    fn into_dynamic(self) -> DynamicValue {
        DynamicValue::Message(Box::new(self))
    }
}

impl<T: IntoDynamic> IntoDynamic for Vec<T> {
    fn into_dynamic(self) -> DynamicValue {
        DynamicValue::Array(self.into_iter().map(|v| v.into_dynamic()).collect())
    }
}

impl<T: IntoDynamic> IntoDynamic for Option<T> {
    fn into_dynamic(self) -> DynamicValue {
        DynamicValue::Optional(self.map(|value| Box::new(value.into_dynamic())))
    }
}

impl<T: FromDynamic> FromDynamic for Option<T> {
    fn from_dynamic(value: &DynamicValue) -> Option<Self> {
        match value {
            DynamicValue::Optional(None) => Some(None),
            DynamicValue::Optional(Some(inner)) => T::from_dynamic(inner.as_ref()).map(Some),
            _ => None,
        }
    }
}

impl IntoDynamic for EnumValue {
    fn into_dynamic(self) -> DynamicValue {
        DynamicValue::Enum(self)
    }
}

/// Create the default value for a given field type.
pub fn default_for_type(field_type: &FieldType) -> DynamicValue {
    match field_type {
        FieldType::Bool => DynamicValue::Bool(false),
        FieldType::Int8 => DynamicValue::Int8(0),
        FieldType::Int16 => DynamicValue::Int16(0),
        FieldType::Int32 => DynamicValue::Int32(0),
        FieldType::Int64 => DynamicValue::Int64(0),
        FieldType::Uint8 => DynamicValue::Uint8(0),
        FieldType::Uint16 => DynamicValue::Uint16(0),
        FieldType::Uint32 => DynamicValue::Uint32(0),
        FieldType::Uint64 => DynamicValue::Uint64(0),
        FieldType::Float32 => DynamicValue::Float32(0.0),
        FieldType::Float64 => DynamicValue::Float64(0.0),
        FieldType::String | FieldType::BoundedString(_) => DynamicValue::String(String::new()),
        FieldType::Message(schema) => DynamicValue::Message(Box::new(DynamicMessage::new(schema))),
        FieldType::Optional(_) => DynamicValue::Optional(None),
        FieldType::Enum(schema) => DynamicValue::Enum(default_enum_value(schema)),
        FieldType::Array(inner, len) => DynamicValue::Array(vec![default_for_type(inner); *len]),
        FieldType::Sequence(_) | FieldType::BoundedSequence(_, _) => {
            DynamicValue::Array(Vec::new())
        }
    }
}

fn default_enum_value(schema: &Arc<EnumSchema>) -> EnumValue {
    let variant = schema
        .variants
        .first()
        .expect("enum schemas must have at least one variant");

    EnumValue {
        variant_index: 0,
        variant_name: variant.name.clone(),
        payload: default_enum_payload(&variant.payload),
    }
}

fn default_enum_payload(payload: &EnumPayloadSchema) -> EnumPayloadValue {
    match payload {
        EnumPayloadSchema::Unit => EnumPayloadValue::Unit,
        EnumPayloadSchema::Newtype(field_type) => {
            EnumPayloadValue::Newtype(Box::new(default_for_type(field_type)))
        }
        EnumPayloadSchema::Tuple(field_types) => {
            EnumPayloadValue::Tuple(field_types.iter().map(default_for_type).collect())
        }
        EnumPayloadSchema::Struct(fields) => EnumPayloadValue::Struct(
            fields
                .iter()
                .map(|field| DynamicNamedValue {
                    name: field.name.clone(),
                    value: default_for_type(&field.field_type),
                })
                .collect(),
        ),
    }
}
