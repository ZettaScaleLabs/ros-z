use std::sync::Arc;

use serde::{Deserialize, Serialize};
use sha2::Digest;

use crate::dynamic::{DynamicError, FieldType, MessageSchema};
use crate::node::ZNode;

pub trait ExtendedMessageTypeInfo: crate::MessageTypeInfo {
    fn extended_message_schema() -> Arc<MessageSchema>;

    fn extended_field_type() -> FieldType {
        FieldType::Message(Self::extended_message_schema())
    }
}

#[derive(Serialize, Deserialize)]
struct ExtendedMessageSchema {
    type_name: String,
    package: String,
    name: String,
    fields: Vec<ExtendedFieldSchema>,
}

#[derive(Serialize, Deserialize)]
struct ExtendedFieldSchema {
    name: String,
    field_type: ExtendedFieldType,
}

#[derive(Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum ExtendedFieldType {
    Bool,
    Int8,
    Int16,
    Int32,
    Int64,
    Uint8,
    Uint16,
    Uint32,
    Uint64,
    Float32,
    Float64,
    String,
    BoundedString {
        capacity: usize,
    },
    Message {
        schema: Box<ExtendedMessageSchema>,
    },
    Optional {
        inner: Box<ExtendedFieldType>,
    },
    Enum {
        schema: Box<ExtendedEnumSchema>,
    },
    Array {
        inner: Box<ExtendedFieldType>,
        len: usize,
    },
    Sequence {
        inner: Box<ExtendedFieldType>,
    },
    BoundedSequence {
        inner: Box<ExtendedFieldType>,
        max: usize,
    },
}

#[derive(Serialize, Deserialize)]
struct ExtendedEnumSchema {
    type_name: String,
    variants: Vec<ExtendedEnumVariantSchema>,
}

#[derive(Serialize, Deserialize)]
struct ExtendedEnumVariantSchema {
    name: String,
    payload: ExtendedEnumPayloadSchema,
}

#[derive(Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum ExtendedEnumPayloadSchema {
    Unit,
    Newtype { field_type: Box<ExtendedFieldType> },
    Tuple { field_types: Vec<ExtendedFieldType> },
    Struct { fields: Vec<ExtendedFieldSchema> },
}

pub fn compute_extended_type_hash(
    schema: &MessageSchema,
) -> Result<ros_z_schema::TypeHash, DynamicError> {
    let extended = message_schema_to_extended(schema);
    let json = ros_z_schema::to_ros2_json(&extended).map_err(|err| {
        DynamicError::SerializationError(format!(
            "failed to serialize extended schema hash view for {}: {}",
            schema.type_name, err
        ))
    })?;

    let mut hasher = sha2::Sha256::new();
    hasher.update(json.as_bytes());
    Ok(ros_z_schema::TypeHash(hasher.finalize().into()))
}

pub fn schema_to_extension_json(schema: &MessageSchema) -> Result<String, DynamicError> {
    let extended = message_schema_to_extended(schema);
    serde_json::to_string(&extended).map_err(|err| {
        DynamicError::SerializationError(format!(
            "failed to serialize extended schema for {}: {}",
            schema.type_name, err
        ))
    })
}

pub fn schema_from_extension_json(json: &str) -> Result<Arc<MessageSchema>, DynamicError> {
    let extended: ExtendedMessageSchema = serde_json::from_str(json).map_err(|err| {
        DynamicError::DeserializationError(format!("failed to parse extended schema JSON: {}", err))
    })?;
    Ok(Arc::new(extended_to_message_schema(extended)))
}

pub fn register_type<T: ExtendedMessageTypeInfo>(node: &ZNode) -> Result<(), String> {
    let schema = T::extended_message_schema();
    if !schema.uses_extended_types() {
        return Ok(());
    }

    let service = node.extended_type_description_service().ok_or_else(|| {
        "extended type description service is not enabled on this node; call with_extended_type_description_service() to expose schemas for extended-only types".to_string()
    })?;

    service
        .register_schema(schema)
        .map_err(|err| format!("failed to register extended schema: {}", err))
}

fn message_schema_to_extended(schema: &MessageSchema) -> ExtendedMessageSchema {
    ExtendedMessageSchema {
        type_name: schema.type_name.clone(),
        package: schema.package.clone(),
        name: schema.name.clone(),
        fields: schema.fields.iter().map(field_schema_to_extended).collect(),
    }
}

fn field_schema_to_extended(field: &crate::dynamic::FieldSchema) -> ExtendedFieldSchema {
    ExtendedFieldSchema {
        name: field.name.clone(),
        field_type: field_type_to_extended(&field.field_type),
    }
}

fn field_type_to_extended(field_type: &FieldType) -> ExtendedFieldType {
    match field_type {
        FieldType::Bool => ExtendedFieldType::Bool,
        FieldType::Int8 => ExtendedFieldType::Int8,
        FieldType::Int16 => ExtendedFieldType::Int16,
        FieldType::Int32 => ExtendedFieldType::Int32,
        FieldType::Int64 => ExtendedFieldType::Int64,
        FieldType::Uint8 => ExtendedFieldType::Uint8,
        FieldType::Uint16 => ExtendedFieldType::Uint16,
        FieldType::Uint32 => ExtendedFieldType::Uint32,
        FieldType::Uint64 => ExtendedFieldType::Uint64,
        FieldType::Float32 => ExtendedFieldType::Float32,
        FieldType::Float64 => ExtendedFieldType::Float64,
        FieldType::String => ExtendedFieldType::String,
        FieldType::BoundedString(capacity) => ExtendedFieldType::BoundedString {
            capacity: *capacity,
        },
        FieldType::Message(schema) => ExtendedFieldType::Message {
            schema: Box::new(message_schema_to_extended(schema)),
        },
        FieldType::Optional(inner) => ExtendedFieldType::Optional {
            inner: Box::new(field_type_to_extended(inner)),
        },
        FieldType::Enum(schema) => ExtendedFieldType::Enum {
            schema: Box::new(enum_schema_to_extended(schema)),
        },
        FieldType::Array(inner, len) => ExtendedFieldType::Array {
            inner: Box::new(field_type_to_extended(inner)),
            len: *len,
        },
        FieldType::Sequence(inner) => ExtendedFieldType::Sequence {
            inner: Box::new(field_type_to_extended(inner)),
        },
        FieldType::BoundedSequence(inner, max) => ExtendedFieldType::BoundedSequence {
            inner: Box::new(field_type_to_extended(inner)),
            max: *max,
        },
    }
}

fn enum_schema_to_extended(schema: &crate::dynamic::EnumSchema) -> ExtendedEnumSchema {
    ExtendedEnumSchema {
        type_name: schema.type_name.clone(),
        variants: schema
            .variants
            .iter()
            .map(enum_variant_to_extended)
            .collect(),
    }
}

fn enum_variant_to_extended(
    variant: &crate::dynamic::EnumVariantSchema,
) -> ExtendedEnumVariantSchema {
    ExtendedEnumVariantSchema {
        name: variant.name.clone(),
        payload: enum_payload_to_extended(&variant.payload),
    }
}

fn enum_payload_to_extended(
    payload: &crate::dynamic::EnumPayloadSchema,
) -> ExtendedEnumPayloadSchema {
    match payload {
        crate::dynamic::EnumPayloadSchema::Unit => ExtendedEnumPayloadSchema::Unit,
        crate::dynamic::EnumPayloadSchema::Newtype(field_type) => {
            ExtendedEnumPayloadSchema::Newtype {
                field_type: Box::new(field_type_to_extended(field_type)),
            }
        }
        crate::dynamic::EnumPayloadSchema::Tuple(field_types) => ExtendedEnumPayloadSchema::Tuple {
            field_types: field_types.iter().map(field_type_to_extended).collect(),
        },
        crate::dynamic::EnumPayloadSchema::Struct(fields) => ExtendedEnumPayloadSchema::Struct {
            fields: fields.iter().map(field_schema_to_extended).collect(),
        },
    }
}

fn extended_to_message_schema(schema: ExtendedMessageSchema) -> MessageSchema {
    MessageSchema {
        type_name: schema.type_name,
        package: schema.package,
        name: schema.name,
        fields: schema
            .fields
            .into_iter()
            .map(extended_to_field_schema)
            .collect(),
        type_hash: None,
    }
}

fn extended_to_field_schema(field: ExtendedFieldSchema) -> crate::dynamic::FieldSchema {
    crate::dynamic::FieldSchema {
        name: field.name,
        field_type: extended_to_field_type(field.field_type),
        default_value: None,
    }
}

fn extended_to_field_type(field_type: ExtendedFieldType) -> FieldType {
    match field_type {
        ExtendedFieldType::Bool => FieldType::Bool,
        ExtendedFieldType::Int8 => FieldType::Int8,
        ExtendedFieldType::Int16 => FieldType::Int16,
        ExtendedFieldType::Int32 => FieldType::Int32,
        ExtendedFieldType::Int64 => FieldType::Int64,
        ExtendedFieldType::Uint8 => FieldType::Uint8,
        ExtendedFieldType::Uint16 => FieldType::Uint16,
        ExtendedFieldType::Uint32 => FieldType::Uint32,
        ExtendedFieldType::Uint64 => FieldType::Uint64,
        ExtendedFieldType::Float32 => FieldType::Float32,
        ExtendedFieldType::Float64 => FieldType::Float64,
        ExtendedFieldType::String => FieldType::String,
        ExtendedFieldType::BoundedString { capacity } => FieldType::BoundedString(capacity),
        ExtendedFieldType::Message { schema } => {
            FieldType::Message(Arc::new(extended_to_message_schema(*schema)))
        }
        ExtendedFieldType::Optional { inner } => {
            FieldType::Optional(Box::new(extended_to_field_type(*inner)))
        }
        ExtendedFieldType::Enum { schema } => {
            FieldType::Enum(Arc::new(extended_to_enum_schema(*schema)))
        }
        ExtendedFieldType::Array { inner, len } => {
            FieldType::Array(Box::new(extended_to_field_type(*inner)), len)
        }
        ExtendedFieldType::Sequence { inner } => {
            FieldType::Sequence(Box::new(extended_to_field_type(*inner)))
        }
        ExtendedFieldType::BoundedSequence { inner, max } => {
            FieldType::BoundedSequence(Box::new(extended_to_field_type(*inner)), max)
        }
    }
}

fn extended_to_enum_schema(schema: ExtendedEnumSchema) -> crate::dynamic::EnumSchema {
    crate::dynamic::EnumSchema {
        type_name: schema.type_name,
        variants: schema
            .variants
            .into_iter()
            .map(extended_to_enum_variant)
            .collect(),
    }
}

fn extended_to_enum_variant(
    variant: ExtendedEnumVariantSchema,
) -> crate::dynamic::EnumVariantSchema {
    crate::dynamic::EnumVariantSchema {
        name: variant.name,
        payload: extended_to_enum_payload(variant.payload),
    }
}

fn extended_to_enum_payload(
    payload: ExtendedEnumPayloadSchema,
) -> crate::dynamic::EnumPayloadSchema {
    match payload {
        ExtendedEnumPayloadSchema::Unit => crate::dynamic::EnumPayloadSchema::Unit,
        ExtendedEnumPayloadSchema::Newtype { field_type } => {
            crate::dynamic::EnumPayloadSchema::Newtype(Box::new(extended_to_field_type(
                *field_type,
            )))
        }
        ExtendedEnumPayloadSchema::Tuple { field_types } => {
            crate::dynamic::EnumPayloadSchema::Tuple(
                field_types
                    .into_iter()
                    .map(extended_to_field_type)
                    .collect(),
            )
        }
        ExtendedEnumPayloadSchema::Struct { fields } => crate::dynamic::EnumPayloadSchema::Struct(
            fields.into_iter().map(extended_to_field_schema).collect(),
        ),
    }
}
