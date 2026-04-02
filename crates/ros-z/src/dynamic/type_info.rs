use std::sync::Arc;

use tracing::warn;

use crate::{
    dynamic::{MessageSchema, MessageSchemaTypeDescription},
    entity::{TypeHash, TypeInfo},
};

#[derive(Debug, Clone)]
pub struct DiscoveredTopicSchema {
    pub qualified_topic: String,
    pub schema: Arc<MessageSchema>,
    pub type_hash: String,
}

pub(crate) fn dds_type_name_from_schema(schema: &MessageSchema) -> String {
    schema
        .type_name
        .replace("/msg/", "::msg::dds_::")
        .replace("/srv/", "::srv::dds_::")
        .replace("/action/", "::action::dds_::")
        + "_"
}

pub(crate) fn schema_hash(schema: &MessageSchema) -> TypeHash {
    match schema.compute_type_hash() {
        Ok(hash) => {
            let rihs_string = hash.to_rihs_string();
            TypeHash::from_rihs_string(&rihs_string).unwrap_or_else(TypeHash::zero)
        }
        Err(error) => {
            warn!(
                "[NOD] Failed to compute type hash for {}: {}",
                schema.type_name, error
            );
            TypeHash::zero()
        }
    }
}

pub(crate) fn schema_type_info(schema: &MessageSchema) -> TypeInfo {
    TypeInfo {
        name: dds_type_name_from_schema(schema),
        hash: schema_hash(schema),
    }
}

pub(crate) fn schema_type_info_with_hash(
    schema: &MessageSchema,
    discovered_hash: &str,
) -> TypeInfo {
    TypeInfo {
        name: dds_type_name_from_schema(schema),
        hash: TypeHash::from_rihs_string(discovered_hash).unwrap_or_else(TypeHash::zero),
    }
}
