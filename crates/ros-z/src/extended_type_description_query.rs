use std::{sync::Arc, time::Duration};

use crate::{Builder, dynamic::DynamicError, node::ZNode};

use crate::dynamic::{MessageSchema, discovery::TopicSchemaCandidate};
use crate::extended_type_description_service::{
    GetExtendedTypeDescription, GetExtendedTypeDescriptionRequest,
    GetExtendedTypeDescriptionResponse,
};

use crate::extended_schema::schema_from_extension_json;

/// Query the extended type-description service for a single current topic candidate.
pub(crate) async fn query_extended_type_description(
    node: &ZNode,
    candidate: &TopicSchemaCandidate,
    timeout: Duration,
) -> Result<(Arc<MessageSchema>, String), DynamicError> {
    let service_name = if candidate.namespace.is_empty() || candidate.namespace == "/" {
        format!("/{}/get_extended_type_description", candidate.node_name)
    } else {
        format!(
            "{}/{}/get_extended_type_description",
            candidate.namespace, candidate.node_name
        )
    };

    let client = node
        .create_client::<GetExtendedTypeDescription>(&service_name)
        .build()
        .map_err(|e| DynamicError::SerializationError(e.to_string()))?;
    let request = GetExtendedTypeDescriptionRequest {
        type_name: candidate.type_name.clone(),
        type_hash: candidate.type_hash.clone(),
    };

    let response = client
        .call_or_timeout(&request, timeout)
        .await
        .map_err(|_| {
            DynamicError::SerializationError(
                "extended type description service timed out".to_string(),
            )
        })?;

    let schema = schema_from_extended_type_description_response(&response)?;
    Ok((schema, response.type_hash))
}

pub fn schema_from_extended_type_description_response(
    response: &GetExtendedTypeDescriptionResponse,
) -> Result<Arc<MessageSchema>, DynamicError> {
    if !response.successful {
        return Err(DynamicError::SerializationError(format!(
            "Response indicates failure: {}",
            response.failure_reason
        )));
    }

    schema_from_extension_json(&response.schema_json)
}
