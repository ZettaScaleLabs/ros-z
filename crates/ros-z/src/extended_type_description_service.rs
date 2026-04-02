use std::sync::{Arc, RwLock};

use serde::{Deserialize, Serialize};
use tracing::{debug, info, warn};
use zenoh::Session;

use crate::ServiceTypeInfo;
use crate::attachment::Attachment;
use crate::context::GlobalCounter;
use crate::dynamic::{DynamicError, MessageSchema};
use crate::entity::{EndpointEntity, EndpointKind, NodeEntity, TypeHash, TypeInfo};
use crate::msg::{SerdeCdrSerdes, ZMessage, ZService};
use crate::service::{ZServer, ZServerBuilder};

use crate::extended_schema::{compute_extended_type_hash, schema_to_extension_json};

const EXTENDED_SERVICE_NAME: &str = "~get_extended_type_description";

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetExtendedTypeDescriptionRequest {
    pub type_name: String,
    pub type_hash: String,
}

impl ZMessage for GetExtendedTypeDescriptionRequest {
    type Serdes = SerdeCdrSerdes<Self>;
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetExtendedTypeDescriptionResponse {
    pub successful: bool,
    pub failure_reason: String,
    pub type_hash: String,
    pub schema_json: String,
}

impl ZMessage for GetExtendedTypeDescriptionResponse {
    type Serdes = SerdeCdrSerdes<Self>;
}

pub struct GetExtendedTypeDescription;

impl ZService for GetExtendedTypeDescription {
    type Request = GetExtendedTypeDescriptionRequest;
    type Response = GetExtendedTypeDescriptionResponse;
}

impl ServiceTypeInfo for GetExtendedTypeDescription {
    fn service_type_info() -> TypeInfo {
        TypeInfo::new(
            "ros_z::srv::dds_::GetExtendedTypeDescription_",
            TypeHash::zero(),
        )
    }
}

#[derive(Clone)]
pub struct RegisteredExtendedSchema {
    pub schema: Arc<MessageSchema>,
    pub type_hash: String,
}

impl RegisteredExtendedSchema {
    pub fn new(schema: Arc<MessageSchema>) -> Result<Self, DynamicError> {
        let type_hash = compute_extended_type_hash(&schema)?.to_rihs_string();
        Ok(Self { schema, type_hash })
    }
}

#[derive(Clone)]
pub struct ExtendedTypeDescriptionService {
    schemas: Arc<RwLock<std::collections::HashMap<String, RegisteredExtendedSchema>>>,
    _server: Arc<ZServer<GetExtendedTypeDescription, ()>>,
}

impl ExtendedTypeDescriptionService {
    pub fn new(
        session: Arc<Session>,
        node_name: &str,
        namespace: &str,
        node_id: usize,
        counter: &GlobalCounter,
        clock: &crate::time::ZClock,
    ) -> zenoh::Result<Self> {
        let schemas = Arc::new(RwLock::new(std::collections::HashMap::new()));
        let node_entity = NodeEntity::new(
            0,
            session.zid(),
            node_id,
            node_name.to_string(),
            namespace.to_string(),
            String::new(),
        );

        let entity = EndpointEntity {
            id: counter.increment(),
            node: Some(node_entity),
            kind: EndpointKind::Service,
            topic: EXTENDED_SERVICE_NAME.to_string(),
            type_info: Some(GetExtendedTypeDescription::service_type_info()),
            qos: Default::default(),
        };

        let server_builder: ZServerBuilder<GetExtendedTypeDescription> = ZServerBuilder {
            entity,
            session,
            clock: clock.clone(),
            keyexpr_format: ros_z_protocol::KeyExprFormat::default(),
            _phantom_data: Default::default(),
        };

        let schemas_clone = schemas.clone();
        let server = server_builder.build_with_callback(move |query| {
            Self::handle_query(&schemas_clone, &query);
        })?;

        info!(
            "[ETDS] ExtendedTypeDescriptionService created for node: {}/{}",
            namespace, node_name
        );

        Ok(Self {
            schemas,
            _server: Arc::new(server),
        })
    }

    pub fn register_schema(&self, schema: Arc<MessageSchema>) -> Result<(), DynamicError> {
        let registered = RegisteredExtendedSchema::new(schema.clone())?;
        let type_name = schema.type_name.clone();

        let mut schemas = self
            .schemas
            .write()
            .map_err(|_| DynamicError::RegistryLockPoisoned)?;
        schemas.insert(type_name.clone(), registered);

        debug!("[ETDS] Registered extended schema: {}", type_name);
        Ok(())
    }

    pub fn get_schema(&self, type_name: &str) -> Result<Option<Arc<MessageSchema>>, DynamicError> {
        let schemas = self
            .schemas
            .read()
            .map_err(|_| DynamicError::RegistryLockPoisoned)?;
        Ok(schemas
            .get(type_name)
            .map(|registered| registered.schema.clone()))
    }

    fn handle_query(
        schemas: &Arc<RwLock<std::collections::HashMap<String, RegisteredExtendedSchema>>>,
        query: &zenoh::query::Query,
    ) {
        let request = match query.payload() {
            Some(payload) => match <GetExtendedTypeDescriptionRequest as ZMessage>::deserialize(
                payload.to_bytes().as_ref(),
            ) {
                Ok(request) => request,
                Err(err) => {
                    warn!("[ETDS] Failed to decode request: {}", err);
                    return;
                }
            },
            None => {
                warn!("[ETDS] Missing request payload");
                return;
            }
        };

        let response = Self::build_response(schemas, &request);
        let bytes = ZMessage::serialize(&response);
        use zenoh::Wait;
        let mut reply = query.reply(query.key_expr().clone(), bytes);
        if let Some(att_bytes) = query.attachment()
            && let Ok(att) = Attachment::try_from(att_bytes)
        {
            reply = reply.attachment(att);
        }
        if let Err(err) = reply.wait() {
            warn!("[ETDS] Failed to send response: {}", err);
        }
    }

    fn build_response(
        schemas: &Arc<RwLock<std::collections::HashMap<String, RegisteredExtendedSchema>>>,
        request: &GetExtendedTypeDescriptionRequest,
    ) -> GetExtendedTypeDescriptionResponse {
        let schemas_guard = match schemas.read() {
            Ok(guard) => guard,
            Err(_) => {
                return GetExtendedTypeDescriptionResponse {
                    successful: false,
                    failure_reason: "Internal error: registry lock poisoned".to_string(),
                    type_hash: String::new(),
                    schema_json: String::new(),
                };
            }
        };

        let registered = match schemas_guard.get(&request.type_name) {
            Some(schema) => schema,
            None => {
                return GetExtendedTypeDescriptionResponse {
                    successful: false,
                    failure_reason: format!("Type '{}' not registered", request.type_name),
                    type_hash: String::new(),
                    schema_json: String::new(),
                };
            }
        };

        if !request.type_hash.is_empty() && request.type_hash != registered.type_hash {
            return GetExtendedTypeDescriptionResponse {
                successful: false,
                failure_reason: format!(
                    "Type hash mismatch: expected {}, got {}",
                    registered.type_hash, request.type_hash
                ),
                type_hash: String::new(),
                schema_json: String::new(),
            };
        }

        match schema_to_extension_json(&registered.schema) {
            Ok(schema_json) => GetExtendedTypeDescriptionResponse {
                successful: true,
                failure_reason: String::new(),
                type_hash: registered.type_hash.clone(),
                schema_json,
            },
            Err(err) => GetExtendedTypeDescriptionResponse {
                successful: false,
                failure_reason: format!("Failed to serialize extended schema: {}", err),
                type_hash: String::new(),
                schema_json: String::new(),
            },
        }
    }
}
