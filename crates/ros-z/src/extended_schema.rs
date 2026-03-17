use std::sync::{Arc, RwLock};
use std::time::Duration;

use serde::{Deserialize, Serialize};
use sha2::Digest;
use tracing::{debug, info, warn};
use zenoh::Session;

use crate::context::GlobalCounter;
use crate::dynamic::{DynamicError, FieldType, MessageSchema};
use crate::entity::{EndpointEntity, Entity, EntityKind, NodeEntity, TypeHash, TypeInfo};
use crate::graph::Graph;
use crate::msg::{SerdeCdrSerdes, ZMessage, ZService};
use crate::node::ZNode;
use crate::service::{ZClient, ZClientBuilder, ZServer, ZServerBuilder};
use crate::{Builder, ServiceTypeInfo};

const EXTENDED_SERVICE_NAME: &str = "~get_extended_type_description";

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
            node: node_entity,
            kind: EntityKind::Service,
            topic: EXTENDED_SERVICE_NAME.to_string(),
            type_info: Some(GetExtendedTypeDescription::service_type_info()),
            ..Default::default()
        };

        let server_builder: ZServerBuilder<GetExtendedTypeDescription> = ZServerBuilder {
            entity,
            session,
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
        if let Err(err) = query.reply(query.key_expr().clone(), bytes).wait() {
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

pub struct ExtendedTypeDescriptionClient {
    session: Arc<Session>,
    counter: Arc<GlobalCounter>,
    graph: Option<Arc<Graph>>,
    timeout: Duration,
}

impl ExtendedTypeDescriptionClient {
    pub fn new(session: Arc<Session>, counter: Arc<GlobalCounter>) -> Self {
        Self {
            session,
            counter,
            graph: None,
            timeout: Duration::from_secs(10),
        }
    }

    pub fn with_graph(
        session: Arc<Session>,
        counter: Arc<GlobalCounter>,
        graph: Arc<Graph>,
    ) -> Self {
        Self {
            session,
            counter,
            graph: Some(graph),
            timeout: Duration::from_secs(10),
        }
    }

    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    pub async fn get_type_description(
        &self,
        node_name: &str,
        namespace: &str,
        type_name: &str,
        type_hash: &str,
    ) -> Result<GetExtendedTypeDescriptionResponse, DynamicError> {
        let service_name = if namespace.is_empty() || namespace == "/" {
            format!("/{}/get_extended_type_description", node_name)
        } else {
            format!("{}/{}/get_extended_type_description", namespace, node_name)
        };

        let client = self.create_client(&service_name, "")?;
        self.query_with_client(&client, type_name, type_hash).await
    }

    async fn query_with_client(
        &self,
        client: &ZClient<GetExtendedTypeDescription>,
        type_name: &str,
        type_hash: &str,
    ) -> Result<GetExtendedTypeDescriptionResponse, DynamicError> {
        let request = GetExtendedTypeDescriptionRequest {
            type_name: type_name.to_string(),
            type_hash: type_hash.to_string(),
        };

        client
            .send_request(&request)
            .await
            .map_err(|e| DynamicError::SerializationError(e.to_string()))?;

        client.take_response_timeout(self.timeout).map_err(|_| {
            DynamicError::SerializationError(
                "extended type description service timed out".to_string(),
            )
        })
    }

    pub async fn get_type_description_for_topic(
        &self,
        topic: &str,
        timeout: Duration,
    ) -> Result<(Arc<MessageSchema>, String), DynamicError> {
        let graph = self.graph.as_ref().ok_or_else(|| {
            DynamicError::SerializationError(
                "ExtendedTypeDescriptionClient requires graph for topic-based discovery"
                    .to_string(),
            )
        })?;

        let start = tokio::time::Instant::now();
        let mut last_error = None;

        while start.elapsed() < timeout {
            let publishers = graph.get_entities_by_topic(EntityKind::Publisher, topic);
            if publishers.is_empty() {
                tokio::time::sleep(Duration::from_millis(250)).await;
                continue;
            }

            for publisher in &publishers {
                let Entity::Endpoint(endpoint) = &**publisher else {
                    continue;
                };

                let type_info = match &endpoint.type_info {
                    Some(info) => info,
                    None => continue,
                };
                let type_name = normalize_type_name(&type_info.name);
                let type_hash = type_info.hash.to_rihs_string();

                match self
                    .get_type_description(
                        &endpoint.node.name,
                        &endpoint.node.namespace,
                        &type_name,
                        &type_hash,
                    )
                    .await
                {
                    Ok(response) => {
                        let schema = Self::response_to_schema(&response)?;
                        return Ok((schema, response.type_hash));
                    }
                    Err(err) => last_error = Some(err),
                }
            }

            tokio::time::sleep(Duration::from_millis(250)).await;
        }

        Err(last_error.unwrap_or_else(|| {
            DynamicError::SchemaNotFound(format!(
                "Failed to get extended type description from any publisher on {}",
                topic
            ))
        }))
    }

    pub fn response_to_schema(
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

    fn create_client(
        &self,
        service_name: &str,
        namespace: &str,
    ) -> Result<ZClient<GetExtendedTypeDescription>, DynamicError> {
        let node_entity = NodeEntity::new(
            0,
            self.session.zid(),
            self.counter.increment(),
            "extended_type_desc_client".to_string(),
            namespace.to_string(),
            String::new(),
        );

        let entity = EndpointEntity {
            id: self.counter.increment(),
            node: node_entity,
            kind: EntityKind::Client,
            topic: service_name.to_string(),
            type_info: Some(GetExtendedTypeDescription::service_type_info()),
            ..Default::default()
        };

        let builder: ZClientBuilder<GetExtendedTypeDescription> = ZClientBuilder {
            entity,
            session: self.session.clone(),
            keyexpr_format: ros_z_protocol::KeyExprFormat::default(),
            _phantom_data: Default::default(),
        };

        builder
            .build()
            .map_err(|e| DynamicError::SerializationError(e.to_string()))
    }
}

fn normalize_type_name(dds_name: &str) -> String {
    dds_name
        .replace("::msg::dds_::", "/msg/")
        .replace("::srv::dds_::", "/srv/")
        .replace("::action::dds_::", "/action/")
        .trim_end_matches('_')
        .to_string()
}
