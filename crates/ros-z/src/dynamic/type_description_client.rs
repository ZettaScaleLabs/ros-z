//! Type Description Client implementation.
//!
//! This module provides the `TypeDescriptionClient` for querying type descriptions
//! from remote nodes. It enables dynamic schema discovery by calling the
//! `GetTypeDescription` service on publisher nodes.
//!
//! # Architecture
//!
//! ```text
//! ┌──────────────────────────────────────────────────────────────┐
//! │                    TypeDescriptionClient                      │
//! ├──────────────────────────────────────────────────────────────┤
//! │  Methods:                                                     │
//! │  ├── get_type_description(node, type_name) -> Response       │
//! │  ├── get_type_description_for_topic(topic) -> Schema         │
//! │  └── response_to_schema(response) -> MessageSchema           │
//! └──────────────────────────────────────────────────────────────┘
//!                         │
//!                         ▼
//! ┌──────────────────────────────────────────────────────────────┐
//! │          ZClient<GetTypeDescription>                          │
//! │          (Zenoh Querier)                                      │
//! └──────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Example
//!
//! ```rust,ignore
//! use ros_z::dynamic::TypeDescriptionClient;
//!
//! // Create client
//! let client = TypeDescriptionClient::new(node);
//!
//! // Query type description from a specific node
//! let response = client.get_type_description(
//!     "talker",           // node name
//!     "",                 // namespace (root)
//!     "std_msgs/msg/String",
//!     "",                 // empty hash = no validation
//!     false,              // don't include sources
//! ).await?;
//!
//! // Convert to schema
//! let schema = TypeDescriptionClient::response_to_schema(&response)?;
//! ```

use std::sync::Arc;
use std::time::Duration;

use tracing::{debug, warn};
use zenoh::Session;

use crate::context::GlobalCounter;
use crate::entity::{EndpointEntity, Entity, EntityKind, NodeEntity};
use crate::graph::Graph;
use crate::service::{ZClient, ZClientBuilder};
use crate::{Builder, ServiceTypeInfo};

use super::error::DynamicError;
use super::schema::MessageSchema;

/// Normalize DDS type name to ROS 2 canonical format.
///
/// Converts "std_msgs::msg::dds_::String_" to "std_msgs/msg/String"
fn normalize_type_name(dds_name: &str) -> String {
    dds_name
        .replace("::msg::dds_::", "/msg/")
        .replace("::srv::dds_::", "/srv/")
        .replace("::action::dds_::", "/action/")
        .trim_end_matches('_')
        .to_string()
}
use super::type_description::type_description_msg_to_schema;
use super::type_description_service::{
    GetTypeDescription, GetTypeDescriptionRequest, GetTypeDescriptionResponse,
    wire_to_schema_type_description,
};

/// Client for querying type descriptions from remote nodes.
///
/// This client enables dynamic schema discovery by:
/// - Querying specific nodes for their registered type descriptions
/// - Discovering publishers on a topic and querying their type descriptions
/// - Converting wire format responses to usable `MessageSchema` objects
pub struct TypeDescriptionClient {
    session: Arc<Session>,
    counter: Arc<GlobalCounter>,
    graph: Option<Arc<Graph>>,
    /// Default timeout for service calls
    timeout: Duration,
}

impl TypeDescriptionClient {
    /// Create a new TypeDescriptionClient.
    ///
    /// # Arguments
    ///
    /// * `session` - The Zenoh session to use
    /// * `counter` - Global counter for entity IDs
    ///
    /// # Returns
    ///
    /// A new `TypeDescriptionClient` instance.
    pub fn new(session: Arc<Session>, counter: Arc<GlobalCounter>) -> Self {
        Self {
            session,
            counter,
            graph: None,
            timeout: Duration::from_secs(10),
        }
    }

    /// Create a TypeDescriptionClient with graph access for topic-based discovery.
    ///
    /// # Arguments
    ///
    /// * `session` - The Zenoh session to use
    /// * `counter` - Global counter for entity IDs
    /// * `graph` - The graph for entity discovery
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

    /// Set the default timeout for service calls.
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// Query type description from a specific node.
    ///
    /// This method creates a service client for the target node's
    /// `get_type_description` service and queries for the specified type.
    ///
    /// # Arguments
    ///
    /// * `node_name` - Name of the target node
    /// * `namespace` - Namespace of the target node (empty string for root namespace)
    /// * `type_name` - Fully qualified type name (e.g., "std_msgs/msg/String")
    /// * `type_hash` - Expected type hash for validation (empty string to skip)
    /// * `include_sources` - Whether to include source files in the response
    ///
    /// # Returns
    ///
    /// The `GetTypeDescriptionResponse` from the remote node.
    pub async fn get_type_description(
        &self,
        node_name: &str,
        namespace: &str,
        type_name: &str,
        type_hash: &str,
        include_sources: bool,
    ) -> Result<GetTypeDescriptionResponse, DynamicError> {
        debug!(
            "[TDC] Querying type description: node={}/{}, type={}",
            namespace, node_name, type_name
        );

        // Build the absolute service name for this node.
        let service_name = if namespace.is_empty() || namespace == "/" {
            format!("/{}/get_type_description", node_name)
        } else {
            format!("{}/{}/get_type_description", namespace, node_name)
        };

        debug!(
            "[TDC] Creating client for absolute service: {}",
            service_name
        );

        // Use empty namespace since we're using an absolute service name (starts with /)
        let client = self.create_client(&service_name, "")?;

        self.query_with_client(
            &client,
            node_name,
            namespace,
            type_name,
            type_hash,
            include_sources,
        )
        .await
    }

    /// Send a type description request via an already-built client and wait for the response.
    ///
    /// Separating client creation from the actual query allows callers to hoist Querier
    /// creation out of tight loops, mirroring rmw_zenoh_cpp where the Querier lives for
    /// the lifetime of the service client rather than being freshly created per request.
    async fn query_with_client(
        &self,
        client: &ZClient<GetTypeDescription>,
        node_name: &str,
        namespace: &str,
        type_name: &str,
        type_hash: &str,
        include_sources: bool,
    ) -> Result<GetTypeDescriptionResponse, DynamicError> {
        let request = GetTypeDescriptionRequest {
            type_name: type_name.to_string(),
            type_hash: type_hash.to_string(),
            include_type_sources: include_sources,
        };

        debug!(
            "[TDC] Sending request to get_type_description: node={}/{}",
            namespace, node_name
        );

        client
            .send_request(&request)
            .await
            .map_err(|e| DynamicError::SerializationError(e.to_string()))?;

        // Map a channel/recv timeout to the dedicated ServiceTimeout variant so callers
        // can distinguish "service didn't respond" from actual CDR decode failures.
        let node_display = if namespace.is_empty() || namespace == "/" {
            node_name.to_string()
        } else {
            format!("{}/{}", namespace, node_name)
        };
        let service_display = if namespace.is_empty() || namespace == "/" {
            format!("/{}/get_type_description", node_name)
        } else {
            format!("{}/{}/get_type_description", namespace, node_name)
        };

        let response = client.take_response_timeout(self.timeout).map_err(|_| {
            DynamicError::ServiceTimeout {
                node: node_display,
                service: service_display,
            }
        })?;

        if response.successful {
            debug!(
                "[TDC] Got type description for: {}",
                response.type_description.type_description.type_name
            );
        } else {
            warn!(
                "[TDC] Type description query failed: {}",
                response.failure_reason
            );
        }

        Ok(response)
    }

    /// Query type description from any node publishing to a topic.
    ///
    /// This method uses the graph to discover publishers on the specified topic,
    /// then queries each publisher's node for the type description until one
    /// succeeds.
    ///
    /// # Arguments
    ///
    /// * `topic` - The topic to discover type information for
    /// * `timeout` - Maximum time to wait for discovery and query
    ///
    /// # Returns
    ///
    /// A tuple of (MessageSchema, type_hash) on success.
    pub async fn get_type_description_for_topic(
        &self,
        topic: &str,
        timeout: Duration,
    ) -> Result<(Arc<MessageSchema>, String), DynamicError> {
        let graph = self.graph.as_ref().ok_or_else(|| {
            DynamicError::SerializationError(
                "TypeDescriptionClient requires graph for topic-based discovery".to_string(),
            )
        })?;

        debug!("[TDC] Discovering type description for topic: {}", topic);

        // ── Phase 1: Publisher discovery ─────────────────────────────────────
        // Retry up to 5 × 500 ms if the graph hasn't seen any publishers yet.
        let mut publishers = graph.get_entities_by_topic(EntityKind::Publisher, topic);
        debug!(
            "[TDC] Initial discovery found {} publishers for topic {}",
            publishers.len(),
            topic
        );

        if publishers.is_empty() {
            debug!("[TDC] No publishers found initially, waiting for discovery...");
            for attempt in 1..=5 {
                tokio::time::sleep(Duration::from_millis(500)).await;
                publishers = graph.get_entities_by_topic(EntityKind::Publisher, topic);
                debug!(
                    "[TDC] Discovery attempt {}: found {} publishers",
                    attempt,
                    publishers.len()
                );
                if !publishers.is_empty() {
                    break;
                }
            }

            if publishers.is_empty() {
                warn!(
                    "[TDC] No publishers found for topic {} after retries",
                    topic
                );
                return Err(DynamicError::SchemaNotFound(format!(
                    "No publishers found for topic: {}",
                    topic
                )));
            }
        }

        // Extract type name and hash from the first publisher.
        // All publishers on a topic share the same type in ROS 2.
        let first_pub = publishers.first().ok_or_else(|| {
            DynamicError::SchemaNotFound(format!("No publishers found for topic: {}", topic))
        })?;
        let first_ep = match &**first_pub {
            Entity::Endpoint(e) => e,
            _ => {
                return Err(DynamicError::SerializationError(
                    "Expected endpoint entity".to_string(),
                ));
            }
        };
        let type_info = first_ep.type_info.as_ref().ok_or_else(|| {
            DynamicError::SchemaNotFound(format!("Publisher on {} has no type information", topic))
        })?;
        let type_name = normalize_type_name(&type_info.name);
        let type_hash = type_info.hash.to_rihs_string();

        debug!(
            "[TDC] Found publisher for {}: node={}/{}, type={} (normalized from: {})",
            topic, first_ep.node.namespace, first_ep.node.name, type_name, type_info.name
        );

        // ── Phase 2: Create one ZClient (Querier) per publisher endpoint ─────
        //
        // This is the key fix for the flaky race condition.  In rmw_zenoh_cpp the
        // Querier is created once when rmw_create_client is called and then reused
        // for every send_request.  By the time user code calls a service the
        // Querier has been alive long enough to have received queryable
        // advertisements from the router.
        //
        // The old code called create_client() inside the retry loop, meaning a
        // brand-new Querier was created and immediately used.  With AllComplete a
        // fresh Querier that hasn't received advertisements yet resolves the GET
        // instantly with zero replies, causing the 10 s take_response_timeout to
        // expire before a reply ever arrives.
        //
        // By creating all Queriers here, before any GET is sent, we give them time
        // to settle in Phase 3 below.
        // (node_name, namespace, client)
        let mut pub_clients: Vec<(String, String, ZClient<GetTypeDescription>)> = Vec::new();
        for publisher in &publishers {
            let Entity::Endpoint(ep) = &**publisher else {
                continue;
            };
            let service_name = if ep.node.namespace.is_empty() || ep.node.namespace == "/" {
                format!("/{}/get_type_description", ep.node.name)
            } else {
                format!(
                    "{}/{}/get_type_description",
                    ep.node.namespace, ep.node.name
                )
            };
            match self.create_client(&service_name, "") {
                Ok(c) => pub_clients.push((ep.node.name.clone(), ep.node.namespace.clone(), c)),
                Err(e) => warn!("[TDC] Could not create client for {}: {}", service_name, e),
            }
        }

        if pub_clients.is_empty() {
            return Err(DynamicError::SchemaNotFound(format!(
                "Could not create any service clients for topic: {}",
                topic
            )));
        }

        // ── Phase 3: Let Queriers settle ─────────────────────────────────────
        //
        // Allow the newly-declared Queriers to receive queryable advertisements
        // from the router.  This mirrors the implicit settling time rmw_zenoh_cpp
        // benefits from because rmw_create_client and rmw_send_request are called
        // in different stages of a node's lifecycle.
        tokio::time::sleep(Duration::from_millis(100)).await;

        // ── Phase 4: Retry loop using the pre-built clients ──────────────────
        //
        // With settled Queriers the happy path succeeds on the first attempt.
        // The retry loop is a safety net for loaded CI systems or slow networks.
        let deadline = tokio::time::Instant::now() + timeout;
        let mut last_error = None;

        'retry: loop {
            for (node_name, namespace, client) in &pub_clients {
                if tokio::time::Instant::now() > deadline {
                    break 'retry;
                }

                match self
                    .query_with_client(client, node_name, namespace, &type_name, &type_hash, false)
                    .await
                {
                    Ok(response) if response.successful => {
                        let schema = Self::response_to_schema(&response)?;
                        return Ok((schema, type_hash.clone()));
                    }
                    Ok(response) => {
                        // Definitive service failure (e.g. hash mismatch): no point retrying.
                        last_error =
                            Some(DynamicError::SerializationError(response.failure_reason));
                        break 'retry;
                    }
                    Err(e @ DynamicError::ServiceTimeout { .. }) => {
                        // Transient: Querier may not yet have received the advertisement.
                        debug!("[TDC] Type description query timed out, retrying...");
                        last_error = Some(e);
                    }
                    Err(e) => {
                        last_error = Some(e);
                        break 'retry;
                    }
                }
            }

            let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
            if remaining.is_zero() {
                break;
            }

            let delay = Duration::from_millis(500).min(remaining);
            tokio::time::sleep(delay).await;

            // (Publisher list is fixed to the clients built in Phase 2;
            // re-discovery on each retry is not needed once Queriers are settled.)
        }

        Err(last_error.unwrap_or_else(|| {
            DynamicError::SchemaNotFound(format!(
                "Failed to get type description from any publisher on {}",
                topic
            ))
        }))
    }

    /// Convert a GetTypeDescriptionResponse to a MessageSchema.
    ///
    /// This method takes the wire format TypeDescription from a service response
    /// and converts it to a runtime MessageSchema that can be used for dynamic
    /// message handling.
    ///
    /// # Arguments
    ///
    /// * `response` - The response from a GetTypeDescription service call
    ///
    /// # Returns
    ///
    /// An `Arc<MessageSchema>` representing the type.
    pub fn response_to_schema(
        response: &GetTypeDescriptionResponse,
    ) -> Result<Arc<MessageSchema>, DynamicError> {
        if !response.successful {
            return Err(DynamicError::SerializationError(format!(
                "Response indicates failure: {}",
                response.failure_reason
            )));
        }

        // Convert wire format to ros-z-schema types
        let type_desc_msg = wire_to_schema_type_description(&response.type_description);

        // Convert to MessageSchema
        type_description_msg_to_schema(&type_desc_msg)
    }

    /// Create a service client for the given service name.
    fn create_client(
        &self,
        service_name: &str,
        namespace: &str,
    ) -> Result<ZClient<GetTypeDescription>, DynamicError> {
        // Create a temporary node entity for the client
        let node_entity = NodeEntity::new(
            0, // domain_id
            self.session.zid(),
            self.counter.increment(),
            "type_desc_client".to_string(),
            namespace.to_string(),
            String::new(), // enclave (empty, normalized to "%" in liveliness token)
        );

        let entity = EndpointEntity {
            id: self.counter.increment(),
            node: node_entity,
            kind: EntityKind::Client,
            topic: service_name.to_string(),
            type_info: Some(GetTypeDescription::service_type_info()),
            ..Default::default()
        };

        let builder: ZClientBuilder<GetTypeDescription> = ZClientBuilder {
            entity,
            session: self.session.clone(),
            clock: crate::time::ZClock::default(),
            keyexpr_format: ros_z_protocol::KeyExprFormat::default(),
            _phantom_data: Default::default(),
        };

        builder
            .build()
            .map_err(|e| DynamicError::SerializationError(e.to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamic::schema::FieldType;
    use crate::dynamic::type_description_service::{
        WireTypeDescription, schema_to_wire_type_description,
    };

    #[test]
    fn test_response_to_schema_success() {
        // Build a schema and convert to wire format
        let original = MessageSchema::builder("std_msgs/msg/String")
            .field("data", FieldType::String)
            .build()
            .unwrap();

        let wire_td = schema_to_wire_type_description(&original).unwrap();

        let response = GetTypeDescriptionResponse {
            successful: true,
            failure_reason: String::new(),
            type_description: wire_td,
            type_sources: vec![],
            extra_information: vec![],
        };

        let schema = TypeDescriptionClient::response_to_schema(&response).unwrap();
        assert_eq!(schema.type_name, "std_msgs/msg/String");
        assert_eq!(schema.fields.len(), 1);
        assert_eq!(schema.fields[0].name, "data");
    }

    #[test]
    fn test_response_to_schema_failure() {
        let response = GetTypeDescriptionResponse {
            successful: false,
            failure_reason: "Type not found".to_string(),
            type_description: WireTypeDescription::default(),
            type_sources: vec![],
            extra_information: vec![],
        };

        let result = TypeDescriptionClient::response_to_schema(&response);
        assert!(result.is_err());
    }

    #[test]
    fn test_response_to_schema_nested() {
        // Build a nested schema
        let vector3 = MessageSchema::builder("geometry_msgs/msg/Vector3")
            .field("x", FieldType::Float64)
            .field("y", FieldType::Float64)
            .field("z", FieldType::Float64)
            .build()
            .unwrap();

        let twist = MessageSchema::builder("geometry_msgs/msg/Twist")
            .field("linear", FieldType::Message(vector3.clone()))
            .field("angular", FieldType::Message(vector3))
            .build()
            .unwrap();

        let wire_td = schema_to_wire_type_description(&twist).unwrap();

        let response = GetTypeDescriptionResponse {
            successful: true,
            failure_reason: String::new(),
            type_description: wire_td,
            type_sources: vec![],
            extra_information: vec![],
        };

        let schema = TypeDescriptionClient::response_to_schema(&response).unwrap();
        assert_eq!(schema.type_name, "geometry_msgs/msg/Twist");
        assert_eq!(schema.fields.len(), 2);

        // Verify nested types are resolved
        if let FieldType::Message(nested) = &schema.fields[0].field_type {
            assert_eq!(nested.type_name, "geometry_msgs/msg/Vector3");
            assert_eq!(nested.fields.len(), 3);
        } else {
            panic!("Expected Message type for linear field");
        }
    }
}
