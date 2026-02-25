//! Parameter service implementation.
//!
//! Creates the 6 parameter service servers and the /parameter_events publisher
//! for a node. Follows the same pattern as TypeDescriptionService.

use std::collections::HashMap;
use std::sync::{Arc, RwLock};

use tracing::{debug, info, warn};
use zenoh::query::Query;
use zenoh::{Result as ZResult, Session};

use crate::Builder;
use crate::ServiceTypeInfo;
use crate::context::GlobalCounter;
use crate::entity::{EndpointEntity, EntityKind, NodeEntity, TypeInfo};
use crate::msg::{CdrSerdes, ZDeserializer, ZSerializer};
use crate::pubsub::{ZPub, ZPubBuilder};
use crate::qos::{QosDurability, QosHistory, QosProfile, QosReliability};
use crate::service::ZServerBuilder;

use super::store::ParameterStore;
use super::types::{Parameter, ParameterDescriptor, ParameterValue, SetParametersResult};
use super::wire_types::{
    self, DescribeParametersSrv, GetParameterTypesSrv, GetParametersSrv, ListParametersSrv,
    SetParametersAtomicallySrv, SetParametersSrv, WireParameter, WireParameterEvent,
    WireSetParametersResult, WireTime, parameter_event_type_info,
};

type SetCallback = Arc<dyn Fn(&[Parameter]) -> SetParametersResult + Send + Sync>;

/// Holds a type-erased ZServer to keep it alive without naming the full type.
type BoxedServer = Arc<dyn std::any::Any + Send + Sync>;

/// Manages ROS 2 parameter services and event publication for a node.
#[derive(Clone)]
pub struct ParameterService {
    store: Arc<RwLock<ParameterStore>>,
    on_set_callback: Arc<RwLock<Option<SetCallback>>>,
    event_publisher: Arc<ZPub<WireParameterEvent, CdrSerdes<WireParameterEvent>>>,
    /// Keeps all service servers alive.
    _servers: Arc<[BoxedServer]>,
    node_fqn: String,
}

impl ParameterService {
    /// Create a new ParameterService for the given node.
    ///
    /// Spawns 6 parameter service servers and creates the /parameter_events publisher.
    pub fn new(
        session: Arc<Session>,
        node_name: &str,
        namespace: &str,
        node_id: usize,
        counter: &GlobalCounter,
        overrides: HashMap<String, ParameterValue>,
    ) -> ZResult<Self> {
        let store: Arc<RwLock<ParameterStore>> = Arc::new(RwLock::new(if overrides.is_empty() {
            ParameterStore::new()
        } else {
            ParameterStore::with_overrides(overrides)
        }));
        let on_set_callback: Arc<RwLock<Option<SetCallback>>> = Arc::new(RwLock::new(None));

        let node_entity = NodeEntity::new(
            0,
            session.zid(),
            node_id,
            node_name.to_string(),
            namespace.to_string(),
            String::new(),
        );

        // Compute node fully-qualified name for parameter events
        let node_fqn = if namespace.is_empty() || namespace == "/" {
            format!("/{}", node_name)
        } else {
            format!("{}/{}", namespace, node_name)
        };

        // Helper to build a service server entity
        let make_entity =
            |counter: &GlobalCounter, service_name: &str, type_info: TypeInfo| EndpointEntity {
                id: counter.increment(),
                node: node_entity.clone(),
                kind: EntityKind::Service,
                topic: service_name.to_string(),
                type_info: Some(type_info),
                ..Default::default()
            };

        let ke_format = ros_z_protocol::KeyExprFormat::default();

        // ── describe_parameters ───────────────────────────────────────────────
        let describe_store = store.clone();
        let describe_server = {
            let entity = make_entity(
                counter,
                "~describe_parameters",
                DescribeParametersSrv::service_type_info(),
            );
            let builder: ZServerBuilder<DescribeParametersSrv> = ZServerBuilder {
                entity,
                session: session.clone(),
                keyexpr_format: ke_format,
                _phantom_data: Default::default(),
            };
            builder.build_with_callback(move |query| {
                handle_describe_parameters(&describe_store, query);
            })?
        };

        // ── get_parameters ───────────────────────────────────────────────────
        let get_store = store.clone();
        let get_server = {
            let entity = make_entity(
                counter,
                "~get_parameters",
                GetParametersSrv::service_type_info(),
            );
            let builder: ZServerBuilder<GetParametersSrv> = ZServerBuilder {
                entity,
                session: session.clone(),
                keyexpr_format: ke_format,
                _phantom_data: Default::default(),
            };
            builder.build_with_callback(move |query| {
                handle_get_parameters(&get_store, query);
            })?
        };

        // ── get_parameter_types ───────────────────────────────────────────────
        let types_store = store.clone();
        let types_server = {
            let entity = make_entity(
                counter,
                "~get_parameter_types",
                GetParameterTypesSrv::service_type_info(),
            );
            let builder: ZServerBuilder<GetParameterTypesSrv> = ZServerBuilder {
                entity,
                session: session.clone(),
                keyexpr_format: ke_format,
                _phantom_data: Default::default(),
            };
            builder.build_with_callback(move |query| {
                handle_get_parameter_types(&types_store, query);
            })?
        };

        // ── list_parameters ───────────────────────────────────────────────────
        let list_store = store.clone();
        let list_server = {
            let entity = make_entity(
                counter,
                "~list_parameters",
                ListParametersSrv::service_type_info(),
            );
            let builder: ZServerBuilder<ListParametersSrv> = ZServerBuilder {
                entity,
                session: session.clone(),
                keyexpr_format: ke_format,
                _phantom_data: Default::default(),
            };
            builder.build_with_callback(move |query| {
                handle_list_parameters(&list_store, query);
            })?
        };

        // ── set_parameters ────────────────────────────────────────────────────
        // set_parameters gets both store and callback so it can publish events too;
        // we pass the publisher indirectly via a channel approach — but to keep it
        // simple, we'll clone the session for event publishing inside the callback.
        let set_store = store.clone();
        let set_callback = on_set_callback.clone();
        let set_session = session.clone();
        let set_fqn = node_fqn.clone();
        let set_server = {
            let entity = make_entity(
                counter,
                "~set_parameters",
                SetParametersSrv::service_type_info(),
            );
            let builder: ZServerBuilder<SetParametersSrv> = ZServerBuilder {
                entity,
                session: session.clone(),
                keyexpr_format: ke_format,
                _phantom_data: Default::default(),
            };
            builder.build_with_callback(move |query| {
                handle_set_parameters(&set_store, &set_callback, &set_session, &set_fqn, query);
            })?
        };

        // ── set_parameters_atomically ─────────────────────────────────────────
        let atomic_store = store.clone();
        let atomic_callback = on_set_callback.clone();
        let atomic_session = session.clone();
        let atomic_fqn = node_fqn.clone();
        let atomic_server = {
            let entity = make_entity(
                counter,
                "~set_parameters_atomically",
                SetParametersAtomicallySrv::service_type_info(),
            );
            let builder: ZServerBuilder<SetParametersAtomicallySrv> = ZServerBuilder {
                entity,
                session: session.clone(),
                keyexpr_format: ke_format,
                _phantom_data: Default::default(),
            };
            builder.build_with_callback(move |query| {
                handle_set_parameters_atomically(
                    &atomic_store,
                    &atomic_callback,
                    &atomic_session,
                    &atomic_fqn,
                    query,
                );
            })?
        };

        // ── /parameter_events publisher ───────────────────────────────────────
        let pub_entity = EndpointEntity {
            id: counter.increment(),
            node: node_entity,
            kind: EntityKind::Publisher,
            topic: "/parameter_events".to_string(),
            type_info: Some(parameter_event_type_info()),
            qos: {
                let qos = QosProfile {
                    reliability: QosReliability::Reliable,
                    durability: QosDurability::TransientLocal,
                    history: QosHistory::KeepLast(std::num::NonZeroUsize::new(1000).unwrap()),
                    ..Default::default()
                };
                qos.to_protocol_qos()
            },
        };

        let pub_builder: ZPubBuilder<WireParameterEvent, CdrSerdes<WireParameterEvent>> =
            ZPubBuilder {
                entity: pub_entity,
                session: session.clone(),
                with_attachment: true,
                shm_config: None,
                keyexpr_format: ke_format,
                dyn_schema: None,
                encoding: None,
                _phantom_data: Default::default(),
            };

        let event_publisher = Arc::new(pub_builder.build()?);

        info!(
            "[PARAMS] ParameterService created for node: {}/{}",
            namespace, node_name
        );

        let servers: Arc<[BoxedServer]> = Arc::from(vec![
            Arc::new(describe_server) as BoxedServer,
            Arc::new(get_server) as BoxedServer,
            Arc::new(types_server) as BoxedServer,
            Arc::new(list_server) as BoxedServer,
            Arc::new(set_server) as BoxedServer,
            Arc::new(atomic_server) as BoxedServer,
        ]);

        Ok(Self {
            store,
            on_set_callback,
            event_publisher,
            _servers: servers,
            node_fqn,
        })
    }

    // ── Public API ────────────────────────────────────────────────────────────

    pub fn declare_parameter(
        &self,
        name: &str,
        default: ParameterValue,
        descriptor: ParameterDescriptor,
    ) -> Result<ParameterValue, String> {
        self.store
            .write()
            .map_err(|_| "parameter store lock poisoned".to_string())?
            .declare(name, default, descriptor)
    }

    pub fn get_parameter(&self, name: &str) -> Option<ParameterValue> {
        self.store.read().ok()?.get(name)
    }

    pub fn set_parameter(&self, param: Parameter) -> SetParametersResult {
        let validation_result = self.validate_and_apply(std::slice::from_ref(&param), false);
        validation_result
            .into_iter()
            .next()
            .unwrap_or_else(|| SetParametersResult::failure("internal error: empty result"))
    }

    pub fn undeclare_parameter(&self, name: &str) -> Result<(), String> {
        self.store
            .write()
            .map_err(|_| "parameter store lock poisoned".to_string())?
            .undeclare(name)
    }

    pub fn describe_parameter(&self, name: &str) -> Option<ParameterDescriptor> {
        self.store.read().ok()?.describe(name)
    }

    pub fn list_parameters(
        &self,
        prefixes: &[String],
        depth: u64,
    ) -> wire_types::WireListParametersResult {
        self.store
            .read()
            .map(|s| s.list(prefixes, depth))
            .unwrap_or_default()
    }

    pub fn on_set_parameters<F>(&self, callback: F)
    where
        F: Fn(&[Parameter]) -> SetParametersResult + Send + Sync + 'static,
    {
        if let Ok(mut cb) = self.on_set_callback.write() {
            *cb = Some(Arc::new(callback));
        }
    }

    // ── Internal helpers ─────────────────────────────────────────────────────

    /// Validate and apply a set of parameters. Returns one result per parameter.
    /// If `atomic` is true, either all succeed or none are committed.
    pub(crate) fn validate_and_apply(
        &self,
        params: &[Parameter],
        atomic: bool,
    ) -> Vec<SetParametersResult> {
        let mut results = Vec::with_capacity(params.len());

        // Phase 1: built-in validation
        {
            let store = match self.store.read() {
                Ok(s) => s,
                Err(_) => {
                    return params
                        .iter()
                        .map(|_| SetParametersResult::failure("store lock poisoned"))
                        .collect();
                }
            };

            for param in params {
                match store.validate_set(param) {
                    Ok(_) => results.push(SetParametersResult::success()),
                    Err(reason) => results.push(SetParametersResult::failure(reason)),
                }
            }
        }

        // Phase 2: user callback validation
        let all_passed = results.iter().all(|r| r.successful);
        if all_passed
            && let Ok(cb_guard) = self.on_set_callback.read()
            && let Some(cb_results) = cb_guard.as_ref().map(|cb| cb(params))
            && !cb_results.successful
        {
            return params
                .iter()
                .map(|_| SetParametersResult::failure(cb_results.reason.clone()))
                .collect();
        }

        // Phase 3: commit (only if all passed, or not atomic)
        let mut new_params = Vec::new();
        let mut changed_params = Vec::new();
        let mut deleted_params = Vec::new();

        if atomic && !results.iter().all(|r| r.successful) {
            return results;
        }

        if let Ok(mut store) = self.store.write() {
            for (i, param) in params.iter().enumerate() {
                if results[i].successful {
                    let is_new = !store.has(&param.name);
                    let old = store.set(param);

                    if is_new {
                        new_params.push(param.to_wire());
                    } else if old.is_some() {
                        if param.value == super::types::ParameterValue::NotSet {
                            deleted_params.push(param.to_wire());
                        } else {
                            changed_params.push(param.to_wire());
                        }
                    }
                }
            }
        }

        // Phase 4: publish parameter event
        if !new_params.is_empty() || !changed_params.is_empty() || !deleted_params.is_empty() {
            let event = WireParameterEvent {
                stamp: WireTime::default(),
                node: self.node_fqn.clone(),
                new_parameters: new_params,
                changed_parameters: changed_params,
                deleted_parameters: deleted_params,
            };
            if let Err(e) = self.event_publisher.publish(&event) {
                warn!("[PARAMS] Failed to publish parameter event: {}", e);
            }
        }

        results
    }
}

// ── Service callback handlers ─────────────────────────────────────────────────

fn reply_with<T: serde::Serialize>(query: Query, response: &T) {
    let bytes = CdrSerdes::<T>::serialize(response);
    use zenoh::Wait;
    if let Err(e) = query.reply(query.key_expr().clone(), bytes).wait() {
        warn!("[PARAMS] Failed to send response: {}", e);
    }
}

fn deserialize_request<T: for<'de> serde::Deserialize<'de>>(query: &Query) -> Option<T> {
    let payload = query.payload()?;
    match CdrSerdes::<T>::deserialize(payload.to_bytes().as_ref()) {
        Ok(req) => Some(req),
        Err(e) => {
            warn!("[PARAMS] Failed to deserialize request: {}", e);
            None
        }
    }
}

fn handle_describe_parameters(store: &Arc<RwLock<ParameterStore>>, query: Query) {
    let Some(req) = deserialize_request::<wire_types::DescribeParametersRequest>(&query) else {
        return;
    };
    debug!("[PARAMS] describe_parameters: {:?}", req.names);
    let descriptors = match store.read() {
        Ok(s) => s.describe_many(&req.names),
        Err(_) => return,
    };
    let response = wire_types::DescribeParametersResponse { descriptors };
    reply_with(query, &response);
}

fn handle_get_parameters(store: &Arc<RwLock<ParameterStore>>, query: Query) {
    let Some(req) = deserialize_request::<wire_types::GetParametersRequest>(&query) else {
        return;
    };
    debug!("[PARAMS] get_parameters: {:?}", req.names);
    let values = match store.read() {
        Ok(s) => s.get_many(&req.names),
        Err(_) => return,
    };
    let response = wire_types::GetParametersResponse { values };
    reply_with(query, &response);
}

fn handle_get_parameter_types(store: &Arc<RwLock<ParameterStore>>, query: Query) {
    let Some(req) = deserialize_request::<wire_types::GetParameterTypesRequest>(&query) else {
        return;
    };
    debug!("[PARAMS] get_parameter_types: {:?}", req.names);
    let types = match store.read() {
        Ok(s) => s.get_types(&req.names),
        Err(_) => return,
    };
    let response = wire_types::GetParameterTypesResponse { types };
    reply_with(query, &response);
}

fn handle_list_parameters(store: &Arc<RwLock<ParameterStore>>, query: Query) {
    let Some(req) = deserialize_request::<wire_types::ListParametersRequest>(&query) else {
        return;
    };
    debug!(
        "[PARAMS] list_parameters: prefixes={:?}, depth={}",
        req.prefixes, req.depth
    );
    let result = match store.read() {
        Ok(s) => s.list(&req.prefixes, req.depth),
        Err(_) => return,
    };
    let response = wire_types::ListParametersResponse { result };
    reply_with(query, &response);
}

fn handle_set_parameters(
    store: &Arc<RwLock<ParameterStore>>,
    callback: &Arc<RwLock<Option<SetCallback>>>,
    session: &Arc<Session>,
    node_fqn: &str,
    query: Query,
) {
    let Some(req) = deserialize_request::<wire_types::SetParametersRequest>(&query) else {
        return;
    };
    debug!("[PARAMS] set_parameters: {} params", req.parameters.len());

    let params: Vec<Parameter> = req.parameters.iter().map(Parameter::from_wire).collect();
    let results = apply_parameters(store, callback, session, node_fqn, &params, false);

    let wire_results: Vec<WireSetParametersResult> = results.iter().map(|r| r.to_wire()).collect();
    let response = wire_types::SetParametersResponse {
        results: wire_results,
    };
    reply_with(query, &response);
}

fn handle_set_parameters_atomically(
    store: &Arc<RwLock<ParameterStore>>,
    callback: &Arc<RwLock<Option<SetCallback>>>,
    session: &Arc<Session>,
    node_fqn: &str,
    query: Query,
) {
    let Some(req) = deserialize_request::<wire_types::SetParametersAtomicallyRequest>(&query)
    else {
        return;
    };
    debug!(
        "[PARAMS] set_parameters_atomically: {} params",
        req.parameters.len()
    );

    let params: Vec<Parameter> = req.parameters.iter().map(Parameter::from_wire).collect();
    let results = apply_parameters(store, callback, session, node_fqn, &params, true);

    // Atomically: overall success only if all succeeded
    let successful = results.iter().all(|r| r.successful);
    let reason = if successful {
        String::new()
    } else {
        results
            .iter()
            .find(|r| !r.successful)
            .map(|r| r.reason.clone())
            .unwrap_or_default()
    };
    let response = wire_types::SetParametersAtomicallyResponse {
        result: WireSetParametersResult { successful, reason },
    };
    reply_with(query, &response);
}

/// Shared apply logic used by both set_parameters handlers.
fn apply_parameters(
    store: &Arc<RwLock<ParameterStore>>,
    callback: &Arc<RwLock<Option<SetCallback>>>,
    session: &Arc<Session>,
    node_fqn: &str,
    params: &[Parameter],
    atomic: bool,
) -> Vec<SetParametersResult> {
    let mut results = Vec::with_capacity(params.len());

    // Phase 1: built-in validation
    {
        let Ok(s) = store.read() else {
            return params
                .iter()
                .map(|_| SetParametersResult::failure("store lock poisoned"))
                .collect();
        };
        for param in params {
            match s.validate_set(param) {
                Ok(_) => results.push(SetParametersResult::success()),
                Err(reason) => results.push(SetParametersResult::failure(reason)),
            }
        }
    }

    // Phase 2: user callback
    let all_passed = results.iter().all(|r| r.successful);
    if all_passed
        && let Ok(cb_guard) = callback.read()
        && let Some(cb_result) = cb_guard.as_ref().map(|cb| cb(params))
        && !cb_result.successful
    {
        return params
            .iter()
            .map(|_| SetParametersResult::failure(cb_result.reason.clone()))
            .collect();
    }

    // Bail early for atomic if any failed
    if atomic && !results.iter().all(|r| r.successful) {
        return results;
    }

    // Phase 3: commit
    let mut new_params: Vec<WireParameter> = Vec::new();
    let mut changed_params: Vec<WireParameter> = Vec::new();
    let mut deleted_params: Vec<WireParameter> = Vec::new();

    if let Ok(mut s) = store.write() {
        for (i, param) in params.iter().enumerate() {
            if results[i].successful {
                let is_new = !s.has(&param.name);
                let old = s.set(param);
                if is_new {
                    new_params.push(param.to_wire());
                } else if old.is_some() {
                    if param.value == ParameterValue::NotSet {
                        deleted_params.push(param.to_wire());
                    } else {
                        changed_params.push(param.to_wire());
                    }
                }
            }
        }
    }

    // Phase 4: publish parameter event via Zenoh directly
    if !new_params.is_empty() || !changed_params.is_empty() || !deleted_params.is_empty() {
        publish_parameter_event(
            session,
            node_fqn,
            new_params,
            changed_params,
            deleted_params,
        );
    }

    results
}

/// Publish a ParameterEvent directly via the session (used from service callbacks).
fn publish_parameter_event(
    session: &Arc<Session>,
    node_fqn: &str,
    new_parameters: Vec<WireParameter>,
    changed_parameters: Vec<WireParameter>,
    deleted_parameters: Vec<WireParameter>,
) {
    let event = WireParameterEvent {
        stamp: WireTime::default(),
        node: node_fqn.to_string(),
        new_parameters,
        changed_parameters,
        deleted_parameters,
    };

    let bytes = CdrSerdes::<WireParameterEvent>::serialize(&event);

    // Publish to the Zenoh key for /parameter_events
    // We use a simplified key without QoS suffix since this is a well-known topic.
    // The topic key format for a publisher is: rt/parameter_events
    let key = "rt/parameter_events";
    use zenoh::Wait;
    if let Err(e) = session.put(key, bytes).wait() {
        warn!("[PARAMS] Failed to publish parameter event: {}", e);
    }
}
