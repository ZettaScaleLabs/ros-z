use parking_lot::Mutex;
use serde::Serialize;
use slab::Slab;
use std::{
    collections::{HashMap, HashSet},
    sync::{Arc, Weak},
    time::SystemTime,
};
use tracing::debug;

use crate::entity::{
    ADMIN_SPACE, EndpointEntity, Entity, EntityKind, LivelinessKE, NodeKey, Topic,
};
use crate::event::GraphEventManager;
use zenoh::{Result, Session, Wait, pubsub::Subscriber, sample::SampleKind, session::ZenohId};
use tracing;

/// A serializable snapshot of the ROS graph state
#[derive(Debug, Clone, Serialize)]
pub struct GraphSnapshot {
    pub timestamp: SystemTime,
    pub domain_id: usize,
    pub topics: Vec<TopicSnapshot>,
    pub nodes: Vec<NodeSnapshot>,
    pub services: Vec<ServiceSnapshot>,
}

#[derive(Debug, Clone, Serialize)]
pub struct TopicSnapshot {
    pub name: String,
    #[serde(rename = "type")]
    pub type_name: String,
    pub publishers: usize,
    pub subscribers: usize,
}

#[derive(Debug, Clone, Serialize)]
pub struct NodeSnapshot {
    pub name: String,
    pub namespace: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct ServiceSnapshot {
    pub name: String,
    #[serde(rename = "type")]
    pub type_name: String,
}

const DEFAULT_SLAB_CAPACITY: usize = 128;

/// Type alias for entity parser function
type EntityParser = Arc<dyn Fn(&zenoh::key_expr::KeyExpr) -> Result<Entity> + Send + Sync>;

pub struct GraphData {
    cached: HashSet<LivelinessKE>,
    parsed: HashMap<LivelinessKE, Arc<Entity>>,
    by_topic: HashMap<Topic, Slab<Weak<Entity>>>,
    by_service: HashMap<Topic, Slab<Weak<Entity>>>,
    by_node: HashMap<NodeKey, Slab<Weak<Entity>>>,
    parser: EntityParser,
}

impl GraphData {
    fn new_with_parser(parser: EntityParser) -> Self {
        Self {
            cached: HashSet::new(),
            parsed: HashMap::new(),
            by_topic: HashMap::new(),
            by_service: HashMap::new(),
            by_node: HashMap::new(),
            parser,
        }
    }

    fn insert(&mut self, ke: LivelinessKE) {
        // Skip if already parsed to avoid duplicates
        if self.parsed.contains_key(&ke) {
            tracing::debug!("insert: Skipping already parsed key");
            return;
        }
        self.cached.insert(ke);
    }

    fn remove(&mut self, ke: &LivelinessKE) {
        let was_cached = self.cached.remove(ke);
        let was_parsed = self.parsed.remove(ke);
        debug!("[GRF] Removed KE: {}, cached={}, parsed={}", ke.0, was_cached, was_parsed.is_some());

        if was_parsed.is_some() {
            tracing::debug!("remove: Removed from parsed");
        }

        // Note: We don't eagerly remove from by_topic/by_service/by_node maps here.
        // The weak references will naturally fail to upgrade when entities are dropped,
        // and the retain() calls in visit_by_* functions will clean them up lazily.
        // This matches rmw_zenoh_cpp's approach.

        match (was_cached, was_parsed) {
            // Both should not be present at the same time
            (true, Some(_)) => {
                eprintln!(
                    "Warning: LivelinessKE was in both cached and parsed: {:?}",
                    ke
                );
            }
            // If not in either set, it might have been already removed or never existed
            (false, None) => {
                // This can happen due to duplicate removal events or race conditions
                // Log but don't panic
            }
            // Expected cases: either in cached (not yet parsed) or in parsed
            _ => {}
        }
    }

    fn parse(&mut self) {
        let count = self.cached.len();
        debug!("[GRF] Parsing {} cached entities", count);

        for ke in self.cached.drain() {
            // Skip if already parsed (e.g., added via add_local_entity)
            if self.parsed.contains_key(&ke) {
                tracing::debug!("parse: Skipping already parsed key");
                continue;
            }

            // Parse using backend-specific parser
            let entity = match (self.parser)(&ke.0) {
                Ok(e) => e,
                Err(e) => {
                    tracing::warn!("Failed to parse liveliness key {}: {:?}", ke.0, e);
                    continue;
                }
            };
            let arc = Arc::new(entity);
            let weak = Arc::downgrade(&arc);
            match &*arc {
                Entity::Node(x) => {
                    debug!("[GRF] Parsed node: {}/{}", x.namespace, x.name);

                    // TODO: omit the clone of node key
                    let node_key = x.key();
                    tracing::debug!(
                        "parse: Storing Node entity with key=({:?}, {:?})",
                        node_key.0, node_key.1
                    );
                    let slab = self
                        .by_node
                        .entry(node_key)
                        .or_insert_with(|| Slab::with_capacity(DEFAULT_SLAB_CAPACITY));

                            // If slab is full, remove failing weak pointers first
                            if slab.len() >= slab.capacity() {
                                slab.retain(|_, weak_ptr| weak_ptr.upgrade().is_some());
                            }

                            slab.insert(weak);
                }
                Entity::Endpoint(x) => {
                    debug!("[GRF] Parsed endpoint: kind={:?}, topic={}, node={}/{}",
                        x.kind, x.topic, x.node.namespace, x.node.name);
                    let node_key = x.node.key();
                    let type_str = x.type_info.as_ref().map(|t| t.name.as_str()).unwrap_or("unknown");
                    tracing::debug!(
                        "parse: Storing Endpoint ({:?}) for node_key=({:?}, {:?}), topic={}, type={}, id={}",
                        x.kind, node_key.0, node_key.1, x.topic, type_str, x.id
                    );

                    // Index by topic for Publisher/Subscription entities
                    if matches!(x.kind, EntityKind::Publisher | EntityKind::Subscription) {
                        // TODO: omit the clone of topic
                        let topic_slab = self
                            .by_topic
                            .entry(x.topic.clone())
                            .or_insert_with(|| Slab::with_capacity(DEFAULT_SLAB_CAPACITY));

                        // If slab is full, remove failing weak pointers first
                        if topic_slab.len() >= topic_slab.capacity() {
                            topic_slab.retain(|_, weak_ptr| weak_ptr.upgrade().is_some());
                        }

                        topic_slab.insert(weak.clone());
                    }

                    // Index by service for Service/Client entities
                    if matches!(x.kind, EntityKind::Service | EntityKind::Client) {
                        // TODO: omit the clone of service name (stored in topic field)
                        let service_slab = self
                            .by_service
                            .entry(x.topic.clone())
                            .or_insert_with(|| Slab::with_capacity(DEFAULT_SLAB_CAPACITY));

                        // If slab is full, remove failing weak pointers first
                        if service_slab.len() >= service_slab.capacity() {
                            service_slab.retain(|_, weak_ptr| weak_ptr.upgrade().is_some());
                        }

                        service_slab.insert(weak.clone());
                    }
                    let node_slab = self
                        .by_node
                        .entry(node_key)
                        .or_insert_with(|| Slab::with_capacity(DEFAULT_SLAB_CAPACITY));

                    // If slab is full, remove failing weak pointers first
                    if node_slab.len() >= node_slab.capacity() {
                        node_slab.retain(|_, weak_ptr| weak_ptr.upgrade().is_some());
                    }

                    node_slab.insert(weak);
                }
            }
            self.parsed.insert(ke, arc);
        }
    }

    pub fn visit_by_node<F>(&mut self, node_key: NodeKey, mut f: F)
    where
        F: FnMut(Arc<Entity>),
    {
        if !self.cached.is_empty() {
            self.parse();
        }

        if let Some(entities) = self.by_node.get_mut(&node_key) {
            tracing::debug!("visit_by_node: Found {} entities in slab for node ({:?}, {:?})", entities.len(), node_key.0, node_key.1);
            let mut upgraded = 0;
            let mut failed = 0;
            entities.retain(|_, weak| {
                if let Some(rc) = weak.upgrade() {
                    f(rc);
                    upgraded += 1;
                    true
                } else {
                    failed += 1;
                    false
                }
            });
            tracing::debug!("visit_by_node: Upgraded {} entities, failed to upgrade {}", upgraded, failed);
        } else {
            tracing::debug!("visit_by_node: No entities found for node ({:?}, {:?})", node_key.0, node_key.1);
        }
    }

    pub fn visit_by_topic<F>(&mut self, topic: impl AsRef<str>, mut f: F)
    where
        F: FnMut(Arc<Entity>),
    {
        if !self.cached.is_empty() {
            self.parse();
        }

        if let Some(entities) = self.by_topic.get_mut(topic.as_ref()) {
            entities.retain(|_, weak| {
                if let Some(rc) = weak.upgrade() {
                    f(rc);
                    true
                } else {
                    false
                }
            });
        }
    }

    pub fn visit_by_service<F>(&mut self, service_name: impl AsRef<str>, mut f: F)
    where
        F: FnMut(Arc<Entity>),
    {
        if !self.cached.is_empty() {
            self.parse();
        }

        if let Some(entities) = self.by_service.get_mut(service_name.as_ref()) {
            entities.retain(|_, weak| {
                if let Some(rc) = weak.upgrade() {
                    f(rc);
                    true
                } else {
                    false
                }
            });
        }
    }
}

pub struct Graph {
    pub data: Arc<Mutex<GraphData>>,
    pub event_manager: Arc<GraphEventManager>,
    pub zid: ZenohId,
    _subscriber: Subscriber<()>,
}

impl Graph {
    pub fn new(session: &Session, domain_id: usize) -> Result<Self> {
        use crate::backend::{KeyExprBackend, RmwZenohBackend};
        // Default to RmwZenoh backend format with RmwZenohBackend parser
        Self::new_with_pattern(
            session,
            domain_id,
            format!("{ADMIN_SPACE}/{domain_id}/**"),
            RmwZenohBackend::parse_liveliness,
        )
    }

    /// Create a new Graph with a custom liveliness subscription pattern and parser
    ///
    /// # Arguments
    /// * `session` - Zenoh session
    /// * `domain_id` - ROS domain ID (used for filtering, may not be in pattern for ros2dds)
    /// * `liveliness_pattern` - Liveliness key expression pattern to subscribe to
    /// * `parser` - Function to parse liveliness key expressions into Entity
    ///
    /// # Backend Patterns
    /// * RmwZenoh: `@ros2_lv/{domain_id}/**`
    /// * Ros2Dds: `@/*/@ros2_lv/**`
    pub fn new_with_pattern<F>(
        session: &Session,
        _domain_id: usize,
        liveliness_pattern: String,
        parser: F,
    ) -> Result<Self>
    where
        F: Fn(&zenoh::key_expr::KeyExpr) -> Result<Entity> + Send + Sync + 'static,
    {
        let zid = session.zid();
        let parser_arc = Arc::new(parser);
        let graph_data = Arc::new(Mutex::new(GraphData::new_with_parser(parser_arc.clone())));
        let event_manager = Arc::new(GraphEventManager::new());
        let c_graph_data = graph_data.clone();
        let c_event_manager = event_manager.clone();
        let c_zid = zid;
        let c_liveliness_pattern = liveliness_pattern.clone();
        let c_parser = parser_arc.clone();
        let callback_parser = parser_arc.clone();
        tracing::debug!("Creating liveliness subscriber for {}", liveliness_pattern);
        let sub = session
            .liveliness()
            .declare_subscriber(&liveliness_pattern)
            .history(true)
            .callback(move |sample| {
                let mut graph_data_guard = c_graph_data.lock();
                let key_expr = sample.key_expr().to_owned();
                let ke = LivelinessKE(key_expr.clone());
                tracing::debug!("Received liveliness token: {} kind={:?}", key_expr, sample.kind());

                match sample.kind() {
                    SampleKind::Put => {
                        debug!("[GRF] Entity appeared: {}", ke.0);
                        tracing::debug!("Graph subscriber: PUT {}", key_expr.as_str());
                        // Only insert if not already parsed (avoid duplicates from liveliness query)
                        let already_parsed = graph_data_guard.parsed.contains_key(&ke);
                        let already_cached = graph_data_guard.cached.contains(&ke);
                        tracing::debug!("  Check: parsed={}, cached={}, parsed.len()={}, cached.len()={}",
                            already_parsed, already_cached, graph_data_guard.parsed.len(), graph_data_guard.cached.len());
                        if already_parsed {
                            tracing::debug!("  Skipping - already in parsed");
                        } else if already_cached {
                            tracing::debug!("  Skipping - already in cached");
                        } else {
                            tracing::debug!("  Adding to cached");
                            graph_data_guard.insert(ke.clone());
                        }
                        // Trigger graph change events using backend-specific parser
                        match callback_parser(&key_expr) {
                            Ok(entity) => {
                                tracing::debug!("Successfully parsed entity: {:?}", entity);
                                c_event_manager.trigger_graph_change(&entity, true, c_zid);
                            }
                            Err(e) => {
                                tracing::warn!("Failed to parse liveliness token {}: {:?}", key_expr, e);
                            }
                        }
                    }
                    SampleKind::Delete => {
                        debug!("[GRF] Entity disappeared: {}", ke.0);
                        tracing::debug!("Graph subscriber: DELETE {}", key_expr.as_str());
                        // Trigger graph change events before removal using backend-specific parser
                        if let Ok(entity) = callback_parser(&key_expr) {
                            c_event_manager.trigger_graph_change(&entity, false, c_zid);
                        }
                        graph_data_guard.remove(&ke);
                    }
                }
            })
            .wait()?;


        // Query existing liveliness tokens from all connected sessions
        // This is crucial for cross-context discovery where entities from other sessions
        // were created before this session started
        let replies = session
            .liveliness()
            .get(&c_liveliness_pattern)
            .timeout(std::time::Duration::from_secs(3))
            .wait()?;

        // Process all replies and add them to the graph
        // IMPORTANT: Filter out entities from the current session to avoid duplicates
        // Local entities are already added via add_local_entity()
        let mut reply_count = 0;
        let mut filtered_count = 0;
        while let Ok(reply) = replies.recv() {
            reply_count += 1;
            if let Ok(sample) = reply.into_result() {
                let key_expr = sample.key_expr().to_owned();
                let ke = LivelinessKE(key_expr.clone());

                // Parse entity to check if it's from current session using backend-specific parser
                if let Ok(entity) = c_parser(&key_expr) {
                    // Skip entities from current session
                    let is_local = match &entity {
                        Entity::Node(node) => node.z_id == zid,
                        Entity::Endpoint(endpoint) => endpoint.node.z_id == zid,
                    };

                    if !is_local {
                        // Only insert entities from other sessions
                        tracing::debug!("Graph: Adding cross-context entity: {}", key_expr.as_str());
                        graph_data.lock().insert(ke);
                    } else {
                        filtered_count += 1;
                        tracing::debug!("Graph: Filtered local entity: {}", key_expr.as_str());
                    }
                }
            }
        }
        tracing::debug!(
            "Graph: Liveliness query received {} replies, filtered {} local entities",
            reply_count,
            filtered_count
        );

        Ok(Self {
            _subscriber: sub,
            data: graph_data,
            event_manager,
            zid,
        })
    }

    /// Check if an entity belongs to the current session
    pub fn is_entity_local(&self, entity: &Entity) -> bool {
        match entity {
            Entity::Node(node) => node.z_id == self.zid,
            Entity::Endpoint(endpoint) => endpoint.node.z_id == self.zid,
        }
    }

    /// Add a local entity to the graph for immediate discovery
    /// This is used to make local publishers/subscriptions/services/clients
    /// immediately visible in graph queries without waiting for Zenoh liveliness propagation
    pub fn add_local_entity(&self, entity: Entity) -> Result<()> {
        let mut data = self.data.lock();

        // Create LivelinessKE from entity
        let ke = LivelinessKE::try_from(&entity)?;

        // Check if entity already exists (to avoid triggering duplicate graph change events)
        let already_exists = data.parsed.contains_key(&ke);

        // Create Arc for the entity and weak reference
        let arc = Arc::new(entity.clone());
        let weak = Arc::downgrade(&arc);

        // Store in parsed HashMap
        data.parsed.insert(ke, arc.clone());

        // Add to appropriate indexes
        match &entity {
            Entity::Node(node) => {
                let slab = data
                    .by_node
                    .entry(node.key())
                    .or_insert_with(|| Slab::with_capacity(DEFAULT_SLAB_CAPACITY));

                if slab.len() >= slab.capacity() {
                    slab.retain(|_, weak_ptr| weak_ptr.upgrade().is_some());
                }
                slab.insert(weak);
            }
            Entity::Endpoint(endpoint) => {
                // Index by topic for Publisher/Subscription
                if matches!(endpoint.kind, EntityKind::Publisher | EntityKind::Subscription) {
                    let topic_slab = data
                        .by_topic
                        .entry(endpoint.topic.clone())
                        .or_insert_with(|| Slab::with_capacity(DEFAULT_SLAB_CAPACITY));

                    if topic_slab.len() >= topic_slab.capacity() {
                        topic_slab.retain(|_, weak_ptr| weak_ptr.upgrade().is_some());
                    }
                    topic_slab.insert(weak.clone());
                }

                // Index by service for Service/Client
                if matches!(endpoint.kind, EntityKind::Service | EntityKind::Client) {
                    let service_slab = data
                        .by_service
                        .entry(endpoint.topic.clone())
                        .or_insert_with(|| Slab::with_capacity(DEFAULT_SLAB_CAPACITY));

                    if service_slab.len() >= service_slab.capacity() {
                        service_slab.retain(|_, weak_ptr| weak_ptr.upgrade().is_some());
                    }
                    service_slab.insert(weak.clone());
                }

                // Index by node
                let node_slab = data
                    .by_node
                    .entry(endpoint.node.key())
                    .or_insert_with(|| Slab::with_capacity(DEFAULT_SLAB_CAPACITY));

                if node_slab.len() >= node_slab.capacity() {
                    node_slab.retain(|_, weak_ptr| weak_ptr.upgrade().is_some());
                }
                node_slab.insert(weak);
            }
        }

        // Release lock before triggering events
        drop(data);

        // Only trigger graph change event if this is a new entity
        // (to avoid double-counting when liveliness already triggered it)
        if !already_exists {
            self.event_manager.trigger_graph_change(&entity, true, self.zid);
        }

        Ok(())
    }

    /// Remove a local entity from the graph
    pub fn remove_local_entity(&self, entity: &Entity) -> Result<()> {
        let mut data = self.data.lock();

        // Create LivelinessKE from entity
        let ke = LivelinessKE::try_from(entity)?;

        // Remove from both cached and parsed
        data.cached.remove(&ke);
        data.parsed.remove(&ke);

        // Also remove from the index slabs (by_topic, by_service, by_node)
        // The slabs use Weak pointers which will fail to upgrade after we remove from parsed
        // But we need to explicitly remove them to prevent parse() from re-adding the entity
        match entity {
            Entity::Node(node_entity) => {
                if let Some(slab) = data.by_node.get_mut(&node_entity.key()) {
                    slab.retain(|_, weak| {
                        weak.upgrade().is_some_and(|arc| {
                            LivelinessKE::try_from(&*arc).ok().as_ref() != Some(&ke)
                        })
                    });
                }
            }
            Entity::Endpoint(endpoint_entity) => {
                // Remove from by_topic or by_service depending on kind
                if matches!(endpoint_entity.kind, EntityKind::Publisher | EntityKind::Subscription)
                    && let Some(slab) = data.by_topic.get_mut(&endpoint_entity.topic) {
                    slab.retain(|_, weak| {
                        weak.upgrade().is_some_and(|arc| {
                            LivelinessKE::try_from(&*arc).ok().as_ref() != Some(&ke)
                        })
                    });
                }
                if matches!(endpoint_entity.kind, EntityKind::Service | EntityKind::Client)
                    && let Some(slab) = data.by_service.get_mut(&endpoint_entity.topic) {
                    slab.retain(|_, weak| {
                        weak.upgrade().is_some_and(|arc| {
                            LivelinessKE::try_from(&*arc).ok().as_ref() != Some(&ke)
                        })
                    });
                }
                // Also remove from by_node (endpoints are indexed by their node)
                if let Some(slab) = data.by_node.get_mut(&endpoint_entity.node.key()) {
                    slab.retain(|_, weak| {
                        weak.upgrade().is_some_and(|arc| {
                            LivelinessKE::try_from(&*arc).ok().as_ref() != Some(&ke)
                        })
                    });
                }
            }
        }

        // Release lock before triggering events
        drop(data);

        // Trigger graph change event
        self.event_manager.trigger_graph_change(entity, false, self.zid);

        Ok(())
    }

    pub fn count(&self, kind: EntityKind, name: impl AsRef<str>) -> usize {
        assert!(kind != EntityKind::Node);
        let mut total = 0;
        match kind {
            EntityKind::Publisher | EntityKind::Subscription => {
                self.data.lock().visit_by_topic(name, |ent| {
                    if ent.kind() == kind {
                        total += 1;
                    }
                });
            }
            EntityKind::Service | EntityKind::Client => {
                self.data.lock().visit_by_service(name, |ent| {
                    if ent.kind() == kind {
                        total += 1;
                    }
                });
            }
            _ => unreachable!(),
        }
        total
    }

    pub fn get_entities_by_topic(
        &self,
        kind: EntityKind,
        topic: impl AsRef<str>,
    ) -> Vec<Arc<Entity>> {
        assert!(kind != EntityKind::Node);
        let mut res = Vec::new();
        self.data.lock().visit_by_topic(topic, |ent| {
            if ent.kind() == kind {
                res.push(ent);
            }
        });
        res
    }

    pub fn get_entities_by_node(&self, kind: EntityKind, node: NodeKey) -> Vec<EndpointEntity> {
        assert!(kind != EntityKind::Node);
        let mut res = Vec::new();
        self.data.lock().visit_by_node(node, |ent| {
            if ent.kind() == kind
                && let Entity::Endpoint(endpoint) = &*ent
            {
                res.push(endpoint.clone());
            }
        });
        res
    }

    pub fn count_by_service(&self, kind: EntityKind, service_name: impl AsRef<str>) -> usize {
        assert!(matches!(kind, EntityKind::Service | EntityKind::Client));
        let mut total = 0;
        self.data.lock().visit_by_service(service_name, |ent| {
            if ent.kind() == kind {
                total += 1;
            }
        });
        total
    }

    pub fn get_entities_by_service(
        &self,
        kind: EntityKind,
        service_name: impl AsRef<str>,
    ) -> Vec<Arc<Entity>> {
        assert!(matches!(kind, EntityKind::Service | EntityKind::Client));
        let mut res = Vec::new();
        self.data.lock().visit_by_service(service_name, |ent| {
            if ent.kind() == kind {
                res.push(ent);
            }
        });
        res
    }

    pub fn get_service_names_and_types(&self) -> Vec<(String, String)> {
        let mut res = Vec::new();
        let mut data = self.data.lock();

        if !data.cached.is_empty() {
            data.parse();
        }

        // Iterate directly over all services in by_service index
        for (service_name, slab) in &mut data.by_service {
            let mut found_type = None;
            slab.retain(|_, weak| {
                if let Some(ent) = weak.upgrade() {
                    // Skip expensive get_endpoint() if we already found the type
                    if let Some(enp) = ent.get_endpoint() && found_type.is_none() && enp.kind == EntityKind::Service {
                        found_type = enp.type_info.as_ref().map(|x| x.name.clone());
                    }
                    true
                } else {
                    false
                }
            });

            if let Some(type_name) = found_type {
                res.push((service_name.clone(), type_name));
            }
        }

        res
    }

    pub fn get_topic_names_and_types(&self) -> Vec<(String, String)> {
        let mut res = Vec::new();
        let mut data = self.data.lock();

        if !data.cached.is_empty() {
            data.parse();
        }

        // NOTE: Each topic has exactly one topic type
        // Iterate directly over all topics in by_topic index
        for (topic_name, slab) in &mut data.by_topic {
            let mut found_type = None;
            slab.retain(|_, weak| {
                if let Some(ent) = weak.upgrade() {
                    // Skip expensive get_endpoint() if we already found the type
                    if found_type.is_none()
                        && let Some(enp) = ent.get_endpoint()
                    {
                        // Include both publishers and subscribers
                        if matches!(enp.kind, EntityKind::Publisher | EntityKind::Subscription)
                            && let Some(type_info) = &enp.type_info {
                                found_type = Some(type_info.name.clone());
                            }
                    }
                    true
                } else {
                    false
                }
            });

            if let Some(type_name) = found_type {
                res.push((topic_name.clone(), type_name));
            }
        }

        res
    }

    pub fn get_names_and_types_by_node(
        &self,
        node_key: NodeKey,
        kind: EntityKind,
    ) -> Vec<(String, String)> {
        use std::collections::BTreeSet;

        // Use BTreeSet to deduplicate and sort results by (topic, type)
        // This matches rmw_zenoh_cpp behavior which uses std::map
        let mut res_set = BTreeSet::new();
        let mut data = self.data.lock();

        let node_ns = node_key.0.clone();
        let node_name = node_key.1.clone();

        tracing::debug!(
            "get_names_and_types_by_node: Looking for node_key=({:?}, {:?}), kind={:?}",
            node_ns, node_name, kind
        );

        if !data.cached.is_empty() {
            tracing::debug!("get_names_and_types_by_node: Parsing {} cached entries", data.cached.len());
            data.parse();
        }

        data.visit_by_node(node_key, |ent| {
            if let Some(enp) = ent.get_endpoint()
                && enp.kind == kind
                && let Some(type_info) = &enp.type_info {
                    // Insert into set for automatic deduplication
                    res_set.insert((enp.topic.clone(), type_info.name.clone()));
                }
        });

        let res: Vec<_> = res_set.into_iter().collect();

        tracing::debug!(
            "get_names_and_types_by_node: Returning {} topics for node ({:?}, {:?}), kind={:?}: {:?}",
            res.len(), node_ns, node_name, kind, res
        );

        res
    }

    /// Check if a node exists in the graph
    ///
    /// Returns true if the node exists, false otherwise
    pub fn node_exists(&self, node_key: NodeKey) -> bool {
        let mut data = self.data.lock();

        if !data.cached.is_empty() {
            data.parse();
        }

        data.by_node.contains_key(&node_key)
    }

    /// Get all node names and namespaces discovered in the graph
    ///
    /// Returns a vector of tuples (node_name, node_namespace)
    pub fn get_node_names(&self) -> Vec<(String, String)> {
        let mut data = self.data.lock();

        if !data.cached.is_empty() {
            data.parse();
        }

        // Extract all nodes from by_node HashMap
        // Return one entry per node instance (even if multiple nodes have same name/namespace)
        // Denormalize namespace: empty string becomes "/"
        let mut result = Vec::new();
        for ((namespace, name), slab) in data.by_node.iter() {
            let denormalized_ns = if namespace.is_empty() {
                "/".to_string()
            } else if !namespace.starts_with('/') {
                format!("/{}", namespace)
            } else {
                namespace.clone()
            };

            // Count each Node entity separately (not Endpoint entities)
            for (_, weak_entity) in slab.iter() {
                if let Some(entity_arc) = weak_entity.upgrade()
                    && matches!(&*entity_arc, Entity::Node(_))
                {
                    result.push((name.clone(), denormalized_ns.clone()));
                }
            }
        }
        result
    }

    /// Get all node names, namespaces, and enclaves discovered in the graph
    ///
    /// Returns a vector of tuples (node_name, node_namespace, enclave)
    pub fn get_node_names_with_enclaves(&self) -> Vec<(String, String, String)> {
        let mut data = self.data.lock();

        if !data.cached.is_empty() {
            data.parse();
        }

        // Extract all nodes from by_node HashMap
        // Return one entry per node instance (even if multiple nodes have same name/namespace)
        // Denormalize namespace: empty string becomes "/"
        let mut result = Vec::new();
        for ((namespace, name), slab) in data.by_node.iter() {
            let denormalized_ns = if namespace.is_empty() {
                "/".to_string()
            } else if !namespace.starts_with('/') {
                format!("/{}", namespace)
            } else {
                namespace.clone()
            };

            // Process each Node entity separately (not Endpoint entities)
            for (_, weak_entity) in slab.iter() {
                if let Some(entity_arc) = weak_entity.upgrade()
                    && let Entity::Node(node) = &*entity_arc
                {
                    let enclave = if node.enclave.is_empty() {
                        "/".to_string()
                    } else if !node.enclave.starts_with('/') {
                        format!("/{}", node.enclave)
                    } else {
                        node.enclave.clone()
                    };
                    result.push((name.clone(), denormalized_ns.clone(), enclave));
                }
            }
        }
        result
    }

    /// Get action client names and types by node
    ///
    /// Returns a vector of tuples (action_name, action_type) for action clients on the specified node
    ///
    /// This follows the ROS 2 approach: action clients subscribe to feedback topics,
    /// so we query subscribers and filter for topics with the "/_action/feedback" suffix.
    pub fn get_action_client_names_and_types_by_node(
        &self,
        node_key: NodeKey,
    ) -> Vec<(String, String)> {
        // Get all subscribers for this node
        let subscribers = self.get_names_and_types_by_node(node_key, EntityKind::Subscription);

        // Filter for action feedback topics and extract action name/type
        self.filter_action_names_and_types(subscribers)
    }

    /// Get action server names and types by node
    ///
    /// Returns a vector of tuples (action_name, action_type) for action servers on the specified node
    ///
    /// This follows the ROS 2 approach: action servers publish feedback topics,
    /// so we query publishers and filter for topics with the "/_action/feedback" suffix.
    pub fn get_action_server_names_and_types_by_node(
        &self,
        node_key: NodeKey,
    ) -> Vec<(String, String)> {
        // Get all publishers for this node
        let publishers = self.get_names_and_types_by_node(node_key, EntityKind::Publisher);

        // Filter for action feedback topics and extract action name/type
        self.filter_action_names_and_types(publishers)
    }

    /// Filter topic names and types to extract action names and types
    ///
    /// This helper method implements the ROS 2 filtering logic:
    /// - Looks for topics with the "/_action/feedback" suffix
    /// - Extracts the action name by removing the suffix
    /// - Extracts the action type by removing the "_FeedbackMessage" suffix from the type
    fn filter_action_names_and_types(&self, topics: Vec<(String, String)>) -> Vec<(String, String)> {
        const ACTION_NAME_SUFFIX: &str = "/_action/feedback";
        const ACTION_TYPE_SUFFIX: &str = "_FeedbackMessage";

        topics
            .into_iter()
            .filter_map(|(topic_name, type_name)| {
                // Check if topic name ends with "/_action/feedback"
                if topic_name.ends_with(ACTION_NAME_SUFFIX) {
                    // Extract action name by removing the suffix
                    let action_name = topic_name
                        .strip_suffix(ACTION_NAME_SUFFIX)
                        .unwrap()
                        .to_string();

                    // Extract action type by removing "_FeedbackMessage" suffix if present
                    let action_type = type_name
                        .strip_suffix(ACTION_TYPE_SUFFIX)
                        .unwrap_or(&type_name)
                        .to_string();

                    Some((action_name, action_type))
                } else {
                    None
                }
            })
            .collect()
    }

    /// Get all action names and types discovered in the graph
    ///
    /// Returns a vector of tuples (action_name, action_type) for all action clients and servers
    ///
    /// This follows the ROS 2 approach: we query all topics and filter for
    /// topics with the "/_action/feedback" suffix.
    pub fn get_action_names_and_types(&self) -> Vec<(String, String)> {
        // Get all topics
        let topics = self.get_topic_names_and_types();

        // Filter for action feedback topics and extract action name/type
        let mut res = self.filter_action_names_and_types(topics);

        // Remove duplicates (same action name/type may appear on multiple nodes)
        res.sort();
        res.dedup();
        res
    }

    /// Create a serializable snapshot of the current graph state
    ///
    /// This captures topics, nodes, and services with their metadata,
    /// suitable for JSON serialization or other export formats.
    pub fn snapshot(&self, domain_id: usize) -> GraphSnapshot {
        let topics: Vec<TopicSnapshot> = self
            .get_topic_names_and_types()
            .into_iter()
            .map(|(name, type_name)| {
                let publishers = self
                    .get_entities_by_topic(EntityKind::Publisher, &name)
                    .len();
                let subscribers = self
                    .get_entities_by_topic(EntityKind::Subscription, &name)
                    .len();
                TopicSnapshot {
                    name,
                    type_name,
                    publishers,
                    subscribers,
                }
            })
            .collect();

        let nodes: Vec<NodeSnapshot> = self
            .get_node_names()
            .into_iter()
            .map(|(name, namespace)| NodeSnapshot { name, namespace })
            .collect();

        let services: Vec<ServiceSnapshot> = self
            .get_service_names_and_types()
            .into_iter()
            .map(|(name, type_name)| ServiceSnapshot { name, type_name })
            .collect();

        GraphSnapshot {
            timestamp: SystemTime::now(),
            domain_id,
            topics,
            nodes,
            services,
        }
    }
}
