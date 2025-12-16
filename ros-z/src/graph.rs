use parking_lot::Mutex;
use slab::Slab;
use std::{
    collections::{HashMap, HashSet},
    sync::{Arc, Weak},
};

use crate::entity::{
    ADMIN_SPACE, EndpointEntity, Entity, EntityKind, LivelinessKE, NodeKey, Topic,
};
use crate::event::GraphEventManager;
use zenoh::{Result, Session, Wait, pubsub::Subscriber, sample::SampleKind, session::ZenohId};

const DEFAULT_SLAB_CAPACITY: usize = 128;

#[derive(Default, Debug)]
pub struct GraphData {
    cached: HashSet<LivelinessKE>,
    parsed: HashMap<LivelinessKE, Arc<Entity>>,
    by_topic: HashMap<Topic, Slab<Weak<Entity>>>,
    by_service: HashMap<Topic, Slab<Weak<Entity>>>,
    by_node: HashMap<NodeKey, Slab<Weak<Entity>>>,
}

impl GraphData {
    fn new() -> Self {
        Self::default()
    }

    fn insert(&mut self, ke: LivelinessKE) {
        self.cached.insert(ke);
    }

    fn remove(&mut self, ke: &LivelinessKE) {
        let in_cached = self.cached.remove(ke);
        let in_parsed = self.parsed.remove(ke);

        match (in_cached, in_parsed) {
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
        for ke in self.cached.drain() {
            // FIXME: unwrap
            let arc = Arc::new(Entity::try_from(&ke).unwrap());
            let weak = Arc::downgrade(&arc);
            match &*arc {
                Entity::Node(x) => {
                    // TODO: omit the clone of node key
                    let slab = self
                        .by_node
                        .entry(x.key())
                        .or_insert_with(|| Slab::with_capacity(DEFAULT_SLAB_CAPACITY));

                    // If slab is full, remove failing weak pointers first
                    if slab.len() >= slab.capacity() {
                        slab.retain(|_, weak_ptr| weak_ptr.upgrade().is_some());
                    }

                    slab.insert(weak);
                }
                Entity::Endpoint(x) => {
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
                        .entry(x.node.key())
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
        let zid = session.zid();
        let graph_data = Arc::new(Mutex::new(GraphData::new()));
        let event_manager = Arc::new(GraphEventManager::new());
        let c_graph_data = graph_data.clone();
        let c_event_manager = event_manager.clone();
        let c_zid = zid;
        let sub = session
            .liveliness()
            .declare_subscriber(format!("{ADMIN_SPACE}/{domain_id}/**"))
            .history(true)
            .callback(move |sample| {
                let mut graph_data_guard = c_graph_data.lock();
                let key_expr = sample.key_expr().to_owned();
                let ke = LivelinessKE(key_expr);
                match sample.kind() {
                    SampleKind::Put => {
                        graph_data_guard.insert(ke.clone());
                        // Trigger graph change events
                        if let Ok(entity) = Entity::try_from(&ke) {
                            c_event_manager.trigger_graph_change(&entity, true, c_zid);
                        }
                    }
                    SampleKind::Delete => {
                        // Trigger graph change events before removal
                        if let Ok(entity) = Entity::try_from(&ke) {
                            c_event_manager.trigger_graph_change(&entity, false, c_zid);
                        }
                        graph_data_guard.remove(&ke);
                    }
                }
            })
            .wait()?;
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
        let mut res = Vec::new();
        let mut data = self.data.lock();

        if !data.cached.is_empty() {
            data.parse();
        }

        data.visit_by_node(node_key, |ent| {
            if let Some(enp) = ent.get_endpoint()
                && enp.kind == kind
                && let Some(type_info) = &enp.type_info {
                    res.push((
                        enp.topic.clone(),
                        type_info.name.clone(),
                    ));
                }
        });

        res
    }

    /// Get all node names and namespaces discovered in the graph
    ///
    /// Returns a vector of tuples (node_name, node_namespace)
    pub fn get_node_names(&self) -> Vec<(String, String)> {
        let mut data = self.data.lock();

        if !data.cached.is_empty() {
            data.parse();
        }

        // Extract all node keys from by_node HashMap
        // Denormalize namespace: empty string becomes "/"
        data.by_node
            .keys()
            .map(|(namespace, name)| {
                let denormalized_ns = if namespace.is_empty() {
                    "/".to_string()
                } else if !namespace.starts_with('/') {
                    format!("/{}", namespace)
                } else {
                    namespace.clone()
                };
                (name.clone(), denormalized_ns)
            })
            .collect()
    }

    /// Get all node names, namespaces, and enclaves discovered in the graph
    ///
    /// Returns a vector of tuples (node_name, node_namespace, enclave)
    /// Note: Enclave information is not currently tracked, so empty string is returned
    pub fn get_node_names_with_enclaves(&self) -> Vec<(String, String, String)> {
        let mut data = self.data.lock();

        if !data.cached.is_empty() {
            data.parse();
        }

        // Extract all node keys from by_node HashMap
        // Denormalize namespace: empty string becomes "/"
        // FIXME: For now, enclave is always empty string as we don't track this yet
        data.by_node
            .keys()
            .map(|(namespace, name)| {
                let denormalized_ns = if namespace.is_empty() {
                    "/".to_string()
                } else if !namespace.starts_with('/') {
                    format!("/{}", namespace)
                } else {
                    namespace.clone()
                };
                (name.clone(), denormalized_ns, String::new())
            })
            .collect()
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
}
