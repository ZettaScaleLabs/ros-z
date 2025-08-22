use parking_lot::Mutex;
use slab::Slab;
use std::{
    collections::{HashMap, HashSet},
    ffi::CString,
    sync::{Arc, Weak},
};

use crate::entity::{
    ADMIN_SPACE, EndpointEntity, Entity, EntityKind, LivelinessKE, NodeKey, Topic,
};
use zenoh::{Result, Session, Wait, pubsub::Subscriber, sample::SampleKind};

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
        match (self.cached.remove(ke), self.parsed.remove(ke)) {
            (true, Some(_)) => unreachable!(),
            (false, None) => unreachable!(),
            _ => {}
        }
        // TODO: clean up the maps
    }

    fn parse(&mut self) {
        for ke in self.cached.drain() {
            // FIXME: unwrap
            let arc = Arc::new(Entity::try_from(&ke).unwrap());
            let weak = Arc::downgrade(&arc);
            match &*arc {
                Entity::Node(x) => {
                    // TODO: omit the clone of node key
                    let slab = self.by_node
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
                        let topic_slab = self.by_topic
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
                        let service_slab = self.by_service
                            .entry(x.topic.clone())
                            .or_insert_with(|| Slab::with_capacity(DEFAULT_SLAB_CAPACITY));

                        // If slab is full, remove failing weak pointers first
                        if service_slab.len() >= service_slab.capacity() {
                            service_slab.retain(|_, weak_ptr| weak_ptr.upgrade().is_some());
                        }

                        service_slab.insert(weak.clone());
                    }

                    let node_slab = self.by_node
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
        // dbg!(&self, &node_key);
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
    _subscriber: Subscriber<()>,
}

impl Graph {
    pub fn new(session: &Session, domain_id: usize) -> Result<Self> {
        let graph_data = Arc::new(Mutex::new(GraphData::new()));
        let c_graph_data = graph_data.clone();
        let sub = session
            .liveliness()
            .declare_subscriber(format!("{ADMIN_SPACE}/{domain_id}/**"))
            .history(true)
            .callback(move |sample| {
                let mut graph_data_guard = c_graph_data.lock();
                let ke = LivelinessKE(sample.key_expr().to_owned());
                match sample.kind() {
                    SampleKind::Put => graph_data_guard.insert(ke),
                    SampleKind::Delete => graph_data_guard.remove(&ke),
                }
            })
            .wait()?;
        Ok(Self {
            _subscriber: sub,
            data: graph_data,
        })
    }

    pub fn count(&self, kind: EntityKind, topic: impl AsRef<str>) -> usize {
        assert!(kind != EntityKind::Node);
        let mut total = 0;
        self.data.lock().visit_by_topic(topic, |ent| {
            if ent.kind() == kind {
                total += 1;
            }
        });
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
            if ent.kind() == kind {
                if let Entity::Endpoint(endpoint) = &*ent {
                    res.push(endpoint.clone());
                }
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
                    if found_type.is_none() {
                        if let Some(enp) = ent.get_endpoint() {
                            // Include both services and clients
                            if matches!(enp.kind, EntityKind::Service | EntityKind::Client) {
                                found_type = Some(enp.type_info.as_ref().unwrap().name.clone());
                            }
                        }
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
                    if found_type.is_none() {
                        if let Some(enp) = ent.get_endpoint() {
                            // Include both publishers and subscribers
                            if matches!(enp.kind, EntityKind::Publisher | EntityKind::Subscription) {
                                found_type = Some(enp.type_info.as_ref().unwrap().name.clone());
                            }
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

    pub fn get_names_and_types_by_node(&self, node_key: NodeKey, kind: EntityKind) -> Vec<(String, String)> {
        let mut res = Vec::new();
        let mut data = self.data.lock();
        
        if !data.cached.is_empty() {
            data.parse();
        }

        data.visit_by_node(node_key, |ent| {
            if let Some(enp) = ent.get_endpoint() {
                if enp.kind == kind {
                    res.push((enp.topic.clone(), enp.type_info.as_ref().unwrap().name.clone()));
                }
            }
        });

        res
    }
}
