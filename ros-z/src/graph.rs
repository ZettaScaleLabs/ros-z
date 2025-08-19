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
                    // TODO: omit the clone of topic
                    let topic_slab = self.by_topic
                        .entry(x.topic.clone())
                        .or_insert_with(|| Slab::with_capacity(DEFAULT_SLAB_CAPACITY));
                    
                    // If slab is full, remove failing weak pointers first
                    if topic_slab.len() >= topic_slab.capacity() {
                        topic_slab.retain(|_, weak_ptr| weak_ptr.upgrade().is_some());
                    }
                    
                    topic_slab.insert(weak.clone());
                    
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

    pub fn get_entities_by_node(&self, kind: EntityKind, _node: NodeKey) -> Vec<EndpointEntity> {
        assert!(kind != EntityKind::Node);
        todo!()
        // self.data
        //     .read()
        //     .set
        //     .iter()
        //     .filter_map(|x| {
        //         if let Entity::Endpoint(ent) = x {
        //             if ent.node.name == node.0 && ent.node.namespace == node.1 {
        //                 return Some(ent.clone());
        //             }
        //         }
        //         None
        //     })
        //     .collect()
    }
}
