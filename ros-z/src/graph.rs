use parking_lot::RwLock;
use std::{
    collections::{HashMap, HashSet},
    ops::Deref,
    sync::Arc,
};

use crate::entity::{
    ADMIN_SPACE, EndpointEntity, Entity, EntityKind, LivelinessKE, NodeKey, Topic,
};
use zenoh::{Result, Session, Wait, pubsub::Subscriber, sample::SampleKind};

#[derive(Default, Debug)]
pub struct GraphData {
    // by_node: HashMap<NodeKey, Vec<Entity>>,
    // by_topic: HashMap<Topic, Vec<EndpointEntity>>,
    // set: HashSet<Arc<Entity>>,
    ke_set: HashSet<LivelinessKE>,
}

impl GraphData {
    fn new() -> Self {
        Self::default()
    }

    fn insert(&mut self, ke: LivelinessKE) {
        self.ke_set.insert(ke);
    }

    fn remove(&mut self, ke: &LivelinessKE) {
        self.ke_set.remove(ke);
    }

    // fn parse_iter(&mut self) -> impl Iterator<Item = &Entity> {
    //     self.ke_map.iter_mut().map(|(k, v)| match v {
    //         Some(v) => v,
    //         None => match Entity::try_from(k) {
    //             Ok(ent) => {
    //                 v.replace(ent);
    //                 v.as_ref().unwrap()
    //             }
    //             Err(err) => {
    //                 panic!("Failed to iterate ke_map due to {err:?}")
    //             }
    //         },
    //     })
    // }
}

pub struct Graph {
    data: Arc<RwLock<GraphData>>,
    _subscriber: Subscriber<()>,
}

impl Graph {
    pub fn new(session: &Session, domain_id: usize) -> Result<Self> {
        let graph_data = Arc::new(RwLock::new(GraphData::new()));
        let c_graph_data = graph_data.clone();
        let sub = session
            .liveliness()
            .declare_subscriber(format!("{ADMIN_SPACE}/{domain_id}/**"))
            .history(true)
            .callback(move |sample| {
                let mut graph_data_guard = c_graph_data.write();
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

    pub fn count(&mut self, kind: EntityKind, topic: Topic) -> usize {
        assert!(kind != EntityKind::Node);
        todo!()
        // self.data
        //     .read()
        //     .parse_iter()
        //     .filter(|&x| {
        //         if let Entity::Endpoint(ent) = x {
        //             ent.topic == topic && ent.kind == kind
        //         } else {
        //             false
        //         }
        //     })
        //     .count()
    }

    pub fn get_entities_by_node(&self, kind: EntityKind, node: NodeKey) -> Vec<EndpointEntity> {
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

    pub fn get_entities_by_topic(&self, kind: EntityKind, topic: Topic) -> Vec<EndpointEntity> {
        assert!(kind != EntityKind::Node);
        todo!()
        // self.data
        //     .read()
        //     .set
        //     .iter()
        //     .filter_map(|x| {
        //         if let Entity::Endpoint(ent) = x {
        //             if ent.topic == topic {
        //                 return Some(ent.clone());
        //             }
        //         }
        //         None
        //     })
        //     .collect()
    }
}
