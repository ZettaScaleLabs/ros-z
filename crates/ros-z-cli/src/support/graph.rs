use std::collections::{BTreeMap, BTreeSet};

use ros_z::graph::GraphSnapshot;

use crate::model::watch::WatchEvent;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SnapshotFingerprint {
    topics: Vec<(String, String, usize, usize)>,
    nodes: Vec<(String, String)>,
    services: Vec<(String, String)>,
}

impl From<&GraphSnapshot> for SnapshotFingerprint {
    fn from(snapshot: &GraphSnapshot) -> Self {
        let mut topics: Vec<_> = snapshot
            .topics
            .iter()
            .map(|topic| {
                (
                    topic.name.clone(),
                    topic.type_name.clone(),
                    topic.publishers,
                    topic.subscribers,
                )
            })
            .collect();
        topics.sort();

        let mut nodes: Vec<_> = snapshot
            .nodes
            .iter()
            .map(|node| (node.namespace.clone(), node.name.clone()))
            .collect();
        nodes.sort();

        let mut services: Vec<_> = snapshot
            .services
            .iter()
            .map(|service| (service.name.clone(), service.type_name.clone()))
            .collect();
        services.sort();

        Self {
            topics,
            nodes,
            services,
        }
    }
}

pub fn diff_snapshots(previous: &GraphSnapshot, current: &GraphSnapshot) -> Vec<WatchEvent> {
    let mut events = Vec::new();

    let previous_topics: BTreeMap<_, _> = previous
        .topics
        .iter()
        .map(|topic| (topic.name.clone(), topic.type_name.clone()))
        .collect();
    let current_topics: BTreeMap<_, _> = current
        .topics
        .iter()
        .map(|topic| (topic.name.clone(), topic.type_name.clone()))
        .collect();
    for (name, type_name) in &current_topics {
        if !previous_topics.contains_key(name) {
            events.push(WatchEvent::TopicDiscovered {
                name: name.clone(),
                type_name: type_name.clone(),
            });
        }
    }
    for name in previous_topics.keys() {
        if !current_topics.contains_key(name) {
            events.push(WatchEvent::TopicRemoved { name: name.clone() });
        }
    }

    let previous_nodes: BTreeSet<_> = previous
        .nodes
        .iter()
        .map(|node| (node.namespace.clone(), node.name.clone()))
        .collect();
    let current_nodes: BTreeSet<_> = current
        .nodes
        .iter()
        .map(|node| (node.namespace.clone(), node.name.clone()))
        .collect();
    for (namespace, name) in &current_nodes {
        if !previous_nodes.contains(&(namespace.clone(), name.clone())) {
            events.push(WatchEvent::NodeDiscovered {
                namespace: namespace.clone(),
                name: name.clone(),
            });
        }
    }
    for (namespace, name) in &previous_nodes {
        if !current_nodes.contains(&(namespace.clone(), name.clone())) {
            events.push(WatchEvent::NodeRemoved {
                namespace: namespace.clone(),
                name: name.clone(),
            });
        }
    }

    let previous_services: BTreeMap<_, _> = previous
        .services
        .iter()
        .map(|service| (service.name.clone(), service.type_name.clone()))
        .collect();
    let current_services: BTreeMap<_, _> = current
        .services
        .iter()
        .map(|service| (service.name.clone(), service.type_name.clone()))
        .collect();
    for (name, type_name) in &current_services {
        if !previous_services.contains_key(name) {
            events.push(WatchEvent::ServiceDiscovered {
                name: name.clone(),
                type_name: type_name.clone(),
            });
        }
    }
    for name in previous_services.keys() {
        if !current_services.contains_key(name) {
            events.push(WatchEvent::ServiceRemoved { name: name.clone() });
        }
    }

    events
}
