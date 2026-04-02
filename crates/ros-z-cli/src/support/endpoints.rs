use std::{collections::BTreeSet, sync::Arc};

use ros_z::entity::{Entity, entity_get_endpoint};

use crate::{
    model::info::{EndpointSummary, NamedType},
    support::nodes::fully_qualified_node_name,
};

pub fn summarize_endpoints(entities: Vec<Arc<Entity>>) -> Vec<EndpointSummary> {
    let mut endpoints = BTreeSet::new();

    for entity in entities {
        if let Some(endpoint) = entity_get_endpoint(&entity) {
            let node = endpoint
                .node
                .as_ref()
                .map(|node| fully_qualified_node_name(&node.namespace, &node.name));
            let type_hash = endpoint
                .type_info
                .as_ref()
                .map(|type_info| type_info.hash.to_string());
            endpoints.insert((node, type_hash));
        }
    }

    endpoints
        .into_iter()
        .map(|(node, type_hash)| EndpointSummary { node, type_hash })
        .collect()
}

pub fn named_types(entries: Vec<(String, String)>) -> Vec<NamedType> {
    let unique: BTreeSet<_> = entries.into_iter().collect();
    unique
        .into_iter()
        .map(|(name, type_name)| NamedType::new(name, type_name))
        .collect()
}
