use serde::Serialize;

use crate::support::nodes::fully_qualified_node_name;

#[derive(Debug, Clone, Serialize)]
pub struct TopicSummary {
    pub name: String,
    #[serde(rename = "type")]
    pub type_name: String,
    pub publishers: usize,
    pub subscribers: usize,
}

impl From<ros_z::graph::TopicSnapshot> for TopicSummary {
    fn from(value: ros_z::graph::TopicSnapshot) -> Self {
        Self {
            name: value.name,
            type_name: value.type_name,
            publishers: value.publishers,
            subscribers: value.subscribers,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct NodeSummary {
    pub name: String,
    pub namespace: String,
    pub fqn: String,
}

impl NodeSummary {
    pub fn new(name: String, namespace: String) -> Self {
        let fqn = fully_qualified_node_name(&namespace, &name);
        Self {
            name,
            namespace,
            fqn,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ServiceSummary {
    pub name: String,
    #[serde(rename = "type")]
    pub type_name: String,
    pub servers: usize,
    pub clients: usize,
}

impl ServiceSummary {
    pub fn new(name: String, type_name: String, servers: usize, clients: usize) -> Self {
        Self {
            name,
            type_name,
            servers,
            clients,
        }
    }
}
