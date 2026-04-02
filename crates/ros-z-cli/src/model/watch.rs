use ros_z::graph::GraphSnapshot;
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "event", rename_all = "snake_case")]
pub enum WatchEvent {
    InitialState { snapshot: GraphSnapshot },
    TopicDiscovered { name: String, type_name: String },
    TopicRemoved { name: String },
    NodeDiscovered { namespace: String, name: String },
    NodeRemoved { namespace: String, name: String },
    ServiceDiscovered { name: String, type_name: String },
    ServiceRemoved { name: String },
}
