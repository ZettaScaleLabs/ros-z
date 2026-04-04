use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct EndpointSummary {
    pub node: Option<String>,
    pub type_hash: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct NamedType {
    pub name: String,
    #[serde(rename = "type")]
    pub type_name: String,
}

impl NamedType {
    pub fn new(name: String, type_name: String) -> Self {
        Self { name, type_name }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct TopicInfo {
    pub name: String,
    #[serde(rename = "type")]
    pub type_name: String,
    pub publishers: Vec<EndpointSummary>,
    pub subscribers: Vec<EndpointSummary>,
}

impl TopicInfo {
    pub fn new(
        name: String,
        type_name: String,
        publishers: Vec<EndpointSummary>,
        subscribers: Vec<EndpointSummary>,
    ) -> Self {
        Self {
            name,
            type_name,
            publishers,
            subscribers,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ServiceInfo {
    pub name: String,
    #[serde(rename = "type")]
    pub type_name: String,
    pub servers: Vec<EndpointSummary>,
    pub clients: Vec<EndpointSummary>,
}

impl ServiceInfo {
    pub fn new(
        name: String,
        type_name: String,
        servers: Vec<EndpointSummary>,
        clients: Vec<EndpointSummary>,
    ) -> Self {
        Self {
            name,
            type_name,
            servers,
            clients,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct NodeInfo {
    pub name: String,
    pub namespace: String,
    pub fqn: String,
    pub publishers: Vec<NamedType>,
    pub subscribers: Vec<NamedType>,
    pub services: Vec<NamedType>,
    pub clients: Vec<NamedType>,
    pub action_servers: Vec<NamedType>,
    pub action_clients: Vec<NamedType>,
}

impl NodeInfo {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        name: String,
        namespace: String,
        fqn: String,
        publishers: Vec<NamedType>,
        subscribers: Vec<NamedType>,
        services: Vec<NamedType>,
        clients: Vec<NamedType>,
        action_servers: Vec<NamedType>,
        action_clients: Vec<NamedType>,
    ) -> Self {
        Self {
            name,
            namespace,
            fqn,
            publishers,
            subscribers,
            services,
            clients,
            action_servers,
            action_clients,
        }
    }
}
