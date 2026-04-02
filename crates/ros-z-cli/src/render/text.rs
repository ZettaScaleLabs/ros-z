use ros_z::graph::GraphSnapshot;

use crate::{
    model::{
        echo::EchoHeader,
        graph::{NodeSummary, ServiceSummary, TopicSummary},
        info::{EndpointSummary, NamedType, NodeInfo, ServiceInfo, TopicInfo},
        param::{ParameterGetView, ParameterListView, ParameterSetView},
        watch::WatchEvent,
    },
    support::nodes::fully_qualified_node_name,
};

pub fn print_topic_summaries(topics: &[TopicSummary]) {
    let name_width = column_width(topics.iter().map(|topic| topic.name.as_str()));
    let type_width = column_width(topics.iter().map(|topic| topic.type_name.as_str()));

    for topic in topics {
        println!(
            "{:<name_width$}  {:<type_width$}  pubs={} subs={}",
            topic.name, topic.type_name, topic.publishers, topic.subscribers,
        );
    }
}

pub fn print_node_summaries(nodes: &[NodeSummary]) {
    for node in nodes {
        println!("{}", node.fqn);
    }
}

pub fn print_service_summaries(services: &[ServiceSummary]) {
    let name_width = column_width(services.iter().map(|service| service.name.as_str()));
    let type_width = column_width(services.iter().map(|service| service.type_name.as_str()));

    for service in services {
        println!(
            "{:<name_width$}  {:<type_width$}  servers={} clients={}",
            service.name, service.type_name, service.servers, service.clients,
        );
    }
}

pub fn print_graph_snapshot(snapshot: &GraphSnapshot) {
    println!("Domain {}", snapshot.domain_id);
    println!();

    println!("Topics ({})", snapshot.topics.len());
    let topics: Vec<_> = snapshot
        .topics
        .clone()
        .into_iter()
        .map(TopicSummary::from)
        .collect();
    print_topic_summaries(&topics);
    println!();

    println!("Nodes ({})", snapshot.nodes.len());
    let mut nodes: Vec<_> = snapshot
        .nodes
        .iter()
        .map(|node| NodeSummary::new(node.name.clone(), node.namespace.clone()))
        .collect();
    nodes.sort_by(|left, right| left.fqn.cmp(&right.fqn));
    print_node_summaries(&nodes);
    println!();

    println!("Services ({})", snapshot.services.len());
    let mut services: Vec<_> = snapshot
        .services
        .iter()
        .map(|service| ServiceSummary::new(service.name.clone(), service.type_name.clone(), 0, 0))
        .collect();
    services.sort_by(|left, right| left.name.cmp(&right.name));
    let name_width = column_width(services.iter().map(|service| service.name.as_str()));
    let type_width = column_width(services.iter().map(|service| service.type_name.as_str()));
    for service in services {
        println!(
            "{:<name_width$}  {:<type_width$}",
            service.name, service.type_name,
        );
    }
}

pub fn print_topic_info(info: &TopicInfo) {
    println!("Topic {}", info.name);
    println!("Type: {}", info.type_name);
    println!();
    print_endpoint_section("Publishers", &info.publishers);
    println!();
    print_endpoint_section("Subscribers", &info.subscribers);
}

pub fn print_service_info(info: &ServiceInfo) {
    println!("Service {}", info.name);
    println!("Type: {}", info.type_name);
    println!();
    print_endpoint_section("Servers", &info.servers);
    println!();
    print_endpoint_section("Clients", &info.clients);
}

pub fn print_node_info(info: &NodeInfo) {
    println!("Node {}", info.fqn);
    println!();
    print_named_type_section("Publishers", &info.publishers);
    println!();
    print_named_type_section("Subscribers", &info.subscribers);
    println!();
    print_named_type_section("Services", &info.services);
    println!();
    print_named_type_section("Clients", &info.clients);
    println!();
    print_named_type_section("Action servers", &info.action_servers);
    println!();
    print_named_type_section("Action clients", &info.action_clients);
}

pub fn print_parameter_list(view: &ParameterListView) {
    println!("Node: {}", view.node);
    if view.names.is_empty() {
        println!("Parameters: 0");
        return;
    }

    println!("Parameters ({})", view.names.len());
    for name in &view.names {
        println!("{name}");
    }

    if !view.prefixes.is_empty() {
        println!();
        println!("Prefixes ({})", view.prefixes.len());
        for prefix in &view.prefixes {
            println!("{prefix}");
        }
    }
}

pub fn print_parameter_get(view: &ParameterGetView) {
    println!("Node: {}", view.node);
    println!("Name: {}", view.name);
    println!("Type: {}", view.parameter.value_type);
    println!("Value: {}", view.parameter.display);
}

pub fn print_parameter_set(view: &ParameterSetView) {
    println!("Node: {}", view.node);
    println!("Name: {}", view.name);
    println!("Type: {}", view.parameter.value_type);
    println!("Value: {}", view.parameter.display);
    println!("Atomic: {}", view.atomic);
    println!("Updated: {}", view.successful);
}

pub fn print_echo_header(header: &EchoHeader) {
    println!("Topic: {}", header.topic);
    println!("Type: {}", header.type_name);
    println!("Hash: {}", header.type_hash);
    println!();
}

pub fn print_echo_message(message: &str, count: Option<usize>, seen: usize) {
    print!("{message}");
    if count.is_none_or(|limit| seen < limit) {
        println!();
    }
}

pub fn print_watch_event(event: &WatchEvent) {
    match event {
        WatchEvent::InitialState { snapshot } => print_graph_snapshot(snapshot),
        WatchEvent::TopicDiscovered { name, type_name } => {
            println!("topic + {name} ({type_name})");
        }
        WatchEvent::TopicRemoved { name } => {
            println!("topic - {name}");
        }
        WatchEvent::NodeDiscovered { namespace, name } => {
            println!("node + {}", fully_qualified_node_name(namespace, name));
        }
        WatchEvent::NodeRemoved { namespace, name } => {
            println!("node - {}", fully_qualified_node_name(namespace, name));
        }
        WatchEvent::ServiceDiscovered { name, type_name } => {
            println!("service + {name} ({type_name})");
        }
        WatchEvent::ServiceRemoved { name } => {
            println!("service - {name}");
        }
    }
}

fn print_endpoint_section(label: &str, endpoints: &[EndpointSummary]) {
    println!("{label} ({})", endpoints.len());
    if endpoints.is_empty() {
        println!("none");
        return;
    }

    for endpoint in endpoints {
        match (&endpoint.node, &endpoint.type_hash) {
            (Some(node), Some(type_hash)) => println!("{node} [{type_hash}]"),
            (Some(node), None) => println!("{node}"),
            (None, Some(type_hash)) => println!("unknown [{type_hash}]"),
            (None, None) => println!("unknown"),
        }
    }
}

fn print_named_type_section(label: &str, entries: &[NamedType]) {
    println!("{label} ({})", entries.len());
    if entries.is_empty() {
        println!("none");
        return;
    }

    let name_width = column_width(entries.iter().map(|entry| entry.name.as_str()));
    for entry in entries {
        println!("{:<name_width$}  {}", entry.name, entry.type_name);
    }
}

fn column_width<'a>(values: impl Iterator<Item = &'a str>) -> usize {
    values.map(str::len).max().unwrap_or(0)
}
