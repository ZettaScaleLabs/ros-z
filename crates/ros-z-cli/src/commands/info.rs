use color_eyre::eyre::{Result, eyre};
use ros_z::entity::EntityKind;

use crate::{
    app::AppContext,
    cli::InfoTarget,
    model::info::{NodeInfo, ServiceInfo, TopicInfo},
    render::{OutputMode, json, text},
    support::{
        endpoints::{named_types, summarize_endpoints},
        nodes::{can_resolve_node_target, graph_node_key, resolve_node_target},
    },
};

pub async fn run(
    app: &AppContext,
    output_mode: OutputMode,
    target: InfoTarget,
    name: &str,
) -> Result<()> {
    match target {
        InfoTarget::Topic => render_topic_info(app, output_mode, name).await,
        InfoTarget::Service => render_service_info(app, output_mode, name).await,
        InfoTarget::Node => render_node_info(app, output_mode, name).await,
    }
}

async fn render_topic_info(app: &AppContext, output_mode: OutputMode, topic: &str) -> Result<()> {
    app.wait_for_graph_condition(|graph| {
        graph
            .get_topic_names_and_types()
            .iter()
            .any(|(name, _)| name == topic)
    })
    .await;

    let graph = app.graph();
    let type_name = graph
        .get_topic_names_and_types()
        .into_iter()
        .find_map(|(name, type_name)| (name == topic).then_some(type_name))
        .ok_or_else(|| eyre!("topic not found: {topic}"))?;
    let info = TopicInfo::new(
        topic.to_string(),
        type_name,
        summarize_endpoints(graph.get_entities_by_topic(EntityKind::Publisher, topic)),
        summarize_endpoints(graph.get_entities_by_topic(EntityKind::Subscription, topic)),
    );

    match output_mode {
        OutputMode::Json => json::print_pretty(&info),
        OutputMode::Text => {
            text::print_topic_info(&info);
            Ok(())
        }
    }
}

async fn render_service_info(
    app: &AppContext,
    output_mode: OutputMode,
    service: &str,
) -> Result<()> {
    app.wait_for_graph_condition(|graph| {
        graph
            .get_service_names_and_types()
            .iter()
            .any(|(name, _)| name == service)
    })
    .await;

    let graph = app.graph();
    let type_name = graph
        .get_service_names_and_types()
        .into_iter()
        .find_map(|(name, type_name)| (name == service).then_some(type_name))
        .ok_or_else(|| eyre!("service not found: {service}"))?;
    let info = ServiceInfo::new(
        service.to_string(),
        type_name,
        summarize_endpoints(graph.get_entities_by_service(EntityKind::Service, service)),
        summarize_endpoints(graph.get_entities_by_service(EntityKind::Client, service)),
    );

    match output_mode {
        OutputMode::Json => json::print_pretty(&info),
        OutputMode::Text => {
            text::print_service_info(&info);
            Ok(())
        }
    }
}

async fn render_node_info(app: &AppContext, output_mode: OutputMode, selector: &str) -> Result<()> {
    app.wait_for_graph_condition(|graph| can_resolve_node_target(graph, selector))
        .await;

    let graph = app.graph();
    let target = resolve_node_target(graph, selector)?;
    let node_key = graph_node_key(&target);
    let info = NodeInfo::new(
        target.name.clone(),
        target.namespace.clone(),
        target.fully_qualified_name(),
        named_types(graph.get_names_and_types_by_node(node_key.clone(), EntityKind::Publisher)),
        named_types(graph.get_names_and_types_by_node(node_key.clone(), EntityKind::Subscription)),
        named_types(graph.get_names_and_types_by_node(node_key.clone(), EntityKind::Service)),
        named_types(graph.get_names_and_types_by_node(node_key.clone(), EntityKind::Client)),
        named_types(graph.get_action_server_names_and_types_by_node(node_key.clone())),
        named_types(graph.get_action_client_names_and_types_by_node(node_key)),
    );

    match output_mode {
        OutputMode::Json => json::print_pretty(&info),
        OutputMode::Text => {
            text::print_node_info(&info);
            Ok(())
        }
    }
}
