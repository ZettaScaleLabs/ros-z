use color_eyre::eyre::Result;
use ros_z::entity::EntityKind;

use crate::{
    app::AppContext,
    cli::ListTarget,
    model::graph::{NodeSummary, ServiceSummary, TopicSummary},
    render::{OutputMode, json, text},
};

pub async fn run(app: &AppContext, output_mode: OutputMode, target: ListTarget) -> Result<()> {
    app.wait_for_graph_settle().await;

    match target {
        ListTarget::Topics => render_topics(output_mode, app),
        ListTarget::Nodes => render_nodes(output_mode, app),
        ListTarget::Services => render_services(output_mode, app),
    }
}

fn render_topics(output_mode: OutputMode, app: &AppContext) -> Result<()> {
    let mut topics: Vec<_> = app
        .snapshot()
        .topics
        .into_iter()
        .map(TopicSummary::from)
        .collect();
    topics.sort_by(|left, right| left.name.cmp(&right.name));

    match output_mode {
        OutputMode::Json => json::print_pretty(&topics),
        OutputMode::Text => {
            text::print_topic_summaries(&topics);
            Ok(())
        }
    }
}

fn render_nodes(output_mode: OutputMode, app: &AppContext) -> Result<()> {
    let mut nodes: Vec<_> = app
        .graph()
        .get_node_names()
        .into_iter()
        .map(|(name, namespace)| NodeSummary::new(name, namespace))
        .collect();
    nodes.sort_by(|left, right| left.fqn.cmp(&right.fqn));

    match output_mode {
        OutputMode::Json => json::print_pretty(&nodes),
        OutputMode::Text => {
            text::print_node_summaries(&nodes);
            Ok(())
        }
    }
}

fn render_services(output_mode: OutputMode, app: &AppContext) -> Result<()> {
    let graph = app.graph();
    let mut services: Vec<_> = graph
        .get_service_names_and_types()
        .into_iter()
        .map(|(name, type_name)| {
            ServiceSummary::new(
                name.clone(),
                type_name,
                graph.count_by_service(EntityKind::Service, &name),
                graph.count_by_service(EntityKind::Client, &name),
            )
        })
        .collect();
    services.sort_by(|left, right| left.name.cmp(&right.name));

    match output_mode {
        OutputMode::Json => json::print_pretty(&services),
        OutputMode::Text => {
            text::print_service_summaries(&services);
            Ok(())
        }
    }
}
