use chrono::Utc;
use serde::Serialize;

use crate::core::engine::CoreEngine;
use ros_z::graph::GraphSnapshot;

/// Wrapper for initial state that adds the event field
#[derive(Serialize)]
struct InitialStateEvent {
    event: &'static str,
    #[serde(flatten)]
    snapshot: GraphSnapshot,
}

pub async fn run_headless_mode(
    core: &CoreEngine,
    json: bool,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    tracing::info!("Starting headless mode - streaming events to stdout");

    let mut event_rx = core.subscribe_events();

    // Initial state dump
    if json {
        let graph = core.graph.lock();
        let snapshot = graph.snapshot(core.domain_id);
        let event = InitialStateEvent {
            event: "initial_state",
            snapshot,
        };
        println!("{}", serde_json::to_string(&event)?);
    } else {
        print_initial_state(core);
    }

    // Stream events
    while let Ok(event) = event_rx.recv().await {
        if json {
            println!("{}", event.to_json());
        } else {
            println!(
                "[{}] {}",
                Utc::now().format("%Y-%m-%d %H:%M:%S"),
                event.to_human_readable()
            );
        }
    }

    Ok(())
}

fn print_initial_state(core: &CoreEngine) {
    let graph = core.graph.lock();
    let topics = graph.get_topic_names_and_types();
    let services = graph.get_service_names_and_types();
    let nodes = graph.get_node_names();

    println!("Discovered Topics:");
    for (topic, type_name) in &topics {
        println!("  📡 {} ({})", topic, type_name);
    }
    if topics.is_empty() {
        println!("  (none)");
    }

    println!("Discovered Services:");
    for (service, type_name) in &services {
        println!("  🔌 {} ({})", service, type_name);
    }
    if services.is_empty() {
        println!("  (none)");
    }

    println!("Discovered Nodes:");
    for (name, namespace) in &nodes {
        println!("  🤖 {}/{}", namespace, name);
    }
    if nodes.is_empty() {
        println!("  (none)");
    }
}
