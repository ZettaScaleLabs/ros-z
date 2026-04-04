use std::time::Duration;

use color_eyre::eyre::Result;

use crate::{
    app::AppContext,
    model::watch::WatchEvent,
    render::{OutputMode, json, text},
    support::graph::diff_snapshots,
};

const WATCH_POLL_INTERVAL: Duration = Duration::from_millis(500);

pub async fn run(app: &AppContext, output_mode: OutputMode) -> Result<()> {
    app.wait_for_graph_settle().await;

    let mut previous = app.snapshot();
    match output_mode {
        OutputMode::Json => json::print_line(&WatchEvent::InitialState {
            snapshot: previous.clone(),
        })?,
        OutputMode::Text => text::print_graph_snapshot(&previous),
    }

    loop {
        tokio::time::sleep(WATCH_POLL_INTERVAL).await;

        let current = app.snapshot();
        for event in diff_snapshots(&previous, &current) {
            match output_mode {
                OutputMode::Json => json::print_line(&event)?,
                OutputMode::Text => text::print_watch_event(&event),
            }
        }
        previous = current;
    }
}
