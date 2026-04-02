use color_eyre::eyre::Result;

use crate::{
    app::AppContext,
    render::{OutputMode, json, text},
};

pub async fn run(app: &AppContext, output_mode: OutputMode) -> Result<()> {
    app.wait_for_graph_settle().await;
    let snapshot = app.snapshot();

    match output_mode {
        OutputMode::Json => json::print_pretty(&snapshot),
        OutputMode::Text => {
            text::print_graph_snapshot(&snapshot);
            Ok(())
        }
    }
}
