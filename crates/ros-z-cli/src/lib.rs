mod app;
pub mod cli;
mod commands;
mod model;
mod render;
mod support;

use color_eyre::eyre::Result;

use crate::{
    app::AppContext,
    cli::{Cli, Command},
    render::OutputMode,
};

pub async fn run(cli: Cli) -> Result<()> {
    let output_mode = OutputMode::from_json_flag(cli.json);
    let app = AppContext::new(&cli.router, cli.domain, cli.backend)?;

    let result = match cli.command {
        Command::List { target } => commands::list::run(&app, output_mode, target).await,
        Command::Watch => commands::watch::run(&app, output_mode).await,
        Command::Graph => commands::graph::run(&app, output_mode).await,
        Command::Echo {
            topic,
            count,
            timeout,
        } => commands::echo::run(&app, output_mode, &topic, count, timeout).await,
        Command::Info { target, name } => {
            commands::info::run(&app, output_mode, target, &name).await
        }
        Command::Param { command } => commands::param::run(&app, output_mode, command).await,
    };
    let shutdown_result = app.shutdown();

    match (result, shutdown_result) {
        (Ok(()), Ok(())) => Ok(()),
        (Err(error), _) => Err(error),
        (Ok(()), Err(error)) => Err(error),
    }
}
