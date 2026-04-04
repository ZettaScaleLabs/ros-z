use clap::Parser;
use color_eyre::eyre::{Result, WrapErr};

use ros_z_cli::{cli::Cli, run};

#[tokio::main]
async fn main() -> Result<()> {
    color_eyre::install().wrap_err("failed to install color-eyre")?;
    run(Cli::parse()).await
}
