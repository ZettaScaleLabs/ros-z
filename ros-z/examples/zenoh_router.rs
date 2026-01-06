//! ROS-Z Router - Drop-in replacement for rmw_zenoh_cpp router
//!
//! This example provides a Zenoh router configured with ROS 2 compatible settings
//! that match the rmw_zenoh_cpp configuration.
//!
//! Usage:
//!   cargo run --example zenoh_router          # Default: tcp/[::]:7447
//!   cargo run --example zenoh_router -- --port 7448
//!   cargo run --example zenoh_router -- --endpoint "tcp/192.168.1.1:7447"
//!   cargo run --example zenoh_router -- --config /path/to/config.json5
//!
//! Enable logging with RUST_LOG:
//!   RUST_LOG=info cargo run --example zenoh_router
//!   RUST_LOG=debug cargo run --example zenoh_router

use clap::Parser;
use ros_z::config::RouterConfigBuilder;
use zenoh::Wait;

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Port to listen on (default: 7447)
    #[arg(short, long, default_value_t = 7447)]
    port: u16,

    /// Custom endpoint to listen on (overrides --port)
    /// Example: "tcp/192.168.1.1:7447"
    #[arg(short, long)]
    endpoint: Option<String>,

    /// Path to a JSON5 config file to load
    #[arg(short, long)]
    config: Option<String>,
}

fn main() {
    let args = Args::parse();

    // Initialize Zenoh logger (respects RUST_LOG env var, defaults to "error")
    zenoh::init_log_from_env_or("error");

    // Build router configuration
    let config = if let Some(config_path) = &args.config {
        zenoh::Config::from_file(config_path).expect("Failed to load config from file")
    } else if let Some(endpoint) = &args.endpoint {
        RouterConfigBuilder::new()
            .with_listen_endpoint(endpoint)
            .build()
            .expect("Failed to build router config")
    } else {
        RouterConfigBuilder::new()
            .with_listen_port(args.port)
            .build()
            .expect("Failed to build router config")
    };

    // Extract listen endpoint for display
    let listen_info = if let Some(config_path) = &args.config {
        format!("Using config file: {}", config_path)
    } else if let Some(endpoint) = &args.endpoint {
        format!("Listening: {}", endpoint)
    } else {
        format!("Listening: tcp/[::]:{}", args.port)
    };

    println!("ROS-Z Router (rmw_zenoh_cpp compatible)");
    println!("{}", listen_info);
    println!("Run your ROS-Z examples in other terminals now!");
    println!("Example: cargo run --example demo_nodes_talker");
    println!("Press Ctrl-C to stop");
    println!();

    // Open Zenoh session in router mode
    let session = zenoh::open(config)
        .wait()
        .expect("Failed to open Zenoh session");

    // Print session info
    let zid = session.zid();
    println!("Router started with ZID: {}", zid);
    println!();

    // Create runtime for async operations
    let runtime = tokio::runtime::Runtime::new().expect("Failed to create Tokio runtime");
    runtime.block_on(async {
        // Wait for Ctrl-C
        tokio::signal::ctrl_c()
            .await
            .expect("Failed to listen for Ctrl-C");
    });

    println!();
    println!("Router shutting down...");

    // Session will be closed when dropped
    drop(session);
}
