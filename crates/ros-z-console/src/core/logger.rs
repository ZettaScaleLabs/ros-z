use std::fs::OpenOptions;
use tracing_subscriber::{EnvFilter, fmt, prelude::*};

const LOG_FILE: &str = "ros-z-console.log";

pub fn init_logger(json_mode: bool, debug: bool) {
    let filter = if debug {
        EnvFilter::new("ros_z=debug,zenoh=debug")
    } else {
        EnvFilter::new("ros_z=info,zenoh=warn")
    };

    // Open log file for appending
    let log_file = match OpenOptions::new().create(true).append(true).open(LOG_FILE) {
        Ok(file) => file,
        Err(_) => return, // Silently fail if we can't open log file
    };

    if json_mode {
        // Structured JSON logs to file
        tracing_subscriber::registry()
            .with(filter)
            .with(
                fmt::layer()
                    .json()
                    .with_target(false)
                    .with_current_span(false)
                    .with_writer(log_file)
                    .with_ansi(false),
            )
            .init();
    } else {
        // Human-readable logs to file
        tracing_subscriber::registry()
            .with(filter)
            .with(
                fmt::layer()
                    .compact()
                    .with_writer(log_file)
                    .with_ansi(false),
            )
            .init();
    }
}
