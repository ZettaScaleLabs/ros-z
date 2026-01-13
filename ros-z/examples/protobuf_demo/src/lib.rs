// ! Protobuf demo module
//!
//! This module demonstrates using protobuf messages with ros-z for both
//! pub/sub and service communication.
//!
//! Re-exports functions from example modules, allowing tests to import
//! and use the exact same code that users run.

pub mod pubsub;
pub mod service_client;
pub mod service_server;
pub mod types;

// Re-export main functions for easy access
pub use pubsub::run_pubsub_demo;
pub use service_client::run_service_client;
pub use service_server::run_service_server;

// Re-export types for tests
pub use types::{Calculate, CalculateRequest, CalculateResponse, SensorData};
