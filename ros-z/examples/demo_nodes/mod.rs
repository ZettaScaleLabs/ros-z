//! Demo nodes module
//!
//! This module re-exports the functions from the example files,
//! allowing tests to import and use the exact same code that users run.

// Modules that require external_msgs feature (example_interfaces, action_tutorials_interfaces)
#[cfg(feature = "external_msgs")]
pub mod add_two_ints_client;
#[cfg(feature = "external_msgs")]
pub mod add_two_ints_server;
#[cfg(feature = "external_msgs")]
pub mod fibonacci_action_client;
#[cfg(feature = "external_msgs")]
pub mod fibonacci_action_server;

// Modules that work with bundled messages only
pub mod listener;
pub mod talker;

// Re-export the main functions for easy access
#[cfg(feature = "external_msgs")]
pub use add_two_ints_client::run_add_two_ints_client;
#[cfg(feature = "external_msgs")]
pub use add_two_ints_server::run_add_two_ints_server;
#[cfg(feature = "external_msgs")]
pub use fibonacci_action_client::run_fibonacci_action_client;
#[cfg(feature = "external_msgs")]
pub use fibonacci_action_server::run_fibonacci_action_server;
pub use listener::run_listener;
pub use talker::run_talker;
