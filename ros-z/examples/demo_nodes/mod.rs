//! Demo nodes module
//!
//! This module re-exports the functions from the example files,
//! allowing tests to import and use the exact same code that users run.
pub mod talker;
pub mod listener;

// Re-export the main functions for easy access
pub use talker::run_talker;
pub use listener::run_listener;
