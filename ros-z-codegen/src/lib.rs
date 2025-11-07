pub mod roslibrust_adapter;
pub mod type_info_generator;

pub use roslibrust_adapter::generate_ros_messages;
pub use type_info_generator::TypeInfoGenerator;

use anyhow::Result;
use std::path::Path;

/// Configuration for message generation
pub struct GeneratorConfig {
    /// Generate MessageTypeInfo trait implementations
    pub generate_type_info: bool,
    /// Output directory for generated code
    pub output_dir: std::path::PathBuf,
}

impl Default for GeneratorConfig {
    fn default() -> Self {
        Self {
            generate_type_info: true,
            output_dir: std::env::var("OUT_DIR")
                .map(std::path::PathBuf::from)
                .unwrap_or_else(|_| std::path::PathBuf::from(".")),
        }
    }
}

/// Main message generator
pub struct MessageGenerator {
    config: GeneratorConfig,
}

impl MessageGenerator {
    pub fn new(config: GeneratorConfig) -> Self {
        Self { config }
    }

    /// Generate message types from ROS packages
    pub fn generate_from_packages(&self, packages: &[&Path]) -> Result<()> {
        roslibrust_adapter::generate_ros_messages(
            packages.to_vec(),
            &self.config.output_dir,
            self.config.generate_type_info,
        )
    }
}
