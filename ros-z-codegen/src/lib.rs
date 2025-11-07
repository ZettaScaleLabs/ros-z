pub mod roslibrust_adapter;
pub mod type_info_generator;
pub mod protobuf_adapter;

use std::path::{Path, PathBuf};
use anyhow::Result;

pub struct MessageGenerator {
    config: GeneratorConfig,
}

pub struct GeneratorConfig {
    /// Generate CDR-compatible serde types
    pub generate_cdr: bool,

    /// Generate protobuf definitions
    pub generate_protobuf: bool,

    /// Generate MessageTypeInfo trait impls
    pub generate_type_info: bool,

    pub output_dir: PathBuf,
}


impl MessageGenerator {
    pub fn new(config: GeneratorConfig) -> Self {
        Self { config }
    }

    /// Primary generation method - uses roslibrust for .msg files
    pub fn generate_from_msg_files(&self, packages: &[&Path]) -> Result<()> {
        if self.config.generate_cdr {
            roslibrust_adapter::generate_ros_messages(
                packages.to_vec(),
                &self.config.output_dir,
                self.config.generate_type_info,
            )?;
        }
        Ok(())
    }

    /// Protobuf-first generation
    pub fn generate_from_proto_files(&self, protos: &[protobuf_adapter::ProtoFile]) -> Result<()> {
        if self.config.generate_protobuf {
            let generator = protobuf_adapter::ProtobufMessageGenerator::new(&self.config.output_dir);
            generator.generate_from_proto(protos)?;
        }
        Ok(())
    }

    pub fn msg_to_proto(&self, msg_files: &[protobuf_adapter::MsgFile]) -> Result<Vec<protobuf_adapter::ProtoFile>> {
        let generator = protobuf_adapter::ProtobufMessageGenerator::new(&self.config.output_dir);
        generator.msg_to_proto(msg_files)
    }
}