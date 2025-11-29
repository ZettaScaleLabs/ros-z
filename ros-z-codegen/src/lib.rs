#[cfg(feature = "protobuf")]
pub mod protobuf_adapter;
pub mod roslibrust_adapter;

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
        // Generate CDR-compatible types
        if self.config.generate_cdr {
            roslibrust_adapter::generate_ros_messages(
                packages.to_vec(),
                &self.config.output_dir,
                self.config.generate_type_info,
            )?;
        }

        // Generate protobuf types
        #[cfg(feature = "protobuf")]
        if self.config.generate_protobuf {
            // Parse messages for protobuf generation
            let package_paths: Vec<PathBuf> = packages.iter().map(|p| p.to_path_buf()).collect();
            let (messages, services, _actions) =
                roslibrust_codegen::find_and_parse_ros_messages(&package_paths)?;

            // Filter out old-style actionlib messages and wstring messages (same as in roslibrust_adapter)
            let messages: Vec<_> = messages
                .into_iter()
                .filter(|msg| {
                    let full_name = msg.get_full_name();

                    // Filter out actionlib_msgs and old-style Action messages
                    if full_name.starts_with("actionlib_msgs/")
                        || full_name.ends_with("Action")
                        || full_name.ends_with("ActionGoal")
                        || full_name.ends_with("ActionResult")
                        || full_name.ends_with("ActionFeedback")
                    {
                        return false;
                    }

                    // Filter out messages with wstring fields
                    let has_wstring = msg
                        .fields
                        .iter()
                        .any(|field| field.field_type.to_string().contains("wstring"));

                    !has_wstring
                })
                .collect();

            let services: Vec<_> = services
                .into_iter()
                .filter(|srv| {
                    let full_name = format!("{}/{}", srv.package, srv.name);
                    !full_name.starts_with("actionlib_msgs/")
                })
                .collect();

            let (resolved_msgs, _resolved_srvs) =
                roslibrust_codegen::resolve_dependency_graph(messages, services)?;

            let proto_dir = self.config.output_dir.join("proto");
            let generator = protobuf_adapter::ProtobufMessageGenerator::new(&proto_dir);

            // Generate .proto files
            let proto_files = generator.generate_proto_files(&resolved_msgs)?;

            // Generate Rust code from .proto files
            let proto_output = self.config.output_dir.join("generated_proto.rs");
            generator.generate_rust_from_proto(&proto_files, &proto_output)?;

            // Generate MessageTypeInfo implementations
            let type_info_impls = generator.generate_type_info_impls(&resolved_msgs)?;

            // Append type info implementations to the proto output
            let mut proto_code = std::fs::read_to_string(&proto_output)?;
            proto_code.push('\n');
            proto_code.push_str(&type_info_impls);
            std::fs::write(&proto_output, proto_code)?;

            println!(
                "cargo:info=Generated {} protobuf types",
                resolved_msgs.len()
            );
        }

        Ok(())
    }
}
