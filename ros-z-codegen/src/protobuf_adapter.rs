use anyhow::{Context, Result};
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

/// Adapter for generating protobuf definitions and Rust code from ROS messages
pub struct ProtobufMessageGenerator {
    proto_dir: PathBuf,
}

impl ProtobufMessageGenerator {
    pub fn new(proto_dir: &Path) -> Self {
        Self {
            proto_dir: proto_dir.to_path_buf(),
        }
    }

    /// Generate .proto files from parsed ROS message files
    pub fn generate_proto_files(
        &self,
        messages: &[roslibrust_codegen::MessageFile],
    ) -> Result<Vec<PathBuf>> {
        // Create proto directory if it doesn't exist
        fs::create_dir_all(&self.proto_dir).context("Failed to create proto directory")?;

        let mut proto_files = Vec::new();

        // Group messages by package
        let mut packages: BTreeMap<String, Vec<&roslibrust_codegen::MessageFile>> = BTreeMap::new();
        for msg in messages {
            packages
                .entry(msg.parsed.package.clone())
                .or_default()
                .push(msg);
        }

        // Generate one .proto file per package
        for (package, msgs) in packages {
            let proto_content = self.generate_proto_for_package(&package, &msgs)?;
            let proto_file_name = format!("{}.proto", package.replace("-", "_"));
            let proto_file_path = self.proto_dir.join(&proto_file_name);

            fs::write(&proto_file_path, proto_content)
                .with_context(|| format!("Failed to write proto file: {:?}", proto_file_path))?;

            proto_files.push(proto_file_path);
        }

        Ok(proto_files)
    }

    /// Generate protobuf definition for a package
    fn generate_proto_for_package(
        &self,
        package: &str,
        messages: &[&roslibrust_codegen::MessageFile],
    ) -> Result<String> {
        let mut proto = String::new();

        // Proto3 syntax
        proto.push_str("syntax = \"proto3\";\n\n");

        // Package name (convert hyphens to underscores for proto compatibility)
        proto.push_str(&format!("package {};\n\n", package.replace("-", "_")));

        // Generate message definitions
        for msg in messages {
            proto.push_str(&self.generate_proto_message(msg)?);
            proto.push('\n');
        }

        Ok(proto)
    }

    /// Generate a single protobuf message definition from a parsed ROS message
    fn generate_proto_message(&self, msg: &roslibrust_codegen::MessageFile) -> Result<String> {
        let mut proto = String::new();

        proto.push_str(&format!("message {} {{\n", msg.parsed.name));

        // Generate fields
        for (index, field) in msg.parsed.fields.iter().enumerate() {
            let field_number = index + 1;
            let proto_type = self.ros_field_to_proto_type(field)?;
            let field_name = &field.field_name;

            proto.push_str(&format!(
                "  {} {} = {};\n",
                proto_type, field_name, field_number
            ));
        }

        proto.push_str("}\n");

        Ok(proto)
    }

    /// Convert ROS field type to protobuf type
    fn ros_field_to_proto_type(&self, field: &roslibrust_codegen::FieldInfo) -> Result<String> {
        let base_type = &field.field_type.field_type;
        let is_array = !matches!(
            field.field_type.array_info,
            roslibrust_codegen::ArrayType::NotArray
        );

        // Map ROS primitive types to protobuf types
        let proto_type = match base_type.as_str() {
            "bool" => "bool",
            "byte" | "uint8" | "char" => "uint32", // Proto3 has no uint8
            "int8" | "int16" => "int32",           // Proto3 has no int8/int16
            "uint16" => "uint32",
            "int32" => "int32",
            "uint32" => "uint32",
            "int64" => "int64",
            "uint64" => "uint64",
            "float32" => "float",
            "float64" => "double",
            "string" => "string",
            // Complex types (other messages)
            _ => {
                // If it has a package name, it's a message type
                if field.field_type.package_name.is_some()
                    || field.field_type.source_package != field.field_type.field_type
                {
                    // Use the message type name directly
                    // Proto will resolve it within the same package
                    base_type.as_str()
                } else {
                    // Unknown type - use bytes as fallback
                    "bytes"
                }
            }
        };

        // Handle arrays
        if is_array {
            Ok(format!("repeated {}", proto_type))
        } else {
            Ok(proto_type.to_string())
        }
    }

    /// Generate Rust code from .proto files using prost_build
    pub fn generate_rust_from_proto(
        &self,
        proto_files: &[PathBuf],
        output_file: &Path,
    ) -> Result<()> {
        if proto_files.is_empty() {
            return Ok(());
        }

        // Configure prost_build
        let mut config = prost_build::Config::new();

        // Generate code to a specific output directory
        let temp_dir = self.proto_dir.join("prost_output");
        fs::create_dir_all(&temp_dir)?;
        config.out_dir(&temp_dir);

        // Compile proto files
        let proto_paths: Vec<&Path> = proto_files.iter().map(|p| p.as_path()).collect();
        config
            .compile_protos(&proto_paths, &[&self.proto_dir])
            .context("Failed to compile proto files with prost_build")?;

        // Read generated files and combine them
        let mut combined_output = String::new();

        // Add necessary imports
        combined_output.push_str("// Auto-generated protobuf message types\n");
        combined_output.push_str("// DO NOT EDIT\n\n");
        combined_output.push_str("use prost::Message as ProstMessage;\n\n");

        // Read all generated .rs files
        for entry in fs::read_dir(&temp_dir)? {
            let entry = entry?;
            let path = entry.path();
            if path.extension().and_then(|s| s.to_str()) == Some("rs") {
                let content = fs::read_to_string(&path)?;
                combined_output.push_str(&content);
                combined_output.push('\n');
            }
        }

        // Write combined output
        fs::write(output_file, combined_output)
            .with_context(|| format!("Failed to write combined output: {:?}", output_file))?;

        // Clean up temp directory
        fs::remove_dir_all(&temp_dir).ok();

        Ok(())
    }

    /// Generate MessageTypeInfo implementations for protobuf types
    pub fn generate_type_info_impls(
        &self,
        messages: &[roslibrust_codegen::MessageFile],
    ) -> Result<String> {
        let mut impls = String::new();

        impls.push_str("// MessageTypeInfo implementations for protobuf types\n\n");

        for msg in messages {
            let package = &msg.parsed.package;
            let msg_name = &msg.parsed.name;

            // Rust type name for the protobuf struct
            let proto_type = format!("{}::{}", package.replace("-", "_"), msg_name);

            // ROS2 type name
            let ros2_type_name = format!("{}::msg::dds_::{}_", package, msg_name);

            // Get hash
            let hash = msg.ros2_hash.to_hash_string();

            impls.push_str(&format!(
                r#"impl ::ros_z::MessageTypeInfo for {proto_type} {{
    fn type_name() -> &'static str {{
        "{ros2_type_name}"
    }}

    fn type_hash() -> ::ros_z::entity::TypeHash {{
        ::ros_z::entity::TypeHash::from_rihs_string("{hash}")
            .expect("Invalid RIHS hash string")
    }}
}}

impl ::ros_z::WithTypeInfo for {proto_type} {{}}

impl ::ros_z::msg::ZMessage for {proto_type} {{
    type Serdes = ::ros_z::msg::ProtobufSerdes<{proto_type}>;
}}

"#
            ));
        }

        Ok(impls)
    }
}
