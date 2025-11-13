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

        // Collect all dependencies
        let mut dependencies = std::collections::BTreeSet::new();
        for msg in messages {
            self.collect_dependencies(msg, &mut dependencies);
        }

        // Add imports for dependencies (excluding self)
        for dep in &dependencies {
            if dep != package {
                proto.push_str(&format!("import \"{}.proto\";\n", dep.replace("-", "_")));
            }
        }
        if !dependencies.is_empty() {
            proto.push('\n');
        }

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

    /// Collect all package dependencies for a message
    fn collect_dependencies(
        &self,
        msg: &roslibrust_codegen::MessageFile,
        dependencies: &mut std::collections::BTreeSet<String>,
    ) {
        for field in &msg.parsed.fields {
            if let Some(ref package_name) = field.field_type.package_name {
                if package_name != &msg.parsed.package {
                    dependencies.insert(package_name.clone());
                }
            } else if field.field_type.source_package != msg.parsed.package {
                dependencies.insert(field.field_type.source_package.clone());
            }
        }
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
            "bool" => "bool".to_string(),
            "byte" | "uint8" | "char" => "uint32".to_string(), // Proto3 has no uint8
            "int8" | "int16" => "int32".to_string(),           // Proto3 has no int8/int16
            "uint16" => "uint32".to_string(),
            "int32" => "int32".to_string(),
            "uint32" => "uint32".to_string(),
            "int64" => "int64".to_string(),
            "uint64" => "uint64".to_string(),
            "float32" => "float".to_string(),
            "float64" => "double".to_string(),
            "string" => "string".to_string(),
            // Complex types (other messages)
            _ => {
                // If it has a package name, it's a message type from another package
                if let Some(ref package_name) = field.field_type.package_name {
                    format!("{}.{}", package_name.replace("-", "_"), base_type)
                } else if field.field_type.source_package != field.field_type.field_type {
                    format!(
                        "{}.{}",
                        field.field_type.source_package.replace("-", "_"),
                        base_type
                    )
                } else {
                    // Same package message type
                    base_type.clone()
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

        // Read generated files and combine them
        let mut combined_output = String::new();

        // Add necessary imports
        combined_output.push_str("// Auto-generated protobuf message types\n");
        combined_output.push_str("// DO NOT EDIT\n\n");
        combined_output.push_str("#[allow(unused_imports)]\n");
        combined_output.push_str("use prost::Message as ProstMessage;\n");
        combined_output.push_str("#[allow(unused_imports)]\n");
        combined_output.push_str("use ros_z::MessageTypeInfo;\n");
        combined_output.push_str("#[allow(unused_imports)]\n");
        combined_output.push_str("use ros_z::ros_msg::WithTypeInfo;\n");
        combined_output.push_str("#[allow(unused_imports)]\n");
        combined_output.push_str("use ros_z::msg::ZMessage;\n");
        combined_output.push_str("#[allow(unused_imports)]\n");
        combined_output.push_str("use ros_z::msg::ProtobufSerdes;\n\n");

        // Compile all proto files at once to avoid duplicates
        let temp_dir = self.proto_dir.join("prost_output");
        fs::create_dir_all(&temp_dir)?;

        let mut config = prost_build::Config::new();
        config.out_dir(&temp_dir);

        // Compile all proto files together
        config
            .compile_protos(proto_files, &[&self.proto_dir])
            .context("Failed to compile proto files with prost_build")?;

        // Read all generated files and organize them by package
        let mut package_modules = std::collections::BTreeMap::new();

        for entry in fs::read_dir(&temp_dir)? {
            let entry = entry?;
            let path = entry.path();
            if path.extension().and_then(|s| s.to_str()) == Some("rs") {
                let content = fs::read_to_string(&path)?;

                // The file name typically corresponds to the package name
                let file_stem = path
                    .file_stem()
                    .and_then(|s| s.to_str())
                    .unwrap_or("unknown")
                    .to_string(); // Convert to owned String

                // Map common file patterns to package names
                let package_name = match file_stem.as_str() {
                    "builtin_interfaces" => "builtin_interfaces",
                    "example_interfaces" => "example_interfaces",
                    "geometry_msgs" => "geometry_msgs",
                    "nav_msgs" => "nav_msgs",
                    "sensor_msgs" => "sensor_msgs",
                    "service_msgs" => "service_msgs",
                    "std_msgs" => "std_msgs",
                    _ => file_stem.as_str(),
                };

                package_modules.insert(package_name.to_string(), content);
            }
        }

        // Write modules in a consistent order
        for (package_name, content) in package_modules {
            combined_output.push_str(&format!("pub mod {} {{\n", package_name.replace("-", "_")));
            combined_output.push_str(&content);
            combined_output.push_str("}\n\n");
        }

        // Write combined output
        fs::write(output_file, combined_output)
            .with_context(|| format!("Failed to write combined output: {:?}", output_file))?;

        // Note: Temp directory left for debugging
        // if temp_dir.exists() {
        //     fs::remove_dir_all(&temp_dir).ok();
        // }

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

            // Convert ROS message name to prost naming convention
            // Prost converts camelCase to snake_case differently for acronyms
            let proto_struct_name = self.convert_to_prost_naming(msg_name);

            // Rust type name for the protobuf struct
            let proto_type = format!("{}::{}", package.replace("-", "_"), proto_struct_name);

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

    /// Convert ROS message name to prost naming convention
    fn convert_to_prost_naming(&self, name: &str) -> String {
        // Handle specific known cases where prost naming differs
        match name {
            "MultiDOFJointState" => "MultiDofJointState".to_string(),
            "ColorRGBA" => "ColorRgba".to_string(),
            // Add more mappings as needed
            _ => name.to_string(),
        }
    }
}
