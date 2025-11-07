use anyhow::{Context, Result};
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

use crate::type_info_generator::TypeInfoGenerator;

/// Generate ROS messages using roslibrust with proper MessageTypeInfo implementations
pub fn generate_ros_messages(
    package_paths: Vec<&Path>,
    output_dir: &Path,
    generate_type_info: bool,
) -> Result<()> {
    // Convert paths once
    let package_paths_vec: Vec<PathBuf> = package_paths.into_iter().map(|p| p.to_path_buf()).collect();

    // Use roslibrust_codegen to generate message code
    // This returns a TokenStream and list of files
    let (token_stream, _files) = roslibrust_codegen::find_and_generate_ros_messages_without_ros_package_path(
        package_paths_vec.clone(),
    )?;

    // Convert TokenStream to string
    let mut generated_code = token_stream.to_string();

    // If requested, add MessageTypeInfo trait implementations with real ROS2 hashes
    if generate_type_info {
        // Parse all messages to get their metadata (actions are also returned but we ignore for now)
        let (messages, services, _actions) = roslibrust_codegen::find_and_parse_ros_messages(&package_paths_vec)?;

        // Resolve dependencies to calculate hashes
        let (resolved_msgs, _resolved_srvs) = roslibrust_codegen::resolve_dependency_graph(messages, services)?;

        // Build a map for hash lookups
        let msg_map: BTreeMap<String, &roslibrust_codegen::MessageFile> = resolved_msgs
            .iter()
            .map(|msg| (msg.parsed.get_full_name(), msg))
            .collect();

        // Generate MessageTypeInfo implementations
        let type_info_impls = generate_message_type_info_impls(&resolved_msgs, &msg_map)?;

        generated_code.push_str("\n\n");
        generated_code.push_str("// MessageTypeInfo trait implementations for ros-z integration\n");
        generated_code.push_str(&type_info_impls);
    }

    // Write to output file
    let output_file = output_dir.join("generated.rs");
    fs::write(&output_file, generated_code)
        .with_context(|| format!("Failed to write generated code to {:?}", output_file))?;

    println!("cargo:rerun-if-changed=build.rs");

    Ok(())
}

/// Generate MessageTypeInfo trait implementations for all messages
fn generate_message_type_info_impls(
    messages: &[roslibrust_codegen::MessageFile],
    _msg_map: &BTreeMap<String, &roslibrust_codegen::MessageFile>,
) -> Result<String> {
    let mut impls = String::new();

    for msg in messages {
        let package_name = &msg.parsed.package;
        let msg_name = &msg.parsed.name;

        // Generate the Rust type name (e.g., geometry_msgs::Vector3 - roslibrust doesn't use ::msg)
        let rust_type_name = format!("{}::{}", package_name, msg_name);

        // Generate the ROS2 type name (e.g., "geometry_msgs::msg::dds_::Vector3_")
        let ros_type_name = format!("{}::msg::dds_::{}_", package_name, msg_name);

        // Get the RIHS01 hash from roslibrust's calculation
        let type_hash = msg.ros2_hash.to_hash_string();

        // Generate the MessageTypeInfo implementation
        let type_info_impl = TypeInfoGenerator::generate_type_info_impl(
            &rust_type_name,
            &ros_type_name,
            &type_hash,
        );

        impls.push_str(&type_info_impl);
    }

    Ok(impls)
}
