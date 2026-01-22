// Standalone codegen modules (Phase 1+)
pub mod discovery;
pub mod generator;
pub mod hashing;
pub mod parser;
pub mod resolver;
pub mod types;

// Legacy adapters (will be migrated to use ResolvedMessage)
#[cfg(feature = "protobuf")]
pub mod protobuf_adapter;

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
// Re-exports for backward compatibility
pub use types::{ResolvedMessage, ResolvedService};

/// Configuration for the message generator
pub struct GeneratorConfig {
    /// Generate CDR-compatible serde types
    pub generate_cdr: bool,

    /// Generate protobuf definitions
    pub generate_protobuf: bool,

    /// Generate MessageTypeInfo trait impls
    pub generate_type_info: bool,

    /// Humble compatibility mode (no ServiceEventInfo, placeholder type hashes)
    pub is_humble: bool,

    pub output_dir: PathBuf,

    /// External crate path for standard message types (e.g., "ros_z_msgs").
    /// When set, references to packages NOT in the local package set will use
    /// fully qualified paths: `::{external_crate}::ros::{package}::{Type}`
    pub external_crate: Option<String>,

    /// Set of local package names (used with external_crate to determine
    /// which types need external references)
    pub local_packages: std::collections::HashSet<String>,
}

/// Message generator that orchestrates parsing, resolution, and code generation
pub struct MessageGenerator {
    config: GeneratorConfig,
}

impl MessageGenerator {
    pub fn new(config: GeneratorConfig) -> Self {
        Self { config }
    }

    /// Primary generation method - uses pure Rust codegen pipeline
    pub fn generate_from_msg_files(&self, packages: &[&Path]) -> Result<()> {
        // Discover and parse all messages, services, and actions
        let (messages, services, actions) = discovery::discover_all(packages)
            .context("Failed to discover messages, services, and actions")?;

        // Filter out problematic messages
        let messages = Self::filter_messages(messages);
        let services = Self::filter_services(services);

        println!(
            "cargo:info=Discovered {} messages, {} services, and {} actions",
            messages.len(),
            services.len(),
            actions.len()
        );

        // Resolve dependencies and calculate type hashes
        // If external_crate is set, determine which packages are external (not local)
        let external_packages = if self.config.external_crate.is_some() {
            // Standard ROS2 packages that are provided by ros_z_msgs
            let standard_packages: std::collections::HashSet<String> = [
                "builtin_interfaces",
                "std_msgs",
                "geometry_msgs",
                "sensor_msgs",
                "nav_msgs",
                "action_msgs",
                "unique_identifier_msgs",
                "service_msgs",
                "example_interfaces",
                "action_tutorials_interfaces",
                "test_msgs",
            ]
            .iter()
            .map(|s| s.to_string())
            .collect();

            // External packages = standard packages - local packages
            standard_packages
                .difference(&self.config.local_packages)
                .cloned()
                .collect()
        } else {
            std::collections::HashSet::new()
        };

        let mut resolver =
            resolver::Resolver::with_external_packages(self.config.is_humble, external_packages);
        let resolved_messages = resolver
            .resolve_messages(messages)
            .context("Failed to resolve message dependencies")?;
        let resolved_services = resolver
            .resolve_services(services)
            .context("Failed to resolve service dependencies")?;
        let resolved_actions = resolver
            .resolve_actions(actions)
            .context("Failed to resolve action dependencies")?;

        println!(
            "cargo:info=Resolved {} messages, {} services, and {} actions",
            resolved_messages.len(),
            resolved_services.len(),
            resolved_actions.len()
        );

        // Generate CDR-compatible types (using pure Rust codegen with ZBuf support)
        if self.config.generate_cdr {
            self.generate_cdr_types(&resolved_messages, &resolved_services, &resolved_actions)?;
        }

        // Generate protobuf types
        #[cfg(feature = "protobuf")]
        if self.config.generate_protobuf {
            self.generate_protobuf_types(&resolved_messages)?;
        }

        Ok(())
    }

    /// Filter out problematic messages (actionlib, wstring, etc.)
    fn filter_messages(
        messages: Vec<crate::types::ParsedMessage>,
    ) -> Vec<crate::types::ParsedMessage> {
        let filtered: Vec<_> = messages
            .into_iter()
            .filter(|msg| {
                let full_name = format!("{}/{}", msg.package, msg.name);

                // Filter out old ROS 1 actionlib_msgs (deprecated)
                // Note: ROS 2 action messages (Goal/Result/Feedback) are now generated from .action files
                if full_name.starts_with("actionlib_msgs/") {
                    println!(
                        "cargo:info=Filtered deprecated actionlib_msgs: {}",
                        full_name
                    );
                    return false;
                }

                // Filter out redundant service Request/Response message files
                // ROS 2 Humble ships with *_Request.msg and *_Response.msg that duplicate
                // the messages auto-generated from .srv files
                if msg.name.ends_with("_Request") || msg.name.ends_with("_Response") {
                    println!("cargo:info=Filtered service msg file: {}", full_name);
                    return false;
                }

                // Also filter any message file in the "srv" directory (sometimes ROS puts srv msgs there)
                if msg.path.to_string_lossy().contains("/srv/") {
                    println!("cargo:info=Filtered srv directory message: {}", full_name);
                    return false;
                }

                // Filter out messages with wstring fields
                let has_wstring = msg
                    .fields
                    .iter()
                    .any(|field| field.field_type.base_type.contains("wstring"));

                if has_wstring {
                    println!(
                        "cargo:warning=Skipping message {} due to wstring field (unsupported)",
                        full_name
                    );
                    return false;
                }

                true
            })
            .collect();

        println!(
            "cargo:info=After filtering: {} messages remain",
            filtered.len()
        );
        filtered
    }

    /// Filter out problematic services
    fn filter_services(
        services: Vec<crate::types::ParsedService>,
    ) -> Vec<crate::types::ParsedService> {
        services
            .into_iter()
            .filter(|srv| {
                let full_name = format!("{}/{}", srv.package, srv.name);
                !full_name.starts_with("actionlib_msgs/")
            })
            .collect()
    }

    /// Generate CDR-compatible Rust types with ZBuf support
    fn generate_cdr_types(
        &self,
        messages: &[ResolvedMessage],
        services: &[ResolvedService],
        actions: &[crate::types::ResolvedAction],
    ) -> Result<()> {
        use std::collections::BTreeMap;

        use quote::quote;

        // Group messages, services, and actions by package
        let mut packages: BTreeMap<String, Vec<&ResolvedMessage>> = BTreeMap::new();
        let mut package_services: BTreeMap<String, Vec<&ResolvedService>> = BTreeMap::new();
        let mut package_actions: BTreeMap<String, Vec<&crate::types::ResolvedAction>> =
            BTreeMap::new();

        for msg in messages {
            packages
                .entry(msg.parsed.package.clone())
                .or_default()
                .push(msg);
        }

        // Add service request/response messages and track services
        for srv in services {
            packages
                .entry(srv.parsed.package.clone())
                .or_default()
                .push(&srv.request);
            packages
                .entry(srv.parsed.package.clone())
                .or_default()
                .push(&srv.response);

            package_services
                .entry(srv.parsed.package.clone())
                .or_default()
                .push(srv);
        }

        // Add action goal/result/feedback messages and track actions
        for action in actions {
            packages
                .entry(action.parsed.package.clone())
                .or_default()
                .push(&action.goal);
            if let Some(ref result) = action.result {
                packages
                    .entry(action.parsed.package.clone())
                    .or_default()
                    .push(result);
            }
            if let Some(ref feedback) = action.feedback {
                packages
                    .entry(action.parsed.package.clone())
                    .or_default()
                    .push(feedback);
            }

            package_actions
                .entry(action.parsed.package.clone())
                .or_default()
                .push(action);
        }

        // Generate code for each package
        let mut all_tokens = proc_macro2::TokenStream::new();

        // Collect all package names
        let mut all_package_names = std::collections::BTreeSet::new();
        all_package_names.extend(packages.keys().cloned());
        all_package_names.extend(package_services.keys().cloned());
        all_package_names.extend(package_actions.keys().cloned());

        // Create generation context for external type references
        let gen_ctx = generator::rust::GenerationContext::new(
            self.config.external_crate.clone(),
            self.config.local_packages.clone(),
        );

        for package_name in all_package_names {
            let package_ident = quote::format_ident!("{}", &package_name);

            // Generate message implementations
            let message_impls: Vec<_> = packages
                .get(&package_name)
                .map(|msgs| {
                    msgs.iter()
                        .map(|msg| {
                            generator::rust::generate_message_impl_with_context(msg, &gen_ctx)
                        })
                        .collect::<Result<Vec<_>>>()
                })
                .transpose()
                .context("Failed to generate message implementations")?
                .unwrap_or_default();

            // Generate service implementations
            let service_impls: Vec<_> = package_services
                .get(&package_name)
                .map(|srvs| {
                    srvs.iter()
                        .map(|srv| generator::rust::generate_service_impl(srv))
                        .collect::<Result<Vec<_>>>()
                })
                .transpose()
                .context("Failed to generate service implementations")?
                .unwrap_or_default();

            // Generate action implementations
            let action_impls: Vec<_> = package_actions
                .get(&package_name)
                .map(|acts| {
                    acts.iter()
                        .map(|action| generator::rust::generate_action_impl(action))
                        .collect::<Result<Vec<_>>>()
                })
                .transpose()
                .context("Failed to generate action implementations")?
                .unwrap_or_default();

            // Create submodules for services and actions
            let service_module = if !service_impls.is_empty() {
                quote! {
                    pub mod srv {
                        #(#service_impls)*
                    }
                }
            } else {
                quote! {}
            };

            let action_module = if !action_impls.is_empty() {
                quote! {
                    pub mod action {
                        #(#action_impls)*
                    }
                }
            } else {
                quote! {}
            };

            let pkg_tokens = quote! {
                pub mod #package_ident {
                    #(#message_impls)*
                    #service_module
                    #action_module
                }
            };

            all_tokens.extend(pkg_tokens);
        }

        // Wrap in ros module for namespacing
        let wrapped_tokens = quote! {
            #[allow(clippy::approx_constant)]
            pub mod ros {
                #all_tokens
            }
        };

        // Format and write
        let syntax_tree: syn::File =
            syn::parse2(wrapped_tokens).context("Failed to parse generated code")?;
        let formatted_code = prettyplease::unparse(&syntax_tree);

        let output_file = self.config.output_dir.join("generated.rs");
        std::fs::write(&output_file, formatted_code)
            .with_context(|| format!("Failed to write generated code to {:?}", output_file))?;

        println!(
            "cargo:info=Generated {} CDR types with ZBuf support",
            messages.len() + services.len() + actions.len()
        );

        Ok(())
    }

    /// Generate protobuf types
    #[cfg(feature = "protobuf")]
    fn generate_protobuf_types(&self, messages: &[ResolvedMessage]) -> Result<()> {
        use crate::protobuf_adapter::ProtobufMessageGenerator;

        let proto_dir = self.config.output_dir.join("proto");
        let generator = ProtobufMessageGenerator::new(&proto_dir);

        // Generate .proto files
        let proto_files = generator.generate_proto_files(messages)?;
        println!("cargo:info=Generated {} .proto files", proto_files.len());

        // Generate Rust code from .proto files
        let proto_output = self.config.output_dir.join("generated_proto.rs");
        generator.generate_rust_from_proto(&proto_files, &proto_output)?;

        // Generate MessageTypeInfo implementations
        let type_info_impls = generator.generate_type_info_impls(messages)?;
        let type_info_output = self.config.output_dir.join("protobuf_type_info.rs");
        std::fs::write(&type_info_output, type_info_impls)?;

        println!("cargo:info=Protobuf generation complete");
        Ok(())
    }
}

/// Discover user message packages from the ROS_Z_MSG_PATH environment variable.
///
/// The environment variable should contain a colon-separated list of paths,
/// where each path is a ROS2 package directory containing msg/, srv/, or action/ subdirs.
///
/// # Example
/// ```bash
/// export ROS_Z_MSG_PATH="/path/to/my_msgs:/path/to/other_msgs"
/// ```
pub fn discover_user_packages() -> Result<Vec<PathBuf>> {
    let msg_path =
        std::env::var("ROS_Z_MSG_PATH").context("ROS_Z_MSG_PATH environment variable not set")?;

    let mut packages = Vec::new();

    for path_str in msg_path.split(':') {
        let path = PathBuf::from(path_str.trim());
        if path_str.trim().is_empty() {
            continue;
        }

        if !path.exists() {
            println!(
                "cargo:warning=ROS_Z_MSG_PATH entry does not exist: {:?}",
                path
            );
            continue;
        }

        // Check if this path has msg/, srv/, or action/ subdirectories
        let has_messages =
            path.join("msg").exists() || path.join("srv").exists() || path.join("action").exists();

        if has_messages {
            println!("cargo:info=Found user package at: {:?}", path);
            packages.push(path);
        } else {
            println!(
                "cargo:warning=Path {:?} has no msg/, srv/, or action/ directory",
                path
            );
        }
    }

    if packages.is_empty() {
        anyhow::bail!("No valid message packages found in ROS_Z_MSG_PATH");
    }

    Ok(packages)
}

/// High-level API for user crates to generate messages from ROS_Z_MSG_PATH.
///
/// This function:
/// 1. Discovers packages from ROS_Z_MSG_PATH environment variable
/// 2. Generates Rust code with external references to ros_z_msgs for standard types
///
/// # Arguments
/// * `output_dir` - Directory where generated.rs will be written
/// * `is_humble` - Set to true for ROS2 Humble compatibility mode
///
/// # Example
/// ```rust,ignore
/// // In build.rs
/// fn main() -> anyhow::Result<()> {
///     let out_dir = std::env::var("OUT_DIR")?;
///     ros_z_codegen::generate_user_messages(&out_dir.into(), false)?;
///     println!("cargo:rerun-if-env-changed=ROS_Z_MSG_PATH");
///     Ok(())
/// }
/// ```
pub fn generate_user_messages(output_dir: &Path, is_humble: bool) -> Result<()> {
    let packages = discover_user_packages()?;

    // Collect local package names
    let local_packages: std::collections::HashSet<String> = packages
        .iter()
        .filter_map(|p| discovery::discover_package_name(p).ok())
        .collect();

    println!(
        "cargo:info=Generating user messages for packages: {:?}",
        local_packages
    );

    let config = GeneratorConfig {
        generate_cdr: true,
        generate_protobuf: false,
        generate_type_info: true,
        is_humble,
        output_dir: output_dir.to_path_buf(),
        external_crate: Some("ros_z_msgs".to_string()),
        local_packages,
    };

    let generator = MessageGenerator::new(config);
    let package_refs: Vec<&Path> = packages.iter().map(|p| p.as_path()).collect();
    generator.generate_from_msg_files(&package_refs)
}

#[cfg(test)]
mod tests {
    use std::fs;

    use serial_test::serial;

    use super::*;

    // Helper to safely set/remove env vars in Rust 2024
    // SAFETY: Tests using these are marked #[serial] to prevent data races
    fn set_env(key: &str, value: &str) {
        unsafe { std::env::set_var(key, value) };
    }

    fn remove_env(key: &str) {
        unsafe { std::env::remove_var(key) };
    }

    /// Test that user-defined messages with external dependencies generate correct code
    #[test]
    #[serial]
    fn test_generate_user_messages_with_external_deps() {
        // Create a temp directory structure
        let temp_dir = tempfile::tempdir().unwrap();
        let pkg_dir = temp_dir.path().join("my_test_msgs");
        let msg_dir = pkg_dir.join("msg");
        fs::create_dir_all(&msg_dir).unwrap();

        // Create a message that references external types (geometry_msgs/Point)
        let msg_content = r#"
string robot_id
geometry_msgs/Point position
bool is_active
"#;
        fs::write(msg_dir.join("TestStatus.msg"), msg_content).unwrap();

        // Create output directory
        let out_dir = temp_dir.path().join("out");
        fs::create_dir_all(&out_dir).unwrap();

        // Set the environment variable
        set_env("ROS_Z_MSG_PATH", pkg_dir.to_str().unwrap());

        // Generate messages
        let result = generate_user_messages(&out_dir, false);
        assert!(
            result.is_ok(),
            "generate_user_messages failed: {:?}",
            result
        );

        // Read the generated file
        let generated_path = out_dir.join("generated.rs");
        assert!(generated_path.exists(), "generated.rs was not created");

        let generated_code = fs::read_to_string(&generated_path).unwrap();

        // Verify external type reference uses fully qualified path
        assert!(
            generated_code.contains("::ros_z_msgs::ros::geometry_msgs::Point"),
            "Generated code should use fully qualified path for external types.\nGenerated:\n{}",
            generated_code
        );

        // Verify struct was generated
        assert!(
            generated_code.contains("pub struct TestStatus"),
            "Generated code should contain TestStatus struct.\nGenerated:\n{}",
            generated_code
        );

        // Verify it's in the correct module
        assert!(
            generated_code.contains("pub mod my_test_msgs"),
            "Generated code should have my_test_msgs module.\nGenerated:\n{}",
            generated_code
        );

        // Clean up env var
        remove_env("ROS_Z_MSG_PATH");
    }

    /// Test that services with external dependencies generate correct code
    #[test]
    #[serial]
    fn test_generate_user_services_with_external_deps() {
        let temp_dir = tempfile::tempdir().unwrap();
        let pkg_dir = temp_dir.path().join("my_test_srvs");
        let srv_dir = pkg_dir.join("srv");
        fs::create_dir_all(&srv_dir).unwrap();

        // Create a service that references external types
        let srv_content = r#"
geometry_msgs/Point target
float64 speed
---
bool success
"#;
        fs::write(srv_dir.join("MoveTo.srv"), srv_content).unwrap();

        let out_dir = temp_dir.path().join("out");
        fs::create_dir_all(&out_dir).unwrap();

        set_env("ROS_Z_MSG_PATH", pkg_dir.to_str().unwrap());

        let result = generate_user_messages(&out_dir, false);
        assert!(
            result.is_ok(),
            "generate_user_messages failed: {:?}",
            result
        );

        let generated_code = fs::read_to_string(out_dir.join("generated.rs")).unwrap();

        // Verify service request has external type reference
        assert!(
            generated_code.contains("pub struct MoveToRequest"),
            "Generated code should contain MoveToRequest struct"
        );
        assert!(
            generated_code.contains("::ros_z_msgs::ros::geometry_msgs::Point"),
            "Service request should use fully qualified path for external types"
        );

        // Verify service module
        assert!(
            generated_code.contains("pub mod srv"),
            "Generated code should have srv submodule"
        );

        remove_env("ROS_Z_MSG_PATH");
    }

    /// Test discover_user_packages with missing env var
    #[test]
    #[serial]
    fn test_discover_user_packages_missing_env() {
        remove_env("ROS_Z_MSG_PATH");
        let result = discover_user_packages();
        assert!(result.is_err());
        assert!(
            result
                .unwrap_err()
                .to_string()
                .contains("ROS_Z_MSG_PATH environment variable not set")
        );
    }

    /// Test discover_user_packages with invalid path
    #[test]
    #[serial]
    fn test_discover_user_packages_no_valid_packages() {
        let temp_dir = tempfile::tempdir().unwrap();
        // Create a directory without msg/srv/action subdirs
        let empty_pkg = temp_dir.path().join("empty_pkg");
        fs::create_dir_all(&empty_pkg).unwrap();

        set_env("ROS_Z_MSG_PATH", empty_pkg.to_str().unwrap());

        let result = discover_user_packages();
        assert!(result.is_err());
        assert!(
            result
                .unwrap_err()
                .to_string()
                .contains("No valid message packages found")
        );

        remove_env("ROS_Z_MSG_PATH");
    }
}
