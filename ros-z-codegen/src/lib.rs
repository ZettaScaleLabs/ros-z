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
        let mut resolver = resolver::Resolver::new(self.config.is_humble);
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

        for package_name in all_package_names {
            let package_ident = quote::format_ident!("{}", &package_name);

            // Generate message implementations
            let message_impls: Vec<_> = packages
                .get(&package_name)
                .map(|msgs| {
                    msgs.iter()
                        .map(|msg| generator::rust::generate_message_impl(msg))
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
