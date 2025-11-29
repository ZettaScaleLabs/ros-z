use anyhow::{Context, Result};
use proc_macro2::TokenStream;
use quote::quote;
use roslibrust_codegen::{MessageFile, ServiceFile};
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

/// Configuration for code generation
pub struct CodegenConfig {
    /// Generate trait implementations for ros-z
    pub generate_type_info: bool,
    /// Format the generated code with proper indentation and newlines
    pub format_code: bool,
    /// Generate DEFINITION field in RosMessageType trait (default: false)
    pub generate_definition: bool,
}

impl Default for CodegenConfig {
    fn default() -> Self {
        Self {
            generate_type_info: true,
            format_code: true,
            generate_definition: false,
        }
    }
}

/// Recursively deduplicate struct definitions by name, keeping the first occurrence
fn dedup_structs_recursive(items: &mut Vec<syn::Item>) {
    let mut struct_map: std::collections::BTreeMap<String, syn::ItemStruct> = std::collections::BTreeMap::new();
    let mut new_items = vec![];

    for item in items.drain(..) {
        match item {
            syn::Item::Struct(struct_item) => {
                let name = struct_item.ident.to_string();
                if !struct_map.contains_key(&name) {
                    struct_map.insert(name.clone(), struct_item.clone());
                    new_items.push(syn::Item::Struct(struct_item));
                }
            }
            syn::Item::Mod(mut mod_item) => {
                if let Some((_, ref mut content)) = mod_item.content {
                    dedup_structs_recursive(content);
                }
                new_items.push(syn::Item::Mod(mod_item));
            }
            other => new_items.push(other),
        }
    }

    *items = new_items;
}

/// Generate ROS messages using roslibrust with ros-z trait implementations
pub fn generate_ros_messages(
    package_paths: Vec<&Path>,
    output_dir: &Path,
    generate_type_info: bool,
) -> Result<()> {
    let config = CodegenConfig {
        generate_type_info,
        format_code: true,
        generate_definition: false,
    };
    generate_ros_messages_with_config(package_paths, output_dir, config)
}

/// Generate ROS messages with custom configuration
pub fn generate_ros_messages_with_config(
    package_paths: Vec<&Path>,
    output_dir: &Path,
    config: CodegenConfig,
) -> Result<()> {
    // Convert paths once
    let package_paths_vec: Vec<PathBuf> = package_paths.into_iter().map(|p| p.to_path_buf()).collect();

    // Parse all messages to get their metadata
    let (messages, services, _actions) = roslibrust_codegen::find_and_parse_ros_messages(&package_paths_vec)?;

    // Filter out deprecated actionlib messages and unsupported wstring messages
    // actionlib_msgs was deprecated in ROS 2 and causes dependency resolution issues
    // wstring is poorly supported in Rust and causes codegen failures
    let messages = messages
        .into_iter()
        .filter(|msg| {
            let full_name = msg.get_full_name();

            // Filter out actionlib_msgs package
            if full_name.starts_with("actionlib_msgs/") {
                return false;
            }

            // Filter out old-style ROS1 Action messages (deprecated in ROS2)
            // These have suffixes like Action, ActionGoal, ActionResult, ActionFeedback
            // and depend on actionlib_msgs which causes resolution failures
            // Note: ROS2 uses the new action system with .action files, not these msg files
            let is_old_action_msg = full_name.ends_with("Action")
                || full_name.ends_with("ActionGoal")
                || full_name.ends_with("ActionResult")
                || full_name.ends_with("ActionFeedback");

            if is_old_action_msg {
                return false;
            }

            // Filter out messages with wstring fields
            let has_wstring = msg.fields.iter().any(|field| {
                field.field_type.to_string().contains("wstring")
            });

            if has_wstring {
                println!("cargo:warning=Skipping message {} due to wstring field (unsupported)", full_name);
                return false;
            }

            true
        })
        .collect();

    let services = services
        .into_iter()
        .filter(|srv| {
            let full_name = format!("{}/{}", srv.package, srv.name);
            !full_name.starts_with("actionlib_msgs/")
        })
        .collect();

    // Resolve dependencies to calculate hashes
    let (mut resolved_msgs, resolved_srvs) = roslibrust_codegen::resolve_dependency_graph(messages, services)?;

    // Filter out duplicate messages by full name to avoid conflicts
    let mut seen = std::collections::HashSet::new();
    resolved_msgs.retain(|msg| seen.insert(msg.parsed.get_full_name()));

    // Create roslibrust codegen options
    let roslibrust_options = roslibrust_codegen::CodegenOptions {
        generate_definition: config.generate_definition,
        roslibrust_serde: false, // Use standard Rust serde instead of roslibrust's re-exports
    };

    // Generate message code with options
    let mut token_stream = roslibrust_codegen::generate_rust_ros_message_definitions(
        resolved_msgs.clone(),
        resolved_srvs.clone(),
        &roslibrust_options,
    )?;

    // If requested, add trait implementations with real ROS2 hashes
    if config.generate_type_info {
        // Build a map for hash lookups
        let msg_map: BTreeMap<String, &MessageFile> = resolved_msgs
            .iter()
            .map(|msg| (msg.parsed.get_full_name(), msg))
            .collect();

        // Generate trait implementations for messages as TokenStreams
        let type_info_tokens = generate_message_type_info_tokens(&resolved_msgs, &msg_map)?;
        token_stream.extend(type_info_tokens);

        // Generate trait implementations for services
        if !resolved_srvs.is_empty() {
            let service_info_tokens = generate_service_type_info_tokens(&resolved_srvs)?;
            token_stream.extend(service_info_tokens);
        }
    }

    // Deduplicate duplicate struct definitions (e.g., Arrays, BasicTypes, Empty)
    // that may be generated multiple times when multiple packages are enabled
    let mut syntax_tree = syn::parse2::<syn::File>(token_stream)
        .context("Failed to parse generated token stream for deduplication")?;

    dedup_structs_recursive(&mut syntax_tree.items);

    token_stream = quote::quote! { #syntax_tree };

    // Wrap the generated code in a ros mod to namespace it
    // Add allow attributes for clippy lints that are triggered by generated code
    let namespaced_token_stream = quote! {
        #[allow(clippy::approx_constant)]
        pub mod ros {
            #token_stream
        }
    };

    // Convert TokenStream to string (formatted or compact)
    let mut generated_code = if config.format_code {
        // Parse and format using prettyplease for readable output
        let syntax_tree = syn::parse2(namespaced_token_stream)
            .context("Failed to parse generated token stream")?;
        prettyplease::unparse(&syntax_tree)
    } else {
        // Compact output
        namespaced_token_stream.to_string()
    };

    // Post-process to fix incorrect default code for arrays and vecs
    // roslibrust generates vec![] for both array and Vec fields, but:
    // - For array fields [T; N], we need plain array syntax: [values]
    // - For Vec fields ::std::vec::Vec<T>, we need vec syntax: [values].to_vec()

    // We need to process line by line to check the field type
    let lines: Vec<&str> = generated_code.lines().collect();
    let mut processed_lines = Vec::new();

    for (i, line) in lines.iter().enumerate() {
        let mut processed_line = line.to_string();

        // Check if this line has a default attribute with vec![]
        if line.contains("#[default(_code = \"vec![") {
            // Look ahead up to 5 lines to find the field declaration
            // (there might be other attributes like #[serde(...)] in between)
            let mut field_type_line = None;
            for offset in 1..=5 {
                if i + offset < lines.len() {
                    let next_line = lines[i + offset];
                    if next_line.contains("pub r#") {
                        field_type_line = Some(next_line);
                        break;
                    }
                }
            }

            if let Some(field_line) = field_type_line {
                // Check if the field is a fixed-size array [T; N]
                if field_line.contains(": [") && !field_line.contains("::std::vec::Vec") {
                    // It's a fixed-size array - convert vec![] to []
                    processed_line = processed_line.replace("#[default(_code = \"vec![", "#[default(_code = \"[");
                } else if field_line.contains("::std::vec::Vec") {
                    // It's a Vec - convert vec![] to [].to_vec()
                    processed_line = processed_line
                        .replace("#[default(_code = \"vec![", "#[default(_code = \"[")
                        .replace("]\")]", "].to_vec()\")]");
                }
            }
        }

        // Also handle iter().map().collect() pattern for string arrays
        // This pattern is: [\"\", \"max value\", \"min value\"].iter().map(|x| x.to_string()).collect()
        // Need to check if the field type (looking forward) is an array
        if line.contains(".iter().map(|x| x.to_string()).collect()") {
            let mut field_type_line = None;
            for offset in 1..=5 {
                if i + offset < lines.len() {
                    let next_line = lines[i + offset];
                    if next_line.contains("pub r#") {
                        field_type_line = Some(next_line);
                        break;
                    }
                }
            }

            if let Some(field_line) = field_type_line {
                // Check if it's a fixed-size array [String; N]
                if field_line.contains(": [") && field_line.contains("::std::string::String") && !field_line.contains("::std::vec::Vec") {
                    // Replace .iter().map().collect() with explicit array
                    // Note: The quotes are escaped as \" in the generated code
                    processed_line = processed_line.replace(
                        r#"[\"\", \"max value\", \"min value\"].iter().map(|x| x.to_string()).collect()"#,
                        r#"[\"\".to_string(), \"max value\".to_string(), \"min value\".to_string()]"#
                    );
                }
            }
        }

        processed_lines.push(processed_line);
    }

    generated_code = processed_lines.join("\n");

    // Write to output file
    let output_file = output_dir.join("generated.rs");
    fs::write(&output_file, generated_code)
        .with_context(|| format!("Failed to write generated code to {:?}", output_file))?;

    println!("cargo:rerun-if-changed=build.rs");

    Ok(())
}

/// Generate MessageTypeInfo trait implementations as TokenStreams
fn generate_message_type_info_tokens(
    messages: &[MessageFile],
    _msg_map: &BTreeMap<String, &MessageFile>,
) -> Result<TokenStream> {
    let mut tokens = TokenStream::new();

    for msg in messages {
        let package_name = syn::Ident::new(&msg.parsed.package, proc_macro2::Span::call_site());
        let msg_name = syn::Ident::new(&msg.parsed.name, proc_macro2::Span::call_site());

        let ros_type_name = format!("{}::msg::dds_::{}_", msg.parsed.package, msg.parsed.name);
        let type_hash = msg.ros2_hash.to_hash_string();

        let impl_block = quote! {
            impl ::ros_z::MessageTypeInfo for #package_name::#msg_name {
                fn type_name() -> &'static str {
                    #ros_type_name
                }

                fn type_hash() -> ::ros_z::entity::TypeHash {
                    ::ros_z::entity::TypeHash::from_rihs_string(#type_hash)
                        .expect("Invalid RIHS hash string")
                }

                fn type_info() -> ::ros_z::entity::TypeInfo {
                    ::ros_z::TypeInfo::new(Self::type_name(), Self::type_hash())
                }
            }

            impl ::ros_z::ros_msg::WithTypeInfo for #package_name::#msg_name {}
        };

        tokens.extend(impl_block);
    }

    Ok(tokens)
}

/// Generate ServiceTypeInfo trait implementations as TokenStreams
fn generate_service_type_info_tokens(services: &[ServiceFile]) -> Result<TokenStream> {
    let mut tokens = TokenStream::new();

    for srv in services {
        let package_name = syn::Ident::new(&srv.get_package_name(), proc_macro2::Span::call_site());
        let srv_name = syn::Ident::new(&srv.get_short_name(), proc_macro2::Span::call_site());
        let request_name = syn::Ident::new(&format!("{}Request", srv.get_short_name()), proc_macro2::Span::call_site());
        let response_name = syn::Ident::new(&format!("{}Response", srv.get_short_name()), proc_macro2::Span::call_site());

        let request_hash = srv.request().ros2_hash.to_hash_string();
        let response_hash = srv.response().ros2_hash.to_hash_string();
        let service_hash = srv.get_ros2_hash().to_hash_string();

        let request_ros_type = format!("{}::srv::dds_::{}_Request_", srv.get_package_name(), srv.get_short_name());
        let response_ros_type = format!("{}::srv::dds_::{}_Response_", srv.get_package_name(), srv.get_short_name());
        let service_ros_type = format!("{}::srv::dds_::{}_", srv.get_package_name(), srv.get_short_name());

        let impl_block = quote! {
            impl ::ros_z::MessageTypeInfo for #package_name::#request_name {
                fn type_name() -> &'static str {
                    #request_ros_type
                }

                fn type_hash() -> ::ros_z::entity::TypeHash {
                    ::ros_z::entity::TypeHash::from_rihs_string(#request_hash)
                        .expect("Invalid RIHS hash string")
                }

                fn type_info() -> ::ros_z::entity::TypeInfo {
                    ::ros_z::TypeInfo::new(Self::type_name(), Self::type_hash())
                }
            }

            impl ::ros_z::ros_msg::WithTypeInfo for #package_name::#request_name {}

            impl ::ros_z::MessageTypeInfo for #package_name::#response_name {
                fn type_name() -> &'static str {
                    #response_ros_type
                }

                fn type_hash() -> ::ros_z::entity::TypeHash {
                    ::ros_z::entity::TypeHash::from_rihs_string(#response_hash)
                        .expect("Invalid RIHS hash string")
                }

                fn type_info() -> ::ros_z::entity::TypeInfo {
                    ::ros_z::TypeInfo::new(Self::type_name(), Self::type_hash())
                }
            }

            impl ::ros_z::ros_msg::WithTypeInfo for #package_name::#response_name {}

            impl ::ros_z::msg::ZService for #package_name::#srv_name {
                type Request = #package_name::#request_name;
                type Response = #package_name::#response_name;
            }

            impl ::ros_z::ServiceTypeInfo for #package_name::#srv_name {
                fn service_type_info() -> ::ros_z::TypeInfo {
                    ::ros_z::TypeInfo::new(
                        #service_ros_type,
                        ::ros_z::TypeHash::from_rihs_string(#service_hash)
                            .expect("Invalid RIHS01 hash")
                    )
                }
            }
        };

        tokens.extend(impl_block);
    }

    Ok(tokens)
}
