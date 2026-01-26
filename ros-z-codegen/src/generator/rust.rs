use std::collections::HashSet;

use anyhow::Result;
use proc_macro2::{Ident, TokenStream};
use quote::{format_ident, quote};

use crate::types::{ArrayType, Field, FieldType, ResolvedMessage, ResolvedService};

/// Context for code generation, tracking external vs local packages
#[derive(Default, Clone)]
pub struct GenerationContext {
    /// External crate path for standard message types (e.g., "ros_z_msgs")
    pub external_crate: Option<String>,
    /// Set of local package names (packages being generated in this crate)
    pub local_packages: HashSet<String>,
}

impl GenerationContext {
    /// Create a new generation context
    pub fn new(external_crate: Option<String>, local_packages: HashSet<String>) -> Self {
        Self {
            external_crate,
            local_packages,
        }
    }

    /// Check if a package is local (being generated in this crate)
    pub fn is_local_package(&self, package: &str) -> bool {
        self.local_packages.is_empty() || self.local_packages.contains(package)
    }
}

/// Generate Rust module for a package containing messages
pub fn generate_package_module(package: &str, messages: &[ResolvedMessage]) -> Result<TokenStream> {
    let package_ident = format_ident!("{}", package);
    let message_impls: Vec<TokenStream> = messages
        .iter()
        .map(generate_message_impl)
        .collect::<Result<Vec<_>>>()?;

    Ok(quote! {
        pub mod #package_ident {
            #(#message_impls)*
        }
    })
}

/// Generate Rust implementation for a single message
pub fn generate_message_impl(msg: &ResolvedMessage) -> Result<TokenStream> {
    generate_message_impl_with_context(msg, &GenerationContext::default())
}

/// Generate Rust implementation for a single message with external type support
pub fn generate_message_impl_with_context(
    msg: &ResolvedMessage,
    ctx: &GenerationContext,
) -> Result<TokenStream> {
    let name = format_ident!("{}", msg.parsed.name);
    let has_zbuf = msg.parsed.fields.iter().any(is_zbuf_field);

    let struct_def = generate_struct_with_context(
        &msg.parsed.package,
        &msg.parsed.name,
        &msg.parsed.fields,
        &msg.parsed.constants,
        ctx,
    )?;
    let type_info =
        generate_message_type_info(&msg.parsed.package, &msg.parsed.name, &msg.type_hash);

    // Generate custom serde for ZBuf-containing messages
    let serde_impl = if has_zbuf {
        generate_zbuf_serde_with_context(&name, &msg.parsed.fields, &msg.parsed.package, ctx)?
    } else {
        quote! {}
    };

    Ok(quote! {
        #struct_def
        #type_info
        #serde_impl
    })
}

/// Generate struct definition with constants
#[allow(dead_code)]
fn generate_struct(
    package: &str,
    name: &str,
    fields: &[Field],
    constants: &[crate::types::Constant],
) -> Result<TokenStream> {
    generate_struct_with_context(
        package,
        name,
        fields,
        constants,
        &GenerationContext::default(),
    )
}

/// Generate struct definition with constants (with external type support)
fn generate_struct_with_context(
    package: &str,
    name: &str,
    fields: &[Field],
    constants: &[crate::types::Constant],
    ctx: &GenerationContext,
) -> Result<TokenStream> {
    let name_ident = format_ident!("{}", name);
    let field_defs: Vec<TokenStream> = fields
        .iter()
        .map(|f| generate_field_def_with_context(f, package, ctx))
        .collect::<Result<Vec<_>>>()?;

    // Check if we have large arrays (>32 elements) that need smart-default
    let has_large_array = fields
        .iter()
        .any(|f| matches!(&f.field_type.array, ArrayType::Fixed(n) if *n > 32));

    // Check if we have ZBuf fields (need custom serde)
    let has_zbuf = fields.iter().any(is_zbuf_field);

    // Generate constants as associated constants
    let const_defs: Vec<TokenStream> = constants
        .iter()
        .map(|c| {
            let const_name = format_ident!("{}", c.name);
            let const_type = generate_constant_type(&c.const_type);
            let const_value = generate_constant_value_from_string(&c.const_type, &c.value);
            quote! {
                pub const #const_name: #const_type = #const_value;
            }
        })
        .collect();

    // Python bridge module path for derive macros
    let py_module_path = format!("ros_z_msgs_py.types.{}", package);

    if has_zbuf {
        // Messages with ZBuf fields need custom serde (no derive)
        // But still need Default
        Ok(quote! {
            #[derive(Debug, Clone, Default)]
            #[cfg_attr(feature = "python_registry", derive(::ros_z_derive::FromPyMessage, ::ros_z_derive::IntoPyMessage))]
            #[cfg_attr(feature = "python_registry", ros_msg(module = #py_module_path))]
            pub struct #name_ident {
                #(#field_defs),*
            }

            impl #name_ident {
                #(#const_defs)*
            }
        })
    } else if has_large_array {
        // Large array messages need smart-default for arrays >32 elements
        Ok(quote! {
            #[derive(Debug, Clone, ::smart_default::SmartDefault, ::serde::Serialize, ::serde::Deserialize)]
            #[cfg_attr(feature = "python_registry", derive(::ros_z_derive::FromPyMessage, ::ros_z_derive::IntoPyMessage))]
            #[cfg_attr(feature = "python_registry", ros_msg(module = #py_module_path))]
            pub struct #name_ident {
                #(#field_defs),*
            }

            impl #name_ident {
                #(#const_defs)*
            }
        })
    } else {
        // Simple messages with standard Default
        Ok(quote! {
            #[derive(Debug, Clone, Default, ::serde::Serialize, ::serde::Deserialize)]
            #[cfg_attr(feature = "python_registry", derive(::ros_z_derive::FromPyMessage, ::ros_z_derive::IntoPyMessage))]
            #[cfg_attr(feature = "python_registry", ros_msg(module = #py_module_path))]
            pub struct #name_ident {
                #(#field_defs),*
            }

            impl #name_ident {
                #(#const_defs)*
            }
        })
    }
}

/// Generate constant type tokens
fn generate_constant_type(const_type: &str) -> TokenStream {
    match const_type {
        "bool" => quote! { bool },
        "byte" | "uint8" => quote! { u8 },
        "char" | "int8" => quote! { i8 },
        "uint16" => quote! { u16 },
        "int16" => quote! { i16 },
        "uint32" => quote! { u32 },
        "int32" => quote! { i32 },
        "uint64" => quote! { u64 },
        "int64" => quote! { i64 },
        "float32" => quote! { f32 },
        "float64" => quote! { f64 },
        "string" => quote! { &'static str },
        _ => quote! { &'static str },
    }
}

/// Generate constant value tokens from string representation
fn generate_constant_value_from_string(const_type: &str, value: &str) -> TokenStream {
    match const_type {
        "bool" => {
            let b: bool = value.parse().unwrap_or(false);
            quote! { #b }
        }
        "byte" | "uint8" | "char" | "int8" | "uint16" | "int16" | "uint32" | "int32" | "uint64"
        | "int64" => {
            let lit = syn::parse_str::<syn::LitInt>(value).unwrap();
            quote! { #lit }
        }
        "float32" | "float64" => {
            let lit = syn::parse_str::<syn::LitFloat>(value).unwrap();
            quote! { #lit }
        }
        _ => quote! { #value },
    }
}

/// Generate a field definition
#[allow(dead_code)]
fn generate_field_def(field: &Field, source_package: &str) -> Result<TokenStream> {
    generate_field_def_with_context(field, source_package, &GenerationContext::default())
}

/// Generate a field definition (with external type support)
fn generate_field_def_with_context(
    field: &Field,
    source_package: &str,
    ctx: &GenerationContext,
) -> Result<TokenStream> {
    // Escape Rust keywords with r# prefix
    let name = escape_field_name(&field.name);
    let field_type =
        generate_field_type_tokens_with_context(&field.field_type, source_package, ctx)?;

    // Check if this is a ZBuf field (for Python bridge attribute)
    let is_zbuf = is_zbuf_field(field);

    // Add attributes for large fixed arrays (>32 elements)
    let serde_attributes = if let ArrayType::Fixed(n) = &field.field_type.array {
        if *n > 32 {
            // Generate a default value code string for the array
            let default_code = generate_array_default_code(&field.field_type, *n)?;
            quote! {
                #[serde(with = "serde_big_array::BigArray")]
                #[default(_code = #default_code)]
            }
        } else {
            quote! {}
        }
    } else {
        quote! {}
    };

    // Add Python bridge attribute for ZBuf fields
    let python_attributes = if is_zbuf {
        quote! {
            #[cfg_attr(feature = "python_registry", ros_msg(zbuf))]
        }
    } else {
        quote! {}
    };

    Ok(quote! {
        #serde_attributes
        #python_attributes
        pub #name: #field_type
    })
}

/// Generate default value code string for an array
fn generate_array_default_code(field_type: &FieldType, size: usize) -> Result<String> {
    let elem_default = match field_type.base_type.as_str() {
        "bool" => "false",
        "byte" | "uint8" | "char" | "int8" | "uint16" | "int16" | "uint32" | "int32" | "uint64"
        | "int64" => "0",
        "float32" | "float64" => "0.0",
        _ => anyhow::bail!(
            "Cannot generate default for large array of type {}",
            field_type.base_type
        ),
    };

    Ok(format!("[{}; {}]", elem_default, size))
}

/// Check if a name is a Rust keyword
fn is_rust_keyword(name: &str) -> bool {
    matches!(
        name,
        "as" | "break"
            | "const"
            | "continue"
            | "crate"
            | "else"
            | "enum"
            | "extern"
            | "false"
            | "fn"
            | "for"
            | "if"
            | "impl"
            | "in"
            | "let"
            | "loop"
            | "match"
            | "mod"
            | "move"
            | "mut"
            | "pub"
            | "ref"
            | "return"
            | "self"
            | "Self"
            | "static"
            | "struct"
            | "super"
            | "trait"
            | "true"
            | "type"
            | "unsafe"
            | "use"
            | "where"
            | "while"
            | "async"
            | "await"
            | "dyn"
            | "abstract"
            | "become"
            | "box"
            | "do"
            | "final"
            | "macro"
            | "override"
            | "priv"
            | "typeof"
            | "unsized"
            | "virtual"
            | "yield"
            | "try"
    )
}

/// Escape a field name if it's a Rust keyword
fn escape_field_name(name: &str) -> Ident {
    if is_rust_keyword(name) {
        format_ident!("r#{}", name)
    } else {
        format_ident!("{}", name)
    }
}

/// Generate Rust type tokens for a field type
#[allow(dead_code)]
fn generate_field_type_tokens(field_type: &FieldType, source_package: &str) -> Result<TokenStream> {
    generate_field_type_tokens_with_context(
        field_type,
        source_package,
        &GenerationContext::default(),
    )
}

/// Generate Rust type tokens for a field type (with external type support)
fn generate_field_type_tokens_with_context(
    field_type: &FieldType,
    source_package: &str,
    ctx: &GenerationContext,
) -> Result<TokenStream> {
    let base = generate_base_type_tokens_with_context(field_type, source_package, ctx)?;

    Ok(match &field_type.array {
        ArrayType::Single => base,
        ArrayType::Fixed(n) => {
            let n_lit = proc_macro2::Literal::usize_unsuffixed(*n);
            quote! { [#base; #n_lit] }
        }
        ArrayType::Unbounded => {
            // Use ZBuf for uint8[]/byte[] (zero-copy optimization)
            if matches!(field_type.base_type.as_str(), "uint8" | "byte") {
                quote! { ::zenoh_buffers::ZBuf }
            } else {
                quote! { ::std::vec::Vec<#base> }
            }
        }
        ArrayType::Bounded(_n) => {
            // Using Vec<T> for bounded arrays (standard Rust approach)
            // Custom bounded type could be added for strict memory guarantees if needed
            quote! { ::std::vec::Vec<#base> }
        }
    })
}

/// Generate base type tokens
#[allow(dead_code)]
fn generate_base_type_tokens(field_type: &FieldType, source_package: &str) -> Result<TokenStream> {
    generate_base_type_tokens_with_context(
        field_type,
        source_package,
        &GenerationContext::default(),
    )
}

/// Generate base type tokens (with external type support)
fn generate_base_type_tokens_with_context(
    field_type: &FieldType,
    source_package: &str,
    ctx: &GenerationContext,
) -> Result<TokenStream> {
    Ok(match field_type.base_type.as_str() {
        "bool" => quote! { bool },
        "byte" | "uint8" => quote! { u8 },
        "char" | "int8" => quote! { i8 },
        "uint16" => quote! { u16 },
        "int16" => quote! { i16 },
        "uint32" => quote! { u32 },
        "int32" => quote! { i32 },
        "uint64" => quote! { u64 },
        "int64" => quote! { i64 },
        "float32" => quote! { f32 },
        "float64" => quote! { f64 },
        "string" => quote! { ::std::string::String },
        custom => {
            // Use explicit package or infer same package
            let pkg = field_type.package.as_deref().unwrap_or(source_package);
            let pkg_ident = format_ident!("{}", pkg);
            let type_ident = format_ident!("{}", custom);

            // Check if this is an external package reference
            if let Some(ref ext_crate) = ctx.external_crate
                && !ctx.is_local_package(pkg)
            {
                // External package - use fully qualified path
                let crate_ident = format_ident!("{}", ext_crate);
                return Ok(quote! { ::#crate_ident::ros::#pkg_ident::#type_ident });
            }

            // Local package - use super:: as before
            quote! { super::#pkg_ident::#type_ident }
        }
    })
}

/// Generate MessageTypeInfo trait implementation
fn generate_message_type_info(
    package: &str,
    name: &str,
    type_hash: &crate::types::TypeHash,
) -> TokenStream {
    let name_ident = format_ident!("{}", name);
    let type_name = format!("{}::msg::dds_::{}_", package, name);
    let hash_str = type_hash.to_rihs_string();

    quote! {
        impl ::ros_z::MessageTypeInfo for #name_ident {
            fn type_name() -> &'static str {
                #type_name
            }

            fn type_hash() -> ::ros_z::entity::TypeHash {
                ::ros_z::entity::TypeHash::from_rihs_string(#hash_str)
                    .expect("Invalid RIHS hash")
            }
        }

        impl ::ros_z::ros_msg::WithTypeInfo for #name_ident {}
    }
}

/// Generate custom serde implementation for ZBuf-containing messages
#[allow(dead_code)]
fn generate_zbuf_serde(
    name: &proc_macro2::Ident,
    fields: &[Field],
    source_package: &str,
) -> Result<TokenStream> {
    generate_zbuf_serde_with_context(name, fields, source_package, &GenerationContext::default())
}

/// Generate custom serde implementation for ZBuf-containing messages (with external type support)
fn generate_zbuf_serde_with_context(
    name: &proc_macro2::Ident,
    fields: &[Field],
    source_package: &str,
    ctx: &GenerationContext,
) -> Result<TokenStream> {
    let serialize_fields: Vec<TokenStream> = fields
        .iter()
        .map(|f| {
            let field_name = escape_field_name(&f.name);
            let field_name_str = &f.name;
            if is_zbuf_field(f) {
                // OPTIMIZED: Use a wrapper type that calls serialize_bytes() directly
                // instead of serialize_field() which treats &[u8] as a sequence.
                // This is critical for performance with large byte arrays!
                quote! {
                    {
                        use ::zenoh_buffers::buffer::SplitBuffer;
                        let bytes = self.#field_name.contiguous();
                        // Wrapper that calls serialize_bytes() directly for efficiency
                        struct BytesSerializer<'a>(&'a [u8]);
                        impl<'a> ::serde::Serialize for BytesSerializer<'a> {
                            fn serialize<S: ::serde::Serializer>(&self, serializer: S) -> ::std::result::Result<S::Ok, S::Error> {
                                serializer.serialize_bytes(self.0)
                            }
                        }
                        state.serialize_field(#field_name_str, &BytesSerializer(bytes.as_ref()))?;
                    }
                }
            } else {
                quote! {
                    state.serialize_field(#field_name_str, &self.#field_name)?;
                }
            }
        })
        .collect();

    // Generate sequential field deserialization for CDR (positional binary format)
    let seq_deserialize_fields: Vec<TokenStream> = fields
        .iter()
        .enumerate()
        .map(|(i, f)| {
            let field_name = escape_field_name(&f.name);
            let field_type = generate_field_type_tokens_with_context(&f.field_type, source_package, ctx).unwrap();
            if is_zbuf_field(f) {
                // OPTIMIZED: Use a wrapper type that calls deserialize_bytes() directly
                // instead of the default Vec<u8> deserialization which uses deserialize_seq().
                // This is critical for performance with large byte arrays!
                quote! {
                    let #field_name: #field_type = {
                        // Wrapper that uses deserialize_bytes for efficient bulk reading
                        struct BytesDeserializer;
                        impl<'de> ::serde::de::DeserializeSeed<'de> for BytesDeserializer {
                            type Value = ::std::vec::Vec<u8>;
                            fn deserialize<D: ::serde::Deserializer<'de>>(self, deserializer: D) -> ::std::result::Result<Self::Value, D::Error> {
                                struct BytesVisitor;
                                impl<'de> ::serde::de::Visitor<'de> for BytesVisitor {
                                    type Value = ::std::vec::Vec<u8>;
                                    fn expecting(&self, formatter: &mut ::std::fmt::Formatter) -> ::std::fmt::Result {
                                        formatter.write_str("byte array")
                                    }
                                    fn visit_bytes<E: ::serde::de::Error>(self, v: &[u8]) -> ::std::result::Result<Self::Value, E> {
                                        Ok(v.to_vec())
                                    }
                                    fn visit_borrowed_bytes<E: ::serde::de::Error>(self, v: &'de [u8]) -> ::std::result::Result<Self::Value, E> {
                                        Ok(v.to_vec())
                                    }
                                    fn visit_byte_buf<E: ::serde::de::Error>(self, v: ::std::vec::Vec<u8>) -> ::std::result::Result<Self::Value, E> {
                                        Ok(v)
                                    }
                                    // Fallback for formats that don't support bytes natively
                                    fn visit_seq<A: ::serde::de::SeqAccess<'de>>(self, mut seq: A) -> ::std::result::Result<Self::Value, A::Error> {
                                        let len = seq.size_hint().unwrap_or(0);
                                        let mut bytes = ::std::vec::Vec::with_capacity(len);
                                        while let Some(b) = seq.next_element()? {
                                            bytes.push(b);
                                        }
                                        Ok(bytes)
                                    }
                                }
                                deserializer.deserialize_bytes(BytesVisitor)
                            }
                        }
                        let bytes: ::std::vec::Vec<u8> = seq.next_element_seed(BytesDeserializer)?
                            .ok_or_else(|| ::serde::de::Error::invalid_length(#i, &self))?;
                        ::zenoh_buffers::ZBuf::from(bytes)
                    };
                }
            } else {
                quote! {
                    let #field_name: #field_type = seq.next_element()?
                        .ok_or_else(|| ::serde::de::Error::invalid_length(#i, &self))?;
                }
            }
        })
        .collect();

    let field_names: Vec<_> = fields.iter().map(|f| escape_field_name(&f.name)).collect();
    let field_name_strs: Vec<_> = fields.iter().map(|f| &f.name).collect();
    let num_fields = fields.len();

    Ok(quote! {
        impl ::serde::Serialize for #name {
            fn serialize<S>(&self, serializer: S) -> ::std::result::Result<S::Ok, S::Error>
            where
                S: ::serde::Serializer,
            {
                use ::serde::ser::SerializeStruct;
                let mut state = serializer.serialize_struct(stringify!(#name), #num_fields)?;
                #(#serialize_fields)*
                state.end()
            }
        }

        impl<'de> ::serde::Deserialize<'de> for #name {
            fn deserialize<D>(deserializer: D) -> ::std::result::Result<Self, D::Error>
            where
                D: ::serde::Deserializer<'de>,
            {
                use ::serde::de::{SeqAccess, Visitor};
                use ::std::fmt;

                struct FieldVisitor;

                impl<'de> Visitor<'de> for FieldVisitor {
                    type Value = #name;

                    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                        formatter.write_str(concat!("struct ", stringify!(#name)))
                    }

                    fn visit_seq<V>(self, mut seq: V) -> ::std::result::Result<Self::Value, V::Error>
                    where
                        V: SeqAccess<'de>,
                    {
                        #(#seq_deserialize_fields)*

                        Ok(#name {
                            #(#field_names),*
                        })
                    }
                }

                const FIELDS: &[&str] = &[#(#field_name_strs),*];
                deserializer.deserialize_struct(stringify!(#name), FIELDS, FieldVisitor)
            }
        }
    })
}

/// Generate service type implementation
pub fn generate_service_impl(srv: &ResolvedService) -> Result<TokenStream> {
    let name = format_ident!("{}", srv.parsed.name);
    let request_type = format_ident!("{}Request", srv.parsed.name);
    let response_type = format_ident!("{}Response", srv.parsed.name);
    let service_type_name = format!("{}::srv::dds_::{}_", srv.parsed.package, srv.parsed.name);
    let hash_str = srv.type_hash.to_rihs_string();

    Ok(quote! {
        pub struct #name;

        impl ::ros_z::msg::ZService for #name {
            type Request = super::#request_type;
            type Response = super::#response_type;
        }

        impl ::ros_z::ServiceTypeInfo for #name {
            fn service_type_info() -> ::ros_z::entity::TypeInfo {
                ::ros_z::entity::TypeInfo::new(
                    #service_type_name,
                    ::ros_z::entity::TypeHash::from_rihs_string(#hash_str)
                        .expect("Invalid RIHS hash")
                )
            }
        }
    })
}

/// Generate action type implementation
/// Actions are generated similarly to services - the struct is at the root level,
/// while Goal/Result/Feedback types are in the package module
pub fn generate_action_impl(action: &crate::types::ResolvedAction) -> Result<TokenStream> {
    let name = format_ident!("{}", action.parsed.name);
    let goal_type = format_ident!("{}Goal", action.parsed.name);
    let result_type = format_ident!("{}Result", action.parsed.name);
    let feedback_type = format_ident!("{}Feedback", action.parsed.name);
    let action_type_name = format!(
        "{}::action::dds_::{}_",
        action.parsed.package, action.parsed.name
    );
    let hash_str = action.type_hash.to_rihs_string();

    // Type names for action-related services and messages (ROS2 format with underscore)
    let send_goal_type_name = format!(
        "{}::action::dds_::{}_SendGoal_",
        action.parsed.package, action.parsed.name
    );
    let get_result_type_name = format!(
        "{}::action::dds_::{}_GetResult_",
        action.parsed.package, action.parsed.name
    );
    let feedback_msg_type_name = format!(
        "{}::action::dds_::{}_FeedbackMessage_",
        action.parsed.package, action.parsed.name
    );

    // Get type hashes from resolved action service hashes (not the Goal/Result/Feedback hashes)
    let send_goal_hash_str = action.send_goal_hash.to_rihs_string();
    let get_result_hash_str = action.get_result_hash.to_rihs_string();
    let feedback_hash_str = action.feedback_message_hash.to_rihs_string();
    let cancel_goal_hash_str = action.cancel_goal_hash.to_rihs_string();
    let status_hash_str = action.status_hash.to_rihs_string();

    Ok(quote! {
        pub struct #name;

        impl ::ros_z::action::ZAction for #name {
            type Goal = super::#goal_type;
            type Result = super::#result_type;
            type Feedback = super::#feedback_type;

            fn name() -> &'static str {
                #action_type_name
            }

            fn send_goal_type_info() -> ::ros_z::entity::TypeInfo {
                ::ros_z::entity::TypeInfo::new(
                    #send_goal_type_name,
                    ::ros_z::entity::TypeHash::from_rihs_string(#send_goal_hash_str)
                        .expect("Invalid RIHS hash")
                )
            }

            fn get_result_type_info() -> ::ros_z::entity::TypeInfo {
                ::ros_z::entity::TypeInfo::new(
                    #get_result_type_name,
                    ::ros_z::entity::TypeHash::from_rihs_string(#get_result_hash_str)
                        .expect("Invalid RIHS hash")
                )
            }

            fn cancel_goal_type_info() -> ::ros_z::entity::TypeInfo {
                ::ros_z::entity::TypeInfo::new(
                    "action_msgs::srv::dds_::CancelGoal_",
                    ::ros_z::entity::TypeHash::from_rihs_string(#cancel_goal_hash_str)
                        .expect("Invalid RIHS hash")
                )
            }

            fn feedback_type_info() -> ::ros_z::entity::TypeInfo {
                ::ros_z::entity::TypeInfo::new(
                    #feedback_msg_type_name,
                    ::ros_z::entity::TypeHash::from_rihs_string(#feedback_hash_str)
                        .expect("Invalid RIHS hash")
                )
            }

            fn status_type_info() -> ::ros_z::entity::TypeInfo {
                ::ros_z::entity::TypeInfo::new(
                    "action_msgs::msg::dds_::GoalStatusArray_",
                    ::ros_z::entity::TypeHash::from_rihs_string(#status_hash_str)
                        .expect("Invalid RIHS hash")
                )
            }
        }

        impl ::ros_z::ActionTypeInfo for #name {
            fn action_type_info() -> ::ros_z::entity::TypeInfo {
                ::ros_z::entity::TypeInfo::new(
                    #action_type_name,
                    ::ros_z::entity::TypeHash::from_rihs_string(#hash_str)
                        .expect("Invalid RIHS hash")
                )
            }
        }
    })
}

/// Check if a field uses ZBuf (uint8[] or byte[])
fn is_zbuf_field(field: &Field) -> bool {
    matches!(
        (field.field_type.base_type.as_str(), &field.field_type.array),
        ("uint8" | "byte", ArrayType::Unbounded)
    )
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;
    use crate::types::{ParsedMessage, TypeHash};

    #[test]
    fn test_is_zbuf_field() {
        let field = Field {
            name: "data".to_string(),
            field_type: FieldType {
                base_type: "uint8".to_string(),
                package: None,
                array: ArrayType::Unbounded,
            },
            default: None,
        };
        assert!(is_zbuf_field(&field));

        let field = Field {
            name: "data".to_string(),
            field_type: FieldType {
                base_type: "byte".to_string(),
                package: None,
                array: ArrayType::Unbounded,
            },
            default: None,
        };
        assert!(is_zbuf_field(&field));

        let field = Field {
            name: "data".to_string(),
            field_type: FieldType {
                base_type: "int32".to_string(),
                package: None,
                array: ArrayType::Unbounded,
            },
            default: None,
        };
        assert!(!is_zbuf_field(&field));
    }

    #[test]
    fn test_generate_simple_message() {
        let msg = ResolvedMessage {
            parsed: ParsedMessage {
                name: "Simple".to_string(),
                package: "test_msgs".to_string(),
                fields: vec![Field {
                    name: "value".to_string(),
                    field_type: FieldType {
                        base_type: "int32".to_string(),
                        package: None,
                        array: ArrayType::Single,
                    },
                    default: None,
                }],
                constants: vec![],
                source: String::new(),
                path: PathBuf::new(),
            },
            type_hash: TypeHash([0u8; 32]),
            definition: String::new(),
        };

        let result = generate_message_impl(&msg);
        assert!(result.is_ok());

        let tokens = result.unwrap();
        let code = tokens.to_string();

        // Should contain struct definition
        assert!(code.contains("struct Simple"));
        // Should contain field
        assert!(code.contains("pub value"));
        // Should derive Serialize/Deserialize (no ZBuf)
        assert!(code.contains("Serialize"));
        assert!(code.contains("Deserialize"));
        // Should have MessageTypeInfo
        assert!(code.contains("MessageTypeInfo"));
    }

    #[test]
    fn test_generate_byte_array_message() {
        let msg = ResolvedMessage {
            parsed: ParsedMessage {
                name: "Image".to_string(),
                package: "test_msgs".to_string(),
                fields: vec![
                    Field {
                        name: "width".to_string(),
                        field_type: FieldType {
                            base_type: "uint32".to_string(),
                            package: None,
                            array: ArrayType::Single,
                        },
                        default: None,
                    },
                    Field {
                        name: "data".to_string(),
                        field_type: FieldType {
                            base_type: "uint8".to_string(),
                            package: None,
                            array: ArrayType::Unbounded,
                        },
                        default: None,
                    },
                ],
                constants: vec![],
                source: String::new(),
                path: PathBuf::new(),
            },
            type_hash: TypeHash([0u8; 32]),
            definition: String::new(),
        };

        let result = generate_message_impl(&msg);
        assert!(result.is_ok());

        let tokens = result.unwrap();
        let code = tokens.to_string();

        // Should contain ZBuf field (optimized for zero-copy)
        assert!(code.contains("zenoh_buffers :: ZBuf"));
        // Should have custom Serialize implementation (not derived)
        assert!(code.contains("impl :: serde :: Serialize"));
        assert!(code.contains("impl < 'de > :: serde :: Deserialize"));
    }

    #[test]
    fn test_generate_service() {
        let request = ParsedMessage {
            name: "AddTwoIntsRequest".to_string(),
            package: "example_interfaces".to_string(),
            fields: vec![
                Field {
                    name: "a".to_string(),
                    field_type: FieldType {
                        base_type: "int64".to_string(),
                        package: None,
                        array: ArrayType::Single,
                    },
                    default: None,
                },
                Field {
                    name: "b".to_string(),
                    field_type: FieldType {
                        base_type: "int64".to_string(),
                        package: None,
                        array: ArrayType::Single,
                    },
                    default: None,
                },
            ],
            constants: vec![],
            source: String::new(),
            path: PathBuf::new(),
        };

        let response = ParsedMessage {
            name: "AddTwoIntsResponse".to_string(),
            package: "example_interfaces".to_string(),
            fields: vec![Field {
                name: "sum".to_string(),
                field_type: FieldType {
                    base_type: "int64".to_string(),
                    package: None,
                    array: ArrayType::Single,
                },
                default: None,
            }],
            constants: vec![],
            source: String::new(),
            path: PathBuf::new(),
        };

        let srv = ResolvedService {
            parsed: crate::types::ParsedService {
                name: "AddTwoInts".to_string(),
                package: "example_interfaces".to_string(),
                request: request.clone(),
                response: response.clone(),
                source: String::new(),
                path: PathBuf::new(),
            },
            request: ResolvedMessage {
                parsed: request,
                type_hash: TypeHash([0u8; 32]),
                definition: String::new(),
            },
            response: ResolvedMessage {
                parsed: response,
                type_hash: TypeHash([0u8; 32]),
                definition: String::new(),
            },
            type_hash: TypeHash([0u8; 32]),
        };

        let result = generate_service_impl(&srv);
        assert!(result.is_ok());

        let tokens = result.unwrap();
        let code = tokens.to_string();

        assert!(code.contains("struct AddTwoInts"));
        assert!(code.contains("ServiceTypeInfo"));
        assert!(code.contains("AddTwoIntsRequest"));
        assert!(code.contains("AddTwoIntsResponse"));
    }
}
