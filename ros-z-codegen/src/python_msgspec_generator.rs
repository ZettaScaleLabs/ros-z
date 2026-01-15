//! Auto-generation of Python msgspec structs and PyO3 bindings for ROS 2 messages
//!
//! This module generates both Python msgspec structs and complete Rust PyO3 modules
//! from ROS message definitions, eliminating the need for manual registry code.

use anyhow::Result;
use roslibrust_codegen::MessageFile;
use std::collections::HashMap;
use std::fs;
use std::path::Path;
use crate::codegen::extraction;

/// Generate both Python msgspec structs AND complete Rust PyO3 module
pub fn generate_python_bindings(
    messages: &[MessageFile],
    python_output_dir: &Path,
    rust_output_path: &Path,
) -> Result<()> {
    // Group messages by package
    let mut packages: HashMap<String, Vec<&MessageFile>> = HashMap::new();
    for msg in messages {
        packages.entry(msg.parsed.package.clone())
            .or_default()
            .push(msg);
    }

    // Generate Python msgspec structs (one file per package)
    for (package_name, package_msgs) in &packages {
        let python_code = generate_python_package(package_name, package_msgs)?;
        let output_path = python_output_dir.join(format!("{}.py", package_name));
        fs::write(output_path, python_code)?;
    }

    // Generate __init__.py for Python package
    let init_code = generate_python_init(&packages)?;
    fs::write(python_output_dir.join("__init__.py"), init_code)?;

    // Generate COMPLETE Rust PyO3 module (replaces python_registry.rs entirely)
    let rust_module = generate_complete_rust_module(&packages)?;
    fs::write(rust_output_path, rust_module)?;

    Ok(())
}

/// Generate Python msgspec structs for a package
fn generate_python_package(
    package_name: &str,
    messages: &[&MessageFile],
) -> Result<String> {
    let mut code = format!(
        "\"\"\"Auto-generated ROS 2 message types for {}.\"\"\"\n\
         import msgspec\n\
         from typing import ClassVar\n\n",
        package_name
    );

    for msg in messages {
        code.push_str(&generate_msgspec_struct(msg)?);
    }

    Ok(code)
}

fn rust_to_python_type(field_type: &roslibrust_codegen::FieldType) -> Result<String> {
    // Get the base field type (without array indicators)
    let base_type = &field_type.field_type;

    // Check if this is an array type
    let is_array = !matches!(
        field_type.array_info,
        roslibrust_codegen::ArrayType::NotArray
    );

    // First check if this is a primitive type
    let python_type = match base_type.as_str() {
        "bool" => "bool",
        "byte" | "int8" | "char" | "uint8" | "int16" | "uint16" | "int32" | "uint32" | "int64" | "uint64" => "int",
        "float32" | "float64" => "float",
        "string" | "wstring" => "str",
        _ => {
            // Not a primitive type - check if it's a nested message
            // Try to construct the fully qualified type name
            if let Some(ref package_name) = field_type.package_name {
                // Has an explicit package name - it's a nested message
                return Ok(if is_array {
                    format!("list[\"{}.{}\"]", package_name, base_type)
                } else {
                    format!("\"{}.{}\"", package_name, base_type)
                });
            } else if !base_type.contains("/") {
                // No package prefix and no explicit package_name
                // This is likely a nested message from the same package or unresolved
                // Use source_package as fallback
                let package = &field_type.source_package;
                return Ok(if is_array {
                    format!("list[\"{}.{}\"]", package, base_type)
                } else {
                    format!("\"{}.{}\"", package, base_type)
                });
            } else {
                // Has "/" in the type name - parse it
                let parts: Vec<&str> = base_type.split("/").collect();
                if parts.len() == 3 && parts[1] == "msg" {
                    return Ok(if is_array {
                        format!("list[\"{}.{}\"]", parts[0], parts[2])
                    } else {
                        format!("\"{}.{}\"", parts[0], parts[2])
                    });
                } else if parts.len() == 2 {
                    // Format: "package/Type"
                    return Ok(if is_array {
                        format!("list[\"{}.{}\"]", parts[0], parts[1])
                    } else {
                        format!("\"{}.{}\"", parts[0], parts[1])
                    });
                } else {
                    return Err(anyhow::anyhow!("Unsupported type format: {}", base_type));
                }
            }
        }
    };

    // Return primitive type, with list wrapper if it's an array
    Ok(if is_array {
        format!("list[{}]", python_type)
    } else {
        python_type.to_string()
    })
}

fn get_python_default(field_type: &roslibrust_codegen::FieldType) -> String {
    let type_str = field_type.to_string();
    match type_str.as_str() {
        "bool" => "False".to_string(),
        "byte" | "int8" | "char" | "uint8" | "int16" | "uint16" | "int32" | "uint32" | "int64" | "uint64" => "0".to_string(),
        "float32" | "float64" => "0.0".to_string(),
        "string" | "wstring" => "\"\"".to_string(),
        type_str if type_str.contains("Time") || type_str.contains("Duration") => "msgspec.field(default_factory=lambda: {'sec': 0, 'nanosec': 0})".to_string(),
        type_str if type_str.ends_with("[]") => "msgspec.field(default_factory=list)".to_string(),
        _ => "msgspec.field(default_factory=dict)".to_string(), // For nested types
    }
}

fn escape_rust_keyword(name: &str) -> String {
    match name {
        "type" | "struct" | "enum" | "fn" | "impl" | "trait" | "use" | "mod" | "pub" | "crate" | "super" | "self" => format!("r#{}", name),
        _ => name.to_string(),
    }
}

// Helper to generate nested field constructions with proper field access
fn generate_python_nested_field_constructions(
    fields: &[roslibrust_codegen::FieldInfo],
    all_messages: &HashMap<String, &MessageFile>,
    _current_package: &str,
    item_var: &str,
) -> Result<String> {
    let mut code = String::new();
    for field in fields {
        let field_name = &field.field_name;
        let escaped_name = escape_rust_keyword(field_name);
        let base_type = &field.field_type.field_type;

        let is_array = !matches!(
            field.field_type.array_info,
            roslibrust_codegen::ArrayType::NotArray
        );

        if is_array {
            // Handle arrays of nested messages recursively
            let nested_msg_key = if let Some(ref package_name) = field.field_type.package_name {
                format!("{}/{}", package_name, base_type)
            } else {
                format!("{}/{}", field.field_type.source_package, base_type)
            };

            if let Some(nested_msg) = all_messages.get(&nested_msg_key) {
                // Generate recursive construction for array of nested messages
                let nested_package = if let Some(ref package_name) = field.field_type.package_name {
                    package_name.clone()
                } else {
                    field.field_type.source_package.clone()
                };

                let inner_constructions = generate_python_nested_field_constructions(
                    &nested_msg.parsed.fields,
                    all_messages,
                    &nested_package,
                    "inner_item"
                )?;

                code.push_str(&format!(
                    "             {{\n\
                     let inner_list = pyo3::types::PyList::empty_bound(py);\n\
                     let inner_types = py.import_bound(\"ros_z_python.types.{}\")?;\n\
                     let inner_class = inner_types.getattr(\"{}\")?;\n\
                     for inner_item in &{}.{} {{\n\
                     let inner_kwargs = PyDict::new_bound(py);\n\
                     {}\
                     let inner_obj = inner_class.call((), Some(&inner_kwargs))?;\n\
                     inner_list.append(inner_obj)?;\n\
                     }}\n\
                     nested_kwargs.set_item(\"{}\", inner_list)?;\n\
                     }}\n",
                    nested_package,
                    base_type,
                    item_var,
                    escaped_name,
                    inner_constructions,
                    field_name
                ));
            } else {
                // Fallback for unknown array types
                code.push_str(&format!(
                    "             nested_kwargs.set_item(\"{}\", {}.{}.to_vec())?;\n",
                    field_name, item_var, escaped_name
                ));
            }
        } else {
            match base_type.as_str() {
                "bool" | "byte" | "char" | "int8" | "uint8" | "int16" | "uint16" |
                "int32" | "uint32" | "int64" | "uint64" | "float32" | "float64" => {
                    code.push_str(&format!(
                        "             nested_kwargs.set_item(\"{}\", {}.{})?;\n",
                        field_name, item_var, escaped_name
                    ));
                },
                "string" => {
                    // For strings, we need to borrow as &str since item_var is often a reference
                    code.push_str(&format!(
                        "             nested_kwargs.set_item(\"{}\", &*{}.{})?;\n",
                        field_name, item_var, escaped_name
                    ));
                },
                _ => {
                    // Handle nested messages recursively
                    let nested_msg_key = if let Some(ref package_name) = field.field_type.package_name {
                        format!("{}/{}", package_name, base_type)
                    } else {
                        format!("{}/{}", field.field_type.source_package, base_type)
                    };

                    if let Some(nested_msg) = all_messages.get(&nested_msg_key) {
                        let nested_package = if let Some(ref package_name) = field.field_type.package_name {
                            package_name.clone()
                        } else {
                            field.field_type.source_package.clone()
                        };

                        // Build the field access path for nested structs
                        let nested_item_var = format!("{}.{}", item_var, escaped_name);

                        let inner_constructions = generate_python_nested_field_constructions(
                            &nested_msg.parsed.fields,
                            all_messages,
                            &nested_package,
                            &nested_item_var
                        )?;

                        code.push_str(&format!(
                            "             {{\n\
                             let inner_types = py.import_bound(\"ros_z_python.types.{}\")?;\n\
                             let inner_class = inner_types.getattr(\"{}\")?;\n\
                             let inner_kwargs = PyDict::new_bound(py);\n\
                             {}\
                             let inner_obj = inner_class.call((), Some(&inner_kwargs))?;\n\
                             nested_kwargs.set_item(\"{}\", inner_obj)?;\n\
                             }}\n",
                            nested_package,
                            base_type,
                            inner_constructions,
                            field_name
                        ));
                    } else {
                        // Fallback for unknown types
                        code.push_str(&format!(
                            "             nested_kwargs.set_item(\"{}\", PyDict::new_bound(py))?;\n",
                            field_name
                        ));
                    }
                }
            }
        }
    }
    Ok(code)
}

fn generate_python_field_construction(
    field: &roslibrust_codegen::FieldInfo,
    all_messages: &HashMap<String, &MessageFile>,
    _current_package: &str,
) -> Result<String> {
    let field_name = &field.field_name;
    let escaped_name = escape_rust_keyword(field_name);

    // Check if it's an array
    let is_array = !matches!(
        field.field_type.array_info,
        roslibrust_codegen::ArrayType::NotArray
    );

    // Check if it's a fixed-size array
    let is_fixed_size = matches!(
        field.field_type.array_info,
        roslibrust_codegen::ArrayType::FixedLength(_)
    );

    let base_type = &field.field_type.field_type;

    if is_array {
        // Arrays - handle based on element type
        match base_type.as_str() {
            // Primitive arrays - direct conversion
            "bool" | "byte" | "char" | "int8" | "uint8" | "int16" | "uint16" |
            "int32" | "uint32" | "int64" | "uint64" | "float32" | "float64" | "string" => {
                if is_fixed_size {
                    // Fixed-size arrays need to_vec() conversion
                    Ok(format!(
                        "        kwargs.set_item(\"{}\", rust_msg.{}.to_vec())?;\n",
                        field_name, escaped_name
                    ))
                } else {
                    // Dynamic arrays can be passed directly
                    Ok(format!(
                        "        kwargs.set_item(\"{}\", &rust_msg.{})?;\n",
                        field_name, escaped_name
                    ))
                }
            },
            // Arrays of nested messages
            _ => {
                let package = if let Some(ref package_name) = field.field_type.package_name {
                    package_name.clone()
                } else {
                    field.field_type.source_package.clone()
                };

                let nested_msg_key = format!("{}/{}", package, base_type);
                let nested_msg = all_messages.get(&nested_msg_key)
                    .ok_or_else(|| anyhow::anyhow!("Message {} not found", nested_msg_key))?;

                // For array elements, use "item" as the variable name since we loop over items
                let nested_constructions = generate_python_nested_field_constructions(
                    &nested_msg.parsed.fields,
                    all_messages,
                    &package,
                    "item"
                )?;

                Ok(format!(
                    "        {{\n\
                     let nested_list = pyo3::types::PyList::empty_bound(py);\n\
                     let nested_types = py.import_bound(\"ros_z_python.types.{}\")?;\n\
                     let nested_class = nested_types.getattr(\"{}\")?;\n\
                     for item in &rust_msg.{} {{\n\
                     let nested_kwargs = PyDict::new_bound(py);\n\
                     {}\
                     let nested_obj = nested_class.call((), Some(&nested_kwargs))?;\n\
                     nested_list.append(nested_obj)?;\n\
                     }}\n\
                     kwargs.set_item(\"{}\", nested_list)?;\n\
                     }}\n",
                    package,
                    base_type,
                    escaped_name,
                    nested_constructions,
                    field_name
                ))
            }
        }
    } else {
        match base_type.as_str() {
            // Primitive types - direct construction
            "bool" | "byte" | "char" | "int8" | "uint8" | "int16" | "uint16" |
            "int32" | "uint32" | "int64" | "uint64" | "float32" | "float64" | "string" => {
                Ok(format!(
                    "        kwargs.set_item(\"{}\", rust_msg.{})?;\n",
                    field_name, escaped_name
                ))
            },

            // Nested message - recursively construct
            _ => {
                let package = if let Some(ref package_name) = field.field_type.package_name {
                    package_name.clone()
                } else {
                    field.field_type.source_package.clone()
                };

                let nested_msg_key = format!("{}/{}", package, base_type);
                let nested_msg = all_messages.get(&nested_msg_key)
                    .ok_or_else(|| anyhow::anyhow!("Message {} not found", nested_msg_key))?;

                // Build the field access path for the nested message
                let nested_item_var = format!("rust_msg.{}", escaped_name);

                let nested_constructions = generate_python_nested_field_constructions(
                    &nested_msg.parsed.fields,
                    all_messages,
                    &package,
                    &nested_item_var
                )?;

                Ok(format!(
                    "        {{\n\
                     let nested_types = py.import_bound(\"ros_z_python.types.{}\")?;\n\
                     let nested_class = nested_types.getattr(\"{}\")?;\n\
                     let nested_kwargs = PyDict::new_bound(py);\n\
                     {}\
                     let nested_obj = nested_class.call((), Some(&nested_kwargs))?;\n\
                     kwargs.set_item(\"{}\", nested_obj)?;\n\
                     }}\n",
                    package,
                    base_type,
                    nested_constructions,
                    field_name
                ))
            }
        }
    }
}

fn generate_msgspec_struct(msg: &MessageFile) -> Result<String> {
    let name = &msg.parsed.name;
    let hash = msg.ros2_hash.to_hash_string();

    let mut code = format!(
        "class {}(msgspec.Struct, frozen=True, kw_only=True):\n",
        name
    );

    // Generate fields
    for field in &msg.parsed.fields {
        let py_type = rust_to_python_type(&field.field_type)?;
        let default = get_python_default(&field.field_type);
        code.push_str(&format!("    {}: {} = {}\n", field.field_name, py_type, default));
    }

    // Add metadata
    code.push_str(&format!(
        "\n    __msgtype__: ClassVar[str] = '{}/msg/{}'\n",
        msg.parsed.package, name
    ));
    code.push_str(&format!("    __hash__: ClassVar[str] = '{}'\n\n", hash));

    Ok(code)
}

/// Generate COMPLETE Rust module with PyO3 bindings
fn generate_complete_rust_module(
    packages: &HashMap<String, Vec<&MessageFile>>,
) -> Result<String> {
    // Build lookup map for all messages
    let mut all_messages: HashMap<String, &MessageFile> = HashMap::new();
    for package_msgs in packages.values() {
        for msg in package_msgs {
            let key = format!("{}/{}", msg.parsed.package, msg.parsed.name);
            all_messages.insert(key, *msg);
        }
    }

    let mut code = String::from(
        "/// Auto-generated Python bindings for ROS 2 messages\n\
         /// Generated by ros-z-codegen - DO NOT EDIT\n\n\
         use pyo3::prelude::*;\n\
         use pyo3::types::{PyAny, PyDict, PyModule};\n\
         use cdr::{CdrLe, Infinite};\n\n"
    );

    // Generate serialize/deserialize functions for each message
    for (package_name, package_msgs) in packages {
        code.push_str("#[allow(unsafe_op_in_unsafe_fn)]\n");
        code.push_str("mod ");
        code.push_str(&package_name.replace("-", "_"));
        code.push_str("_py {\n");
        code.push_str("    use super::*;\n\n");

        for msg in package_msgs {
            code.push_str(&generate_message_functions(msg, &all_messages)?);
        }

        code.push_str("}\n\n");
    }

    // Generate the main PyO3 module with auto-registration
    code.push_str(&generate_pymodule_registration(packages)?);

    Ok(code)
}

/// Generate serialize/deserialize functions for a single message using TokenStream
fn generate_message_functions(
    msg: &MessageFile,
    all_messages: &HashMap<String, &MessageFile>,
) -> Result<String> {
    let package = &msg.parsed.package;
    let name = &msg.parsed.name;

    // Generate serialize function using TokenStream
    let serialize_fn = extraction::generate_serialize_function(msg, all_messages)?;
    
    // Generate deserialize function (keep string-based for now as it's not in the proposal)
    let mut code = serialize_fn.to_string();
    code.push_str("\n\n");
    
    // Generate deserialize function
    code.push_str(&format!(
        "    #[pyfunction]\n\
         pub fn deserialize_{}(py: Python, bytes: &[u8]) -> PyResult<PyObject> {{\n",
        name.to_lowercase()
    ));

    let rust_msg_prefix = if msg.parsed.fields.is_empty() { "_" } else { "" };
    code.push_str(&format!(
        "        let {}rust_msg: {}::{} = cdr::deserialize(bytes)\n\
         .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;\n\n",
        rust_msg_prefix, package, name
    ));

    // Import Python class
    code.push_str(&format!(
        "        let types = py.import_bound(\"ros_z_python.types.{}\")?\n\
         let msg_class = types.getattr(\"{}\")?;\n\n",
        package, name
    ));

    // Build kwargs dict
    code.push_str("        let kwargs = PyDict::new_bound(py);\n");
    for field in &msg.parsed.fields {
        code.push_str(&generate_python_field_construction(field, all_messages, package)?);
    }

    code.push_str(
        "        msg_class.call((), Some(&kwargs)).map(|obj| obj.unbind())\n\
         }\n\n"
    );

    Ok(code)
}

/// Generate PyO3 module with automatic registration
fn generate_pymodule_registration(
    packages: &HashMap<String, Vec<&MessageFile>>,
) -> Result<String> {
    let mut code = String::from(
        "#[allow(unsafe_op_in_unsafe_fn)]\n\
         #[pymodule]\n\
         pub fn ros_z_msgs(py: Python, m: &Bound<'_, PyModule>) -> PyResult<()> {\n\
         // Create message type registry as Python dict\n\
         let registry = PyDict::new_bound(py);\n\n"
    );

    // Auto-register all message types
    for (package_name, package_msgs) in packages {
        code.push_str("    {\n");

        for msg in package_msgs {
            let name = &msg.parsed.name;
            let full_name = format!("{}/msg/{}", package_name, name);
            let fn_name = format!("{}_py::serialize_{}",
                package_name.replace("-", "_"),
                name.to_lowercase()
            );
            let de_fn_name = format!("{}_py::deserialize_{}",
                package_name.replace("-", "_"),
                name.to_lowercase()
            );

            // Register in Python dict
            code.push_str(&format!(
                "        let msg_info = PyDict::new_bound(py);\n\
                 msg_info.set_item(\"serialize\", wrap_pyfunction!({}, m)?)?;\n\
                 msg_info.set_item(\"deserialize\", wrap_pyfunction!({}, m)?)?;\n\
                 msg_info.set_item(\"hash\", \"{}\")?;\n\
                 registry.set_item(\"{}\", msg_info)?;\n\n",
                fn_name,
                de_fn_name,
                msg.ros2_hash.to_hash_string(),
                full_name
            ));
        }

        code.push_str("    }\n\n");
    }

    // Expose registry and helper functions
    code.push_str(
        "    m.add(\"REGISTRY\", registry)?;\n\
         m.add_function(wrap_pyfunction!(serialize_message, m)?)?;\n\
         m.add_function(wrap_pyfunction!(deserialize_message, m)?)?;\n\
         Ok(())\n\
         }\n\n"
    );

    // Add convenience functions
    code.push_str(
        "#[allow(unsafe_op_in_unsafe_fn)]\n\
         #[pyfunction]\n\
         fn serialize_message(py: Python, type_name: &str, msg: &Bound<'_, PyAny>) -> PyResult<Vec<u8>> {\n\
         // Get registry from sys.modules (module is already loaded)\n\
         let sys = py.import_bound(\"sys\")?;\n\
         let modules = sys.getattr(\"modules\")?;\n\
         let ros_z_msgs = modules.get_item(\"ros_z_py.ros_z_msgs\")?;\n\
         let registry = ros_z_msgs.getattr(\"REGISTRY\")?;\n\
         \n\
         let msg_info = registry.get_item(type_name)\n\
         .map_err(|_| pyo3::exceptions::PyTypeError::new_err(\n\
         format!(\"Unknown message type: {}\", type_name)\n\
         ))?;\n\
         let serialize_fn = msg_info.get_item(\"serialize\")?;\n\
         serialize_fn.call1((msg,))?.extract()\n\
         }\n\n\
         #[allow(unsafe_op_in_unsafe_fn)]\n\
         #[pyfunction]\n\
         fn deserialize_message(py: Python, type_name: &str, bytes: &[u8]) -> PyResult<PyObject> {\n\
         // Get registry from sys.modules (module is already loaded)\n\
         let sys = py.import_bound(\"sys\")?;\n\
         let modules = sys.getattr(\"modules\")?;\n\
         let ros_z_msgs = modules.get_item(\"ros_z_py.ros_z_msgs\")?;\n\
         let registry = ros_z_msgs.getattr(\"REGISTRY\")?;\n\
         \n\
         let msg_info = registry.get_item(type_name)\n\
         .map_err(|_| pyo3::exceptions::PyTypeError::new_err(\n\
         format!(\"Unknown message type: {}\", type_name)\n\
         ))?;\n\
         let deserialize_fn = msg_info.get_item(\"deserialize\")?;\n\
         deserialize_fn.call1((bytes,)).map(|obj| obj.unbind())\n\
         }\n"
    );

    // Add Rust API wrappers for use from other Rust crates
    code.push_str(
        "\n// Rust API wrappers\n\
         /// Serialize a Python message to CDR bytes\n\
         pub fn serialize_to_cdr(type_name: &str, py: Python, msg: &Bound<'_, PyAny>) -> PyResult<Vec<u8>> {\n\
         serialize_message(py, type_name, msg)\n\
         }\n\n\
         /// Deserialize CDR bytes to a Python message\n\
         pub fn deserialize_from_cdr(type_name: &str, py: Python, bytes: &[u8]) -> PyResult<PyObject> {\n\
         deserialize_message(py, type_name, bytes)\n\
         }\n\n\
         /// Get the type hash for a message type\n\
         pub fn get_type_hash(type_name: &str) -> Result<String, String> {\n\
         Python::with_gil(|py| {\n\
         // Try to get ros_z_msgs from sys.modules first, then fall back to importing ros_z_py\n\
         let sys = py.import_bound(\"sys\")\n\
         .map_err(|e| format!(\"Failed to import sys: {}\", e))?;\n\
         let modules = sys.getattr(\"modules\")\n\
         .map_err(|e| format!(\"Failed to get sys.modules: {}\", e))?;\n\
         \n\
         let registry = if let Ok(ros_z_msgs) = modules.get_item(\"ros_z_py.ros_z_msgs\") {\n\
         ros_z_msgs.getattr(\"REGISTRY\")\n\
         .map_err(|e| format!(\"Failed to get REGISTRY: {}\", e))?\n\
         } else {\n\
         // Module not loaded yet, try importing ros_z_py first\n\
         let ros_z_py = py.import_bound(\"ros_z_py\")\n\
         .map_err(|e| format!(\"Failed to import ros_z_py: {}\", e))?;\n\
         let ros_z_msgs = ros_z_py.getattr(\"ros_z_msgs\")\n\
         .map_err(|e| format!(\"Failed to get ros_z_msgs from ros_z_py: {}\", e))?;\n\
         ros_z_msgs.getattr(\"REGISTRY\")\n\
         .map_err(|e| format!(\"Failed to get REGISTRY: {}\", e))?\n\
         };\n\
         \n\
         let info_obj = registry.get_item(type_name)\n\
         .map_err(|_| format!(\"Unknown message type: {}\", type_name))?;\n\
         let hash = info_obj.get_item(\"hash\")\n\
         .map_err(|_| \"Missing hash field\".to_string())?;\n\
         hash.extract::<String>()\n\
         .map_err(|e| format!(\"Failed to extract hash: {}\", e))\n\
         })\n\
         }\n\n\
         /// List all registered message types\n\
         pub fn list_registered_types() -> Vec<String> {\n\
         Python::with_gil(|py| {\n\
         // Try to get ros_z_msgs from sys.modules first, then fall back to importing ros_z_py\n\
         let sys = py.import_bound(\"sys\").ok()?;\n\
         let modules = sys.getattr(\"modules\").ok()?;\n\
         \n\
         let registry = if let Ok(ros_z_msgs) = modules.get_item(\"ros_z_py.ros_z_msgs\") {\n\
         ros_z_msgs.getattr(\"REGISTRY\").ok()\n\
         } else {\n\
         py.import_bound(\"ros_z_py\")\n\
         .and_then(|m| m.getattr(\"ros_z_msgs\"))\n\
         .and_then(|m| m.getattr(\"REGISTRY\"))\n\
         .ok()\n\
         }?;\n\
         \n\
         registry.call_method0(\"keys\")\n\
         .ok()\n\
         .and_then(|keys| keys.extract::<Vec<String>>().ok())\n\
         })\n\
         .unwrap_or_default()\n\
         }\n\n\
         /// Initialize the Python registry (called automatically on module load)\n\
         pub fn init_registry() {\n\
         // Registry is initialized when the module is imported\n\
         Python::with_gil(|py| {\n\
         let _ = py.import_bound(\"ros_z_py\");\n\
         });\n\
         }\n"
    );

    Ok(code)
}

fn generate_python_init(packages: &HashMap<String, Vec<&MessageFile>>) -> Result<String> {
    let mut code = String::from(
        "\"\"\"Auto-generated ROS 2 message types package.\"\"\"\n\
         \n\
         # Import all message types\n"
    );

    for package_name in packages.keys() {
        code.push_str(&format!("from . import {}\n", package_name));
    }

    code.push_str("\n__all__ = [\n");
    for package_name in packages.keys() {
        code.push_str(&format!("    \"{}\",\n", package_name));
    }
    code.push_str("]\n");

    Ok(code)
}
