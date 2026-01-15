//! Auto-generation of Python msgspec structs and PyO3 bindings for ROS 2 messages
//!
//! This module generates both Python msgspec structs and complete Rust PyO3 modules
//! from ROS message definitions, eliminating the need for manual registry code.

use anyhow::Result;
use roslibrust_codegen::MessageFile;
use std::collections::HashMap;
use std::fs;
use std::path::Path;

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
    // Convert field type to string and map to Python type
    let type_str = field_type.to_string();
    match type_str.as_str() {
        "bool" => Ok("bool".to_string()),
        "byte" | "int8" => Ok("int".to_string()),
        "char" | "uint8" => Ok("int".to_string()),
        "float32" => Ok("float".to_string()),
        "float64" => Ok("float".to_string()),
        "int16" => Ok("int".to_string()),
        "uint16" => Ok("int".to_string()),
        "int32" => Ok("int".to_string()),
        "uint32" => Ok("int".to_string()),
        "int64" => Ok("int".to_string()),
        "uint64" => Ok("int".to_string()),
        "string" => Ok("str".to_string()),
        "wstring" => Ok("str".to_string()), // Note: wstring not fully supported
        type_str if type_str.contains("Time") => Ok("dict".to_string()), // Time is a struct with sec/nanosec
        type_str if type_str.contains("Duration") => Ok("dict".to_string()), // Duration is a struct
        type_str if type_str.contains("/") => {
            // Nested type like "std_msgs/String"
            let parts: Vec<&str> = type_str.split("/").collect();
            if parts.len() == 3 && parts[1] == "msg" {
                Ok(format!("\"{}.{}\"", parts[0], parts[2])) // Forward reference to other msgspec structs
            } else {
                Err(anyhow::anyhow!("Unsupported nested type: {}", type_str))
            }
        }
        // Handle array types like "int32[]" or "geometry_msgs/Point[]"
        type_str if type_str.ends_with("[]") => {
            let element_type = &type_str[..type_str.len() - 2];
            match element_type {
                "bool" => Ok("list[bool]".to_string()),
                "byte" | "int8" => Ok("list[int]".to_string()),
                "char" | "uint8" => Ok("list[int]".to_string()),
                "float32" => Ok("list[float]".to_string()),
                "float64" => Ok("list[float]".to_string()),
                "int16" => Ok("list[int]".to_string()),
                "uint16" => Ok("list[int]".to_string()),
                "int32" => Ok("list[int]".to_string()),
                "uint32" => Ok("list[int]".to_string()),
                "int64" => Ok("list[int]".to_string()),
                "uint64" => Ok("list[int]".to_string()),
                "string" => Ok("list[str]".to_string()),
                nested if nested.contains("/") => {
                    // Nested array type
                    let parts: Vec<&str> = nested.split("/").collect();
                    if parts.len() == 3 && parts[1] == "msg" {
                        Ok(format!("list[\"{}.{}\"]", parts[0], parts[2]))
                     } else {
                         Err(anyhow::anyhow!("Unsupported array element type: {}", nested))
                     }
                 }
                 _ => {
                    // Fallback for unknown array element types
                    println!("cargo:warning=Unknown array element type '{}', treating as dict", element_type);
                    Ok("list[dict]".to_string())
                }
            }
        }
        // Fallback for other types - treat as dict (for nested messages without package prefix)
        _ => {
            // For unknown types, treat as dict
            println!("cargo:warning=Unknown field type '{}', treating as dict", type_str);
            Ok("dict".to_string())
        }
    }
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

fn generate_rust_field_extraction(
    field: &roslibrust_codegen::FieldInfo,
    all_messages: &HashMap<String, &MessageFile>,
) -> Result<String> {
    let field_name = &field.field_name;
    let escaped_name = escape_rust_keyword(field_name);

    let is_array = !matches!(
        field.field_type.array_info,
        roslibrust_codegen::ArrayType::NotArray
    );

    let base_type = &field.field_type.field_type;

    if is_array {
        // Check if it's a fixed-size array
        let is_fixed_size = matches!(
            field.field_type.array_info,
            roslibrust_codegen::ArrayType::FixedLength(_)
        );

        if is_fixed_size {
            // Fixed-size arrays need conversion from Vec
            match base_type.as_str() {
                "bool" => Ok(format!(
                    "            {}: {{\n\
                     let vec: Vec<bool> = extract_field!(dict, \"{}\", Vec<bool>);\n\
                     vec.try_into().map_err(|_| pyo3::exceptions::PyValueError::new_err(\n\
                     \"Array size mismatch for field '{}'\"))?\n\
                     }},\n",
                    escaped_name, field_name, field_name
                )),
                "byte" | "int8" => Ok(format!(
                    "            {}: {{\n\
                     let vec: Vec<i8> = extract_field!(dict, \"{}\", Vec<i8>);\n\
                     vec.try_into().map_err(|_| pyo3::exceptions::PyValueError::new_err(\n\
                     \"Array size mismatch for field '{}'\"))?\n\
                     }},\n",
                    escaped_name, field_name, field_name
                )),
                "char" | "uint8" => Ok(format!(
                    "            {}: {{\n\
                     let vec: Vec<u8> = extract_field!(dict, \"{}\", Vec<u8>);\n\
                     vec.try_into().map_err(|_| pyo3::exceptions::PyValueError::new_err(\n\
                     \"Array size mismatch for field '{}'\"))?\n\
                     }},\n",
                    escaped_name, field_name, field_name
                )),
                "int16" => Ok(format!(
                    "            {}: {{\n\
                     let vec: Vec<i16> = extract_field!(dict, \"{}\", Vec<i16>);\n\
                     vec.try_into().map_err(|_| pyo3::exceptions::PyValueError::new_err(\n\
                     \"Array size mismatch for field '{}'\"))?\n\
                     }},\n",
                    escaped_name, field_name, field_name
                )),
                "uint16" => Ok(format!(
                    "            {}: {{\n\
                     let vec: Vec<u16> = extract_field!(dict, \"{}\", Vec<u16>);\n\
                     vec.try_into().map_err(|_| pyo3::exceptions::PyValueError::new_err(\n\
                     \"Array size mismatch for field '{}'\"))?\n\
                     }},\n",
                    escaped_name, field_name, field_name
                )),
                "int32" => Ok(format!(
                    "            {}: {{\n\
                     let vec: Vec<i32> = extract_field!(dict, \"{}\", Vec<i32>);\n\
                     vec.try_into().map_err(|_| pyo3::exceptions::PyValueError::new_err(\n\
                     \"Array size mismatch for field '{}'\"))?\n\
                     }},\n",
                    escaped_name, field_name, field_name
                )),
                "uint32" => Ok(format!(
                    "            {}: {{\n\
                     let vec: Vec<u32> = extract_field!(dict, \"{}\", Vec<u32>);\n\
                     vec.try_into().map_err(|_| pyo3::exceptions::PyValueError::new_err(\n\
                     \"Array size mismatch for field '{}'\"))?\n\
                     }},\n",
                    escaped_name, field_name, field_name
                )),
                "int64" => Ok(format!(
                    "            {}: {{\n\
                     let vec: Vec<i64> = extract_field!(dict, \"{}\", Vec<i64>);\n\
                     vec.try_into().map_err(|_| pyo3::exceptions::PyValueError::new_err(\n\
                     \"Array size mismatch for field '{}'\"))?\n\
                     }},\n",
                    escaped_name, field_name, field_name
                )),
                "uint64" => Ok(format!(
                    "            {}: {{\n\
                     let vec: Vec<u64> = extract_field!(dict, \"{}\", Vec<u64>);\n\
                     vec.try_into().map_err(|_| pyo3::exceptions::PyValueError::new_err(\n\
                     \"Array size mismatch for field '{}'\"))?\n\
                     }},\n",
                    escaped_name, field_name, field_name
                )),
                "float32" => Ok(format!(
                    "            {}: {{\n\
                     let vec: Vec<f32> = extract_field!(dict, \"{}\", Vec<f32>);\n\
                     vec.try_into().map_err(|_| pyo3::exceptions::PyValueError::new_err(\n\
                     \"Array size mismatch for field '{}'\"))?\n\
                     }},\n",
                    escaped_name, field_name, field_name
                )),
                "float64" => Ok(format!(
                    "            {}: {{\n\
                     let vec: Vec<f64> = extract_field!(dict, \"{}\", Vec<f64>);\n\
                     vec.try_into().map_err(|_| pyo3::exceptions::PyValueError::new_err(\n\
                     \"Array size mismatch for field '{}'\"))?\n\
                     }},\n",
                    escaped_name, field_name, field_name
                )),
                "string" => Ok(format!(
                    "            {}: {{\n\
                     let vec: Vec<String> = extract_field!(dict, \"{}\", Vec<String>);\n\
                     vec.try_into().map_err(|_| pyo3::exceptions::PyValueError::new_err(\n\
                     \"Array size mismatch for field '{}'\"))?\n\
                     }},\n",
                    escaped_name, field_name, field_name
                )),
                _ => {
                    // Fixed-size array of nested messages
                    let nested_msg_key = if let Some(ref package_name) = field.field_type.package_name {
                        format!("{}/{}", package_name, base_type)
                    } else {
                        format!("{}/{}", field.field_type.source_package, base_type)
                    };

                    let nested_msg = all_messages.get(&nested_msg_key)
                        .ok_or_else(|| anyhow::anyhow!("Message {} not found", nested_msg_key))?;

                    let nested_fields = generate_nested_fields(&nested_msg.parsed.fields, all_messages)?;

                    let package = if let Some(ref package_name) = field.field_type.package_name {
                        package_name.clone()
                    } else {
                        field.field_type.source_package.clone()
                    };

                    Ok(format!(
                        "            {}: {{\n\
                         let py_list = dict.get_item(\"{}\")?\n\
                         .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err(\"Missing '{}'\"))?;\n\
                         let mut vec = Vec::new();\n\
                         for item in py_list.iter()? {{\n\
                         let item_dict = item?.downcast::<PyDict>()?;\n\
                         vec.push({}::{} {{\n\
                         {}\
                         }});\n\
                         }}\n\
                         vec.try_into().map_err(|_| pyo3::exceptions::PyValueError::new_err(\n\
                         \"Array size mismatch for field '{}'\"))?\n\
                         }},\n",
                        escaped_name,
                        field_name,
                        field_name,
                        package,
                        base_type,
                        nested_fields,
                        field_name
                    ))
                }
            }
        } else {
            // Dynamic-size arrays (Vec) - keep existing logic
            match base_type.as_str() {
                "bool" => Ok(format!(
                    "            {}: extract_field!(dict, \"{}\", Vec<bool>),\n",
                    escaped_name, field_name
                )),
                "byte" | "int8" => Ok(format!(
                    "            {}: extract_field!(dict, \"{}\", Vec<i8>),\n",
                    escaped_name, field_name
                )),
                "char" | "uint8" => Ok(format!(
                    "            {}: extract_field!(dict, \"{}\", Vec<u8>),\n",
                    escaped_name, field_name
                )),
                "int16" => Ok(format!(
                    "            {}: extract_field!(dict, \"{}\", Vec<i16>),\n",
                    escaped_name, field_name
                )),
                "uint16" => Ok(format!(
                    "            {}: extract_field!(dict, \"{}\", Vec<u16>),\n",
                    escaped_name, field_name
                )),
                "int32" => Ok(format!(
                    "            {}: extract_field!(dict, \"{}\", Vec<i32>),\n",
                    escaped_name, field_name
                )),
                "uint32" => Ok(format!(
                    "            {}: extract_field!(dict, \"{}\", Vec<u32>),\n",
                    escaped_name, field_name
                )),
                "int64" => Ok(format!(
                    "            {}: extract_field!(dict, \"{}\", Vec<i64>),\n",
                    escaped_name, field_name
                )),
                "uint64" => Ok(format!(
                    "            {}: extract_field!(dict, \"{}\", Vec<u64>),\n",
                    escaped_name, field_name
                )),
                "float32" => Ok(format!(
                    "            {}: extract_field!(dict, \"{}\", Vec<f32>),\n",
                    escaped_name, field_name
                )),
                "float64" => Ok(format!(
                    "            {}: extract_field!(dict, \"{}\", Vec<f64>),\n",
                    escaped_name, field_name
                )),
                "string" => Ok(format!(
                    "            {}: extract_field!(dict, \"{}\", Vec<String>),\n",
                    escaped_name, field_name
                )),
                _ => {
                    let nested_msg_key = if let Some(ref package_name) = field.field_type.package_name {
                        format!("{}/{}", package_name, base_type)
                    } else {
                        format!("{}/{}", field.field_type.source_package, base_type)
                    };

                    let nested_msg = all_messages.get(&nested_msg_key)
                        .ok_or_else(|| anyhow::anyhow!("Message {} not found", nested_msg_key))?;

                    let nested_fields = generate_nested_fields(&nested_msg.parsed.fields, all_messages)?;

                    let package = if let Some(ref package_name) = field.field_type.package_name {
                        package_name.clone()
                    } else {
                        field.field_type.source_package.clone()
                    };

                    Ok(format!(
                        "            {}: {{\n\
                         let py_list = dict.get_item(\"{}\")?\n\
                         .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err(\"Missing '{}'\"))?;\n\
                         let mut vec = Vec::new();\n\
                         for item in py_list.iter()? {{\n\
                         let item_dict = item?.downcast::<PyDict>()?;\n\
                         vec.push({}::{} {{\n\
                         {}\
                         }});\n\
                         }}\n\
                         vec\n\
                         }},\n",
                        escaped_name,
                        field_name,
                        field_name,
                        package,
                        base_type,
                        nested_fields
                    ))
                }
            }
        }
    } else {
        match base_type.as_str() {
            "bool" => Ok(format!(
                "            {}: extract_field!(dict, \"{}\", bool),\n",
                escaped_name, field_name
            )),
            "byte" | "int8" => Ok(format!(
                "            {}: extract_field!(dict, \"{}\", i8),\n",
                escaped_name, field_name
            )),
            "char" | "uint8" => Ok(format!(
                "            {}: extract_field!(dict, \"{}\", u8),\n",
                escaped_name, field_name
            )),
            "int16" => Ok(format!(
                "            {}: extract_field!(dict, \"{}\", i16),\n",
                escaped_name, field_name
            )),
            "uint16" => Ok(format!(
                "            {}: extract_field!(dict, \"{}\", u16),\n",
                escaped_name, field_name
            )),
            "int32" => Ok(format!(
                "            {}: extract_field!(dict, \"{}\", i32),\n",
                escaped_name, field_name
            )),
            "uint32" => Ok(format!(
                "            {}: extract_field!(dict, \"{}\", u32),\n",
                escaped_name, field_name
            )),
            "int64" => Ok(format!(
                "            {}: extract_field!(dict, \"{}\", i64),\n",
                escaped_name, field_name
            )),
            "uint64" => Ok(format!(
                "            {}: extract_field!(dict, \"{}\", u64),\n",
                escaped_name, field_name
            )),
            "float32" => Ok(format!(
                "            {}: extract_field!(dict, \"{}\", f32),\n",
                escaped_name, field_name
            )),
            "float64" => Ok(format!(
                "            {}: extract_field!(dict, \"{}\", f64),\n",
                escaped_name, field_name
            )),
            "string" => Ok(format!(
                "            {}: extract_field!(dict, \"{}\", String),\n",
                escaped_name, field_name
            )),
            _ => {
                let nested_msg_key = if let Some(ref package_name) = field.field_type.package_name {
                    format!("{}/{}", package_name, base_type)
                } else {
                    format!("{}/{}", field.field_type.source_package, base_type)
                };

                let nested_msg = all_messages.get(&nested_msg_key)
                    .ok_or_else(|| anyhow::anyhow!("Message {} not found", nested_msg_key))?;

                let nested_fields = generate_nested_fields(&nested_msg.parsed.fields, all_messages)?;

                let package = if let Some(ref package_name) = field.field_type.package_name {
                    package_name.clone()
                } else {
                    field.field_type.source_package.clone()
                };

                Ok(format!(
                    "            {}: {{\n\
                     let nested_dict = dict.get_item(\"{}\")?\n\
                     .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err(\"Missing '{}'\"))?\n\
                     .downcast::<PyDict>()?;\n\
                     {}::{} {{\n\
                     {}\
                     }}\n\
                     }},\n",
                    escaped_name,
                    field_name,
                    field_name,
                    package,
                    base_type,
                    nested_fields
                ))
            }
        }
    }
}

// Helper function to generate nested field extractions
fn generate_nested_fields(
    fields: &[roslibrust_codegen::FieldInfo],
    all_messages: &HashMap<String, &MessageFile>,
) -> Result<String> {
    let mut code = String::new();
    for field in fields {
        code.push_str(&generate_rust_field_extraction(field, all_messages)?);
    }
    Ok(code)
}

// Helper to generate nested field constructions with proper field access
fn generate_python_nested_field_constructions(
    fields: &[roslibrust_codegen::FieldInfo],
    _all_messages: &HashMap<String, &MessageFile>,
    _current_package: &str,
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

        // Check if it's a fixed-size array
        let is_fixed_size = matches!(
            field.field_type.array_info,
            roslibrust_codegen::ArrayType::FixedLength(_)
        );

        // For nested constructions, reference item.<field> instead of rust_msg.<field>
        match base_type.as_str() {
            "bool" | "byte" | "char" | "int8" | "uint8" | "int16" | "uint16" |
            "int32" | "uint32" | "int64" | "uint64" | "float32" | "float64" | "string" => {
                if is_array {
                    // For arrays (both fixed and dynamic), convert to Python list
                    if is_fixed_size {
                        code.push_str(&format!(
                            "             nested_kwargs.set_item(\"{}\", item.{}.to_vec())?;\n",
                            field_name, escaped_name
                        ));
                    } else {
                        code.push_str(&format!(
                            "             nested_kwargs.set_item(\"{}\", &item.{})?;\n",
                            field_name, escaped_name
                        ));
                    }
                } else {
                    code.push_str(&format!(
                        "             nested_kwargs.set_item(\"{}\", item.{})?;\n",
                        field_name, escaped_name
                    ));
                }
            },
            _ => {
                // For deeply nested messages, add TODO comment
                code.push_str(&format!(
                    "             // TODO: Handle deeply nested field {}\n\
                     nested_kwargs.set_item(\"{}\", PyDict::new_bound(py))?;\n",
                    base_type, field_name
                ));
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

                let nested_constructions = generate_python_nested_field_constructions(
                    &nested_msg.parsed.fields,
                    all_messages,
                    &package
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

                let nested_constructions = generate_python_nested_field_constructions(
                    &nested_msg.parsed.fields,
                    all_messages,
                    &package
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
         use cdr::{CdrLe, Infinite};\n\
         use crate::ros::*;\n\n"
    );

    // Generate helper macro for extracting fields from dict
    code.push_str(
        "macro_rules! extract_field {\n\
          ($dict:expr, $field:expr, $ty:ty) => {\n\
          {\n\
          let item = $dict.get_item($field)?;\n\
          let item = item.ok_or_else(|| pyo3::exceptions::PyKeyError::new_err(\n\
          format!(\"Missing field '{}'\", $field)))?;\n\
          item.extract::<$ty>()?\n\
          }\n\
          };\n\
          }\n\n"
    );

    // Generate serialize/deserialize functions for each message
    for (package_name, package_msgs) in packages {
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

/// Generate serialize/deserialize functions for a single message
fn generate_message_functions(
    msg: &MessageFile,
    all_messages: &HashMap<String, &MessageFile>,
) -> Result<String> {
    let package = &msg.parsed.package;
    let name = &msg.parsed.name;

    let mut code = format!(
        "    #[pyfunction]\n\
         pub fn serialize_{}(msg: &Bound<'_, PyAny>) -> PyResult<Vec<u8>> {{\n",
        name.to_lowercase()
    );

    // Check if it's a msgspec struct
    code.push_str(
        "        if !msg.hasattr(\"__msgtype__\")? {\n\
         return Err(pyo3::exceptions::PyTypeError::new_err(\n\
         \"Expected msgspec.Struct message\"\n\
         ));\n\
         }\n\n"
    );

    // Convert to dict via msgspec.structs.asdict (zero-copy)
    code.push_str(
        "        let msgspec_mod = msg.py().import_bound(\"msgspec.structs\")?;\n\
          let asdict = msgspec_mod.getattr(\"asdict\")?;\n\
          let dict = asdict.call1((msg,))?.downcast::<PyDict>()?;\n\n"
    );

    // Manually construct Rust struct from dict
    code.push_str(&format!("        let rust_msg = {}::{} {{\n", package, name));
    for field in &msg.parsed.fields {
        code.push_str(&generate_rust_field_extraction(field, all_messages)?);
    }
    code.push_str("        };\n\n");

    // Serialize
    code.push_str(
        "        cdr::serialize::<_, _, CdrLe>(&rust_msg, Infinite)\n\
         .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))\n\
         }\n\n"
    );

    // Generate deserialize function
    code.push_str(&format!(
        "    #[pyfunction]\n\
         pub fn deserialize_{}(py: Python, bytes: &[u8]) -> PyResult<PyObject> {{\n",
        name.to_lowercase()
    ));

    code.push_str(&format!(
        "        let rust_msg: {}::{} = cdr::deserialize(bytes)\n\
         .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;\n\n",
        package, name
    ));

    // Import Python class
    code.push_str(&format!(
        "        let types = py.import_bound(\"ros_z_python.types.{}\")?;\n\
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
        "#[pymodule]\n\
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
        "#[pyfunction]\n\
         fn serialize_message(py: Python, type_name: &str, msg: &Bound<'_, PyAny>) -> PyResult<Vec<u8>> {\n\
         let registry = py.import_bound(\"ros_z_msgs\")?.getattr(\"REGISTRY\")?;\n\
         let msg_info = registry.get_item(type_name)?\n\
         .ok_or_else(|| pyo3::exceptions::PyTypeError::new_err(\n\
         format!(\"Unknown message type: {}\", type_name)\n\
         ))?;\n\
         let serialize_fn = msg_info.get_item(\"serialize\")?.unwrap();\n\
         serialize_fn.call1((msg,))?.extract()\n\
         }\n\n\
         #[pyfunction]\n\
         fn deserialize_message(py: Python, type_name: &str, bytes: &[u8]) -> PyResult<PyObject> {\n\
         let registry = py.import_bound(\"ros_z_msgs\")?.getattr(\"REGISTRY\")?;\n\
         let msg_info = registry.get_item(type_name)?\n\
         .ok_or_else(|| pyo3::exceptions::PyTypeError::new_err(\n\
         format!(\"Unknown message type: {}\", type_name)\n\
         ))?;\n\
         let deserialize_fn = msg_info.get_item(\"deserialize\")?.unwrap();\n\
         deserialize_fn.call1((bytes,))\n\
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
