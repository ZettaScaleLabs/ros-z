//! RIHS01 Type Hash Calculation for ROS2
//!
//! This module implements the ROS2 RIHS01 (ROS IDL Hash Standard 01) type hashing algorithm.
//!
use std::collections::BTreeMap;

use anyhow::{Context, Result, bail};
use serde::Serialize;
use sha2::Digest;

use crate::types::{ArrayType, FieldType, ParsedMessage, TypeHash};

/// ROS2 TypeDescription message structure for hashing
#[derive(Serialize, serde::Deserialize)]
pub struct TypeDescriptionMsg {
    pub type_description: TypeDescription,
    pub referenced_type_descriptions: Vec<TypeDescription>,
}

/// Type description for a single message type
#[derive(Serialize, serde::Deserialize, Clone)]
pub struct TypeDescription {
    pub type_name: String,
    pub fields: Vec<FieldDescription>,
}

/// Field description in a type
#[derive(Serialize, serde::Deserialize, Clone)]
pub struct FieldDescription {
    pub name: String,
    #[serde(rename = "type")]
    pub field_type: FieldTypeDescription,
    /// default_value from .msg files (for RIHS01 hash, always use empty string)
    #[serde(default)]
    pub default_value: String,
}

/// Field description for hash computation (excludes default_value)
#[derive(Serialize)]
pub struct FieldDescriptionForHash<'a> {
    name: &'a str,
    #[serde(rename = "type")]
    field_type: &'a FieldTypeDescription,
}

/// Type description for hash computation
#[derive(Serialize)]
pub struct TypeDescriptionForHash<'a> {
    type_name: &'a str,
    fields: Vec<FieldDescriptionForHash<'a>>,
}

/// TypeDescriptionMsg for hash computation
#[derive(Serialize)]
pub struct TypeDescriptionMsgForHash<'a> {
    type_description: TypeDescriptionForHash<'a>,
    referenced_type_descriptions: Vec<TypeDescriptionForHash<'a>>,
}

/// Field type description with type ID
#[derive(Serialize, serde::Deserialize, Clone)]
pub struct FieldTypeDescription {
    pub type_id: u8,
    pub capacity: u64,
    pub string_capacity: u64,
    pub nested_type_name: String,
}

/// Convert TypeDescriptionMsg to hash-computation version (excludes default_value)
pub fn to_hash_version(msg: &TypeDescriptionMsg) -> TypeDescriptionMsgForHash<'_> {
    TypeDescriptionMsgForHash {
        type_description: TypeDescriptionForHash {
            type_name: &msg.type_description.type_name,
            fields: msg.type_description.fields.iter().map(|f| FieldDescriptionForHash {
                name: &f.name,
                field_type: &f.field_type,
            }).collect(),
        },
        referenced_type_descriptions: msg.referenced_type_descriptions.iter().map(|td| TypeDescriptionForHash {
            type_name: &td.type_name,
            fields: td.fields.iter().map(|f| FieldDescriptionForHash {
                name: &f.name,
                field_type: &f.field_type,
            }).collect(),
        }).collect(),
    }
}

/// Calculate RIHS01 hash for a message
pub fn calculate_type_hash(
    msg: &ParsedMessage,
    resolved_deps: &BTreeMap<String, TypeDescription>,
) -> Result<TypeHash> {
    let type_desc_msg = build_type_description_msg(msg, resolved_deps)?;

    // NOTE: ROS2 uses JSON format (libyaml flow style with quoted scalars = JSON)
    // Exclude default_value from hash computation (ROS2's emit_field() doesn't include it)
    let hash_version = to_hash_version(&type_desc_msg);
    let json = to_ros2_json(&hash_version)?;

    let mut hasher = sha2::Sha256::new();
    hasher.update(json.as_bytes());
    let hash_bytes: [u8; 32] = hasher.finalize().into();

    Ok(TypeHash(hash_bytes))
}

/// Calculate RIHS01 hash for a service (Request + Response + Event)
/// Based on ROS2 RIHS01 specification
pub fn calculate_service_type_hash(
    package: &str,
    service_name: &str,
    request_desc: &TypeDescription,
    response_desc: &TypeDescription,
    service_event_info_desc: &TypeDescription,
    resolved_deps: &BTreeMap<String, TypeDescription>,
) -> Result<TypeHash> {
    // ROS2 uses slash format, not :: format
    // Detect if this is an action service (contains SendGoal/GetResult/CancelGoal)
    let is_action = service_name.contains("SendGoal")
        || service_name.contains("GetResult")
        || service_name.contains("CancelGoal");
    let path = if is_action { "action" } else { "srv" };

    // Action services use /action/ path, regular services use /srv/ path
    let service_type_name = format!("{}/{}/{}", package, path, service_name);
    let request_type_name = format!("{}/{}/{}_Request", package, path, service_name);
    let response_type_name = format!("{}/{}/{}_Response", package, path, service_name);
    let event_type_name = format!("{}/{}/{}_Event", package, path, service_name);

    // Create the Event type with three fields: info, request[], response[]
    let event_desc = TypeDescription {
        type_name: event_type_name.clone(),
        fields: vec![
            FieldDescription {
                name: "info".to_string(),
                field_type: FieldTypeDescription {
                    type_id: 1, // FIELD_TYPE_NESTED_TYPE
                    capacity: 0,
                    string_capacity: 0,
                    nested_type_name: service_event_info_desc.type_name.clone(),
                },
                default_value: String::new(),
            },
            FieldDescription {
                name: "request".to_string(),
                field_type: FieldTypeDescription {
                    type_id: 97, // FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE
                    capacity: 1,
                    string_capacity: 0,
                    nested_type_name: request_type_name.clone(),
                },
                default_value: String::new(),
            },
            FieldDescription {
                name: "response".to_string(),
                field_type: FieldTypeDescription {
                    type_id: 97, // FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE
                    capacity: 1,
                    string_capacity: 0,
                    nested_type_name: response_type_name.clone(),
                },
                default_value: String::new(),
            },
        ],
    };

    // Main service type description with three fields in definition order (NOT alphabetical)
    let service_desc = TypeDescription {
        type_name: service_type_name,
        fields: vec![
            FieldDescription {
                name: "request_message".to_string(),
                field_type: FieldTypeDescription {
                    type_id: 1, // FIELD_TYPE_NESTED_TYPE
                    capacity: 0,
                    string_capacity: 0,
                    nested_type_name: request_type_name.clone(),
                },
                default_value: String::new(),
            },
            FieldDescription {
                name: "response_message".to_string(),
                field_type: FieldTypeDescription {
                    type_id: 1, // FIELD_TYPE_NESTED_TYPE
                    capacity: 0,
                    string_capacity: 0,
                    nested_type_name: response_type_name.clone(),
                },
                default_value: String::new(),
            },
            FieldDescription {
                name: "event_message".to_string(),
                field_type: FieldTypeDescription {
                    type_id: 1, // FIELD_TYPE_NESTED_TYPE
                    capacity: 0,
                    string_capacity: 0,
                    nested_type_name: event_type_name.clone(),
                },
                default_value: String::new(),
            },
        ],
    };

    // Build referenced type descriptions
    // Need to include: Request, Response, Event, ServiceEventInfo, and any deps of ServiceEventInfo
    let mut request_desc_corrected = request_desc.clone();
    request_desc_corrected.type_name = request_type_name;

    let mut response_desc_corrected = response_desc.clone();
    response_desc_corrected.type_name = response_type_name;

    let mut referenced = BTreeMap::new();

    // Add ServiceEventInfo and its dependencies (like Time)
    for desc in resolved_deps.values() {
        referenced.insert(desc.type_name.clone(), desc.clone());
    }

    // Add the three service-specific types
    referenced.insert(event_desc.type_name.clone(), event_desc);
    referenced.insert(
        request_desc_corrected.type_name.clone(),
        request_desc_corrected,
    );
    referenced.insert(
        response_desc_corrected.type_name.clone(),
        response_desc_corrected,
    );

    // Sort referenced types alphabetically by type_name
    let mut referenced_vec: Vec<TypeDescription> = referenced.into_values().collect();
    referenced_vec.sort_by(|a, b| a.type_name.cmp(&b.type_name));

    let type_desc_msg = TypeDescriptionMsg {
        type_description: service_desc,
        referenced_type_descriptions: referenced_vec,
    };

    // NOTE: ROS2 uses JSON format (libyaml flow style with quoted scalars = JSON)
    // Exclude default_value from hash computation (ROS2's emit_field() doesn't include it)
    let hash_version = to_hash_version(&type_desc_msg);
    let json = to_ros2_json(&hash_version)?;

    let mut hasher = sha2::Sha256::new();
    hasher.update(json.as_bytes());
    let hash_bytes: [u8; 32] = hasher.finalize().into();

    Ok(TypeHash(hash_bytes))
}

/// Build TypeDescription message from parsed message
fn build_type_description_msg(
    msg: &ParsedMessage,
    resolved_deps: &BTreeMap<String, TypeDescription>,
) -> Result<TypeDescriptionMsg> {
    let type_name = format!("{}/msg/{}", msg.package, msg.name);

    let mut fields = Vec::new();
    let mut referenced_types = BTreeMap::new();

    // ROS2 treats empty messages as having a single uint8 field
    if msg.fields.is_empty() {
        fields.push(FieldDescription {
            name: "structure_needs_at_least_one_member".to_string(),
            field_type: FieldTypeDescription {
                type_id: 3, // FIELD_TYPE_UINT8
                capacity: 0,
                string_capacity: 0,
                nested_type_name: String::new(),
            },
            default_value: String::new(),
        });
    } else {
        for field in &msg.fields {
            let type_id = get_type_id(&field.field_type)?;
            let capacity = get_array_capacity(&field.field_type);

            // For nested types, get the fully qualified name with package context
            let nested_type_name = if is_primitive_type(&field.field_type.base_type) {
                String::new()
            } else {
                get_nested_type_name_with_context(&field.field_type, &msg.package)
            };

            fields.push(FieldDescription {
                name: field.name.clone(),
                field_type: FieldTypeDescription {
                    type_id,
                    capacity,
                    string_capacity: 0, // TODO: bounded strings
                    nested_type_name,
                },
                default_value: String::new(),
            });

            // Collect nested type references
            if !is_primitive_type(&field.field_type.base_type) {
                // Determine the package - if None, it's the same package as the message
                let pkg = field.field_type.package.as_deref().unwrap_or(&msg.package);

                let dep_key = format!("{}/{}", pkg, field.field_type.base_type);

                if let Some(dep_desc) = resolved_deps.get(&dep_key) {
                    collect_referenced_types(dep_desc, resolved_deps, &mut referenced_types);
                    referenced_types.insert(dep_desc.type_name.clone(), dep_desc.clone());
                }
            }
        }
    }

    Ok(TypeDescriptionMsg {
        type_description: TypeDescription { type_name, fields },
        referenced_type_descriptions: referenced_types.into_values().collect(),
    })
}

/// Recursively collect all referenced types
fn collect_referenced_types(
    type_desc: &TypeDescription,
    all_deps: &BTreeMap<String, TypeDescription>,
    collected: &mut BTreeMap<String, TypeDescription>,
) {
    for field in &type_desc.fields {
        if !field.field_type.nested_type_name.is_empty() {
            // Parse the nested type name to get package/type
            if let Some(dep) = all_deps.get(&field.field_type.nested_type_name)
                && !collected.contains_key(&dep.type_name)
            {
                collect_referenced_types(dep, all_deps, collected);
                collected.insert(dep.type_name.clone(), dep.clone());
            }
        }
    }
}

/// Get ROS2 type ID for a field type (1-166)
pub fn get_type_id(field_type: &FieldType) -> Result<u8> {
    let base_type = field_type.base_type.as_str();
    let array = &field_type.array;
    let has_package = field_type.package.is_some();

    // Nested types (either explicitly packaged OR not a primitive)
    // package=None means same-package reference for custom types
    if has_package || !is_primitive_type(base_type) {
        return Ok(match array {
            ArrayType::Single => 1,      // NESTED_TYPE
            ArrayType::Fixed(_) => 49,   // NESTED_TYPE_ARRAY
            ArrayType::Bounded(_) => 97, // NESTED_TYPE_BOUNDED_SEQUENCE
            ArrayType::Unbounded => 145, // NESTED_TYPE_UNBOUNDED_SEQUENCE
        });
    }

    // Primitive types
    // Note: In ROS2, 'char' is uint8 (unsigned), not int8 (signed)
    match (base_type, array) {
        // Single primitives
        ("int8", ArrayType::Single) => Ok(2),
        ("uint8" | "byte" | "char", ArrayType::Single) => Ok(3),
        ("int16", ArrayType::Single) => Ok(4),
        ("uint16", ArrayType::Single) => Ok(5),
        ("int32", ArrayType::Single) => Ok(6),
        ("uint32", ArrayType::Single) => Ok(7),
        ("int64", ArrayType::Single) => Ok(8),
        ("uint64", ArrayType::Single) => Ok(9),
        ("float32", ArrayType::Single) => Ok(10),
        ("float64", ArrayType::Single) => Ok(11),
        ("bool", ArrayType::Single) => Ok(15),
        ("string", ArrayType::Single) => Ok(17),

        // Fixed arrays
        ("int8", ArrayType::Fixed(_)) => Ok(50),
        ("uint8" | "byte" | "char", ArrayType::Fixed(_)) => Ok(51),
        ("int16", ArrayType::Fixed(_)) => Ok(52),
        ("uint16", ArrayType::Fixed(_)) => Ok(53),
        ("int32", ArrayType::Fixed(_)) => Ok(54),
        ("uint32", ArrayType::Fixed(_)) => Ok(55),
        ("int64", ArrayType::Fixed(_)) => Ok(56),
        ("uint64", ArrayType::Fixed(_)) => Ok(57),
        ("float32", ArrayType::Fixed(_)) => Ok(58),
        ("float64", ArrayType::Fixed(_)) => Ok(59),
        ("bool", ArrayType::Fixed(_)) => Ok(63),
        ("string", ArrayType::Fixed(_)) => Ok(65),

        // Bounded sequences
        ("int8", ArrayType::Bounded(_)) => Ok(98),
        ("uint8" | "byte" | "char", ArrayType::Bounded(_)) => Ok(99),
        ("int16", ArrayType::Bounded(_)) => Ok(100),
        ("uint16", ArrayType::Bounded(_)) => Ok(101),
        ("int32", ArrayType::Bounded(_)) => Ok(102),
        ("uint32", ArrayType::Bounded(_)) => Ok(103),
        ("int64", ArrayType::Bounded(_)) => Ok(104),
        ("uint64", ArrayType::Bounded(_)) => Ok(105),
        ("float32", ArrayType::Bounded(_)) => Ok(106),
        ("float64", ArrayType::Bounded(_)) => Ok(107),
        ("bool", ArrayType::Bounded(_)) => Ok(111),
        ("string", ArrayType::Bounded(_)) => Ok(113),

        // Unbounded sequences
        ("int8", ArrayType::Unbounded) => Ok(146),
        ("uint8" | "byte" | "char", ArrayType::Unbounded) => Ok(147),
        ("int16", ArrayType::Unbounded) => Ok(148),
        ("uint16", ArrayType::Unbounded) => Ok(149),
        ("int32", ArrayType::Unbounded) => Ok(150),
        ("uint32", ArrayType::Unbounded) => Ok(151),
        ("int64", ArrayType::Unbounded) => Ok(152),
        ("uint64", ArrayType::Unbounded) => Ok(153),
        ("float32", ArrayType::Unbounded) => Ok(154),
        ("float64", ArrayType::Unbounded) => Ok(155),
        ("bool", ArrayType::Unbounded) => Ok(159),
        ("string", ArrayType::Unbounded) => Ok(161),

        _ => bail!(
            "Unsupported field type: {} with array {:?}",
            base_type,
            array
        ),
    }
}

/// Get array capacity for field type
fn get_array_capacity(field_type: &FieldType) -> u64 {
    match &field_type.array {
        ArrayType::Fixed(n) | ArrayType::Bounded(n) => *n as u64,
        _ => 0,
    }
}

/// Get nested type name in ROS2 format
/// Note: For same-package references (package=None), this will return empty string.
/// The caller must provide the source package context.
#[allow(dead_code)]
fn get_nested_type_name(field_type: &FieldType) -> String {
    if let Some(ref pkg) = field_type.package {
        format!("{}::msg::dds_::{}_", pkg, field_type.base_type)
    } else {
        String::new()
    }
}

/// Get nested type name with package context for same-package references
fn get_nested_type_name_with_context(field_type: &FieldType, source_package: &str) -> String {
    let pkg = field_type.package.as_deref().unwrap_or(source_package);
    format!("{}/msg/{}", pkg, field_type.base_type)
}

/// Check if a base type name is a ROS primitive
pub fn is_primitive_type(base_type: &str) -> bool {
    matches!(
        base_type,
        "bool"
            | "byte"
            | "char"
            | "uint8"
            | "int8"
            | "uint16"
            | "int16"
            | "uint32"
            | "int32"
            | "uint64"
            | "int64"
            | "float32"
            | "float64"
            | "string"
    )
}

/// Serialize to ROS2-compatible JSON format
/// ROS2 uses JSON with spaces after colons and commas: `{"key": "value", "key2": 2}`
pub fn to_ros2_json<T: Serialize>(value: &T) -> Result<String> {
    // Serialize to compact JSON
    let compact = serde_json::to_string(value).context("Failed to serialize to JSON")?;

    // Add spaces after : and , (matching Python's default json.dumps())
    let mut result = String::with_capacity(compact.len() + 100);
    let chars: Vec<char> = compact.chars().collect();

    for i in 0..chars.len() {
        result.push(chars[i]);

        // Add space after : or , if not already followed by a space
        if (chars[i] == ':' || chars[i] == ',') && i + 1 < chars.len() && chars[i + 1] != ' ' {
            result.push(' ');
        }
    }

    Ok(result)
}
