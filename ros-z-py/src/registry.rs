use pyo3::prelude::*;
use pyo3::types::PyDict;
use std::collections::HashMap;
use once_cell::sync::Lazy;
use parking_lot::RwLock;

/// Runtime registry for user-defined message types
/// Stores Python callable objects for serialization/deserialization
struct RuntimeRegistry {
    serializers: HashMap<String, PyObject>,
    deserializers: HashMap<String, PyObject>,
    type_hashes: HashMap<String, String>,
}

static RUNTIME_REGISTRY: Lazy<RwLock<RuntimeRegistry>> = Lazy::new(|| {
    RwLock::new(RuntimeRegistry {
        serializers: HashMap::new(),
        deserializers: HashMap::new(),
        type_hashes: HashMap::new(),
    })
});

/// Register a custom message type at runtime
///
/// This allows users to register their own message types without rebuilding.
///
/// Args:
///     type_name: Full message type name (e.g., "my_package/msg/MyMessage")
///     serialize_fn: Python callable that takes a dict and returns bytes
///     deserialize_fn: Python callable that takes bytes and returns a dict
///     type_hash: RIHS01 type hash string
///
/// Example:
///     def serialize_my_msg(data):
///         # Custom serialization logic
///         return cdr_bytes
///
///     def deserialize_my_msg(cdr_bytes):
///         # Custom deserialization logic
///         return {"field": value}
///
///     register_message_type(
///         "my_package/msg/MyMessage",
///         serialize_my_msg,
///         deserialize_my_msg,
///         "RIHS01_abcd1234..."
///     )
#[allow(unsafe_op_in_unsafe_fn)]
#[pyfunction]
pub fn register_message_type(
    type_name: String,
    serialize_fn: PyObject,
    deserialize_fn: PyObject,
    type_hash: String,
) -> PyResult<()> {
    let mut registry = RUNTIME_REGISTRY.write();
    registry.serializers.insert(type_name.clone(), serialize_fn);
    registry.deserializers.insert(type_name.clone(), deserialize_fn);
    registry.type_hashes.insert(type_name, type_hash);
    Ok(())
}

/// Serialize using runtime registry (fallback for user-defined types)
#[allow(dead_code)]
pub(crate) fn runtime_serialize(
    type_name: &str,
    py: Python,
    data: &Bound<'_, PyDict>,
) -> PyResult<Vec<u8>> {
    let registry = RUNTIME_REGISTRY.read();
    let serialize_fn = registry.serializers.get(type_name)
        .ok_or_else(|| pyo3::exceptions::PyTypeError::new_err(
            format!("Message type '{}' not found in runtime registry", type_name)
        ))?;

    // Call the Python serialization function
    let result = serialize_fn.call1(py, (data,))?;
    let bytes: Vec<u8> = result.extract(py)?;
    Ok(bytes)
}

/// Deserialize using runtime registry (fallback for user-defined types)
#[allow(dead_code)]
pub(crate) fn runtime_deserialize(
    type_name: &str,
    py: Python,
    data: &[u8],
) -> PyResult<Py<PyDict>> {
    let registry = RUNTIME_REGISTRY.read();
    let deserialize_fn = registry.deserializers.get(type_name)
        .ok_or_else(|| pyo3::exceptions::PyTypeError::new_err(
            format!("Message type '{}' not found in runtime registry", type_name)
        ))?;

    // Convert bytes to Python bytes object
    let py_bytes = pyo3::types::PyBytes::new_bound(py, data);

    // Call the Python deserialization function
    let result = deserialize_fn.call1(py, (py_bytes,))?;
    let dict: Py<PyDict> = result.extract(py)?;
    Ok(dict)
}

/// Get type hash from runtime registry
#[allow(dead_code)]
pub(crate) fn runtime_get_type_hash(type_name: &str) -> Option<String> {
    let registry = RUNTIME_REGISTRY.read();
    registry.type_hashes.get(type_name).cloned()
}

/// Compute RIHS01 type hash for a message definition
///
/// Note: This is a placeholder implementation. Full RIHS01 computation
/// requires parsing the message definition and hashing its structure.
/// For now, this returns a simple hash that can be used for testing.
///
/// Args:
///     msg_definition: Message definition string (e.g., .msg file contents)
///
/// Returns:
///     RIHS01 type hash string
#[allow(unsafe_op_in_unsafe_fn)]
#[pyfunction]
pub fn compute_type_hash(msg_definition: &str) -> PyResult<String> {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    // Simple hash implementation for now
    // TODO: Implement proper RIHS01 computation
    let mut hasher = DefaultHasher::new();
    msg_definition.hash(&mut hasher);
    let hash = hasher.finish();

    // Format as RIHS01 string (this is a simplified version)
    Ok(format!("RIHS01_{:016x}00000000000000000000000000000000", hash))
}
