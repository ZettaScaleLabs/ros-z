/// Python type registry for ROS message serialization/deserialization
///
/// This module provides functions to serialize Python dicts to CDR bytes
/// and deserialize CDR bytes to Python dicts for all ROS message types.

use pyo3::prelude::*;
use pyo3::types::PyDict;
use std::collections::HashMap;
use once_cell::sync::Lazy;

type SerializeFn = fn(Python, &Bound<'_, PyDict>) -> PyResult<Vec<u8>>;
type DeserializeFn = fn(Python, &[u8]) -> PyResult<Py<PyDict>>;

/// Registry storing serialize/deserialize functions for each message type
struct TypeRegistry {
    serializers: HashMap<String, SerializeFn>,
    deserializers: HashMap<String, DeserializeFn>,
    type_hashes: HashMap<String, String>,
}

static REGISTRY: Lazy<parking_lot::RwLock<TypeRegistry>> = Lazy::new(|| {
    parking_lot::RwLock::new(TypeRegistry {
        serializers: HashMap::new(),
        deserializers: HashMap::new(),
        type_hashes: HashMap::new(),
    })
});

/// Register a message type with its serialization functions
pub fn register_type(
    type_name: &str,
    serialize_fn: SerializeFn,
    deserialize_fn: DeserializeFn,
    type_hash: &str,
) {
    let mut registry = REGISTRY.write();
    registry.serializers.insert(type_name.to_string(), serialize_fn);
    registry.deserializers.insert(type_name.to_string(), deserialize_fn);
    registry.type_hashes.insert(type_name.to_string(), type_hash.to_string());
}

/// Serialize a Python dict to CDR bytes for a given message type
pub fn serialize_to_cdr(type_name: &str, py: Python, data: &Bound<'_, PyDict>) -> PyResult<Vec<u8>> {
    let registry = REGISTRY.read();
    let serialize_fn = registry.serializers.get(type_name)
        .ok_or_else(|| pyo3::exceptions::PyTypeError::new_err(
            format!("Unknown message type: {}", type_name)
        ))?;

    serialize_fn(py, data)
}

/// Deserialize CDR bytes to a Python dict for a given message type
pub fn deserialize_from_cdr(type_name: &str, py: Python, data: &[u8]) -> PyResult<Py<PyDict>> {
    let registry = REGISTRY.read();
    let deserialize_fn = registry.deserializers.get(type_name)
        .ok_or_else(|| pyo3::exceptions::PyTypeError::new_err(
            format!("Unknown message type: {}", type_name)
        ))?;

    deserialize_fn(py, data)
}

/// Get the RIHS01 type hash for a message type
pub fn get_type_hash(type_name: &str) -> Option<String> {
    let registry = REGISTRY.read();
    registry.type_hashes.get(type_name).cloned()
}

/// Get list of all registered message types
pub fn list_registered_types() -> Vec<String> {
    let registry = REGISTRY.read();
    registry.serializers.keys().cloned().collect()
}

// Example registration for std_msgs/String
// This would be auto-generated in the future
#[cfg(feature = "std_msgs")]
mod std_msgs_registry {
    use super::*;
    use crate::std_msgs::String as RosString;
    use cdr::{CdrLe, Infinite};

    fn serialize_string(_py: Python, data: &Bound<'_, PyDict>) -> PyResult<Vec<u8>> {
        // Extract the 'data' field from Python dict
        let data_str: String = data.get_item("data")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'data' field"))?
            .extract()?;

        // Create ROS message
        let msg = RosString { data: data_str };

        // Serialize to CDR
        cdr::serialize::<_, _, CdrLe>(&msg, Infinite)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(
                format!("CDR serialization failed: {}", e)
            ))
    }

    fn deserialize_string(py: Python, bytes: &[u8]) -> PyResult<Py<PyDict>> {
        // Deserialize from CDR
        let msg: RosString = cdr::deserialize::<RosString>(bytes)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(
                format!("CDR deserialization failed: {}", e)
            ))?;

        // Create Python dict
        let dict = PyDict::new_bound(py);
        dict.set_item("data", msg.data)?;

        Ok(dict.unbind())
    }

    // Auto-register on module load
    pub(crate) fn register() {
        use crate::MessageTypeInfo;

        let type_info = RosString::type_info();
        super::register_type(
            "std_msgs/msg/String",
            serialize_string,
            deserialize_string,
            &type_info.hash.to_string(),
        );
    }
}

// sensor_msgs registry implementations
#[cfg(feature = "sensor_msgs")]
mod sensor_msgs_registry {
    use super::*;
    use crate::sensor_msgs::LaserScan;
    use crate::std_msgs::Header;
    use crate::builtin_interfaces::Time;
    use cdr::{CdrLe, Infinite};

    fn serialize_laser_scan(_py: Python, data: &Bound<'_, PyDict>) -> PyResult<Vec<u8>> {
        // Extract header
        let header_item = data.get_item("header")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'header' field"))?;
        let header_dict: &Bound<'_, PyDict> = header_item.downcast()?;

        let stamp_item = header_dict.get_item("stamp")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'header.stamp'"))?;
        let stamp_dict: &Bound<'_, PyDict> = stamp_item.downcast()?;
        let stamp = Time {
            sec: stamp_dict.get_item("sec")?.ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'stamp.sec'"))?.extract()?,
            nanosec: stamp_dict.get_item("nanosec")?.ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'stamp.nanosec'"))?.extract()?,
        };

        let frame_id: String = header_dict.get_item("frame_id")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'header.frame_id'"))?
            .extract()?;

        let header = Header { stamp, frame_id };

        // Extract other fields
        let angle_min: f32 = data.get_item("angle_min")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'angle_min'"))?
            .extract()?;
        let angle_max: f32 = data.get_item("angle_max")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'angle_max'"))?
            .extract()?;
        let angle_increment: f32 = data.get_item("angle_increment")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'angle_increment'"))?
            .extract()?;
        let time_increment: f32 = data.get_item("time_increment")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'time_increment'"))?
            .extract()?;
        let scan_time: f32 = data.get_item("scan_time")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'scan_time'"))?
            .extract()?;
        let range_min: f32 = data.get_item("range_min")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'range_min'"))?
            .extract()?;
        let range_max: f32 = data.get_item("range_max")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'range_max'"))?
            .extract()?;
        let ranges: Vec<f32> = data.get_item("ranges")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'ranges'"))?
            .extract()?;
        let intensities: Vec<f32> = data.get_item("intensities")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'intensities'"))?
            .extract()?;

        let msg = LaserScan {
            header,
            angle_min,
            angle_max,
            angle_increment,
            time_increment,
            scan_time,
            range_min,
            range_max,
            ranges,
            intensities,
        };

        cdr::serialize::<_, _, CdrLe>(&msg, Infinite)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(
                format!("CDR serialization failed: {}", e)
            ))
    }

    fn deserialize_laser_scan(py: Python, bytes: &[u8]) -> PyResult<Py<PyDict>> {
        let msg: LaserScan = cdr::deserialize::<LaserScan>(bytes)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(
                format!("CDR deserialization failed: {}", e)
            ))?;

        let dict = PyDict::new_bound(py);

        // Create header dict
        let header_dict = PyDict::new_bound(py);
        let stamp_dict = PyDict::new_bound(py);
        stamp_dict.set_item("sec", msg.header.stamp.sec)?;
        stamp_dict.set_item("nanosec", msg.header.stamp.nanosec)?;
        header_dict.set_item("stamp", stamp_dict)?;
        header_dict.set_item("frame_id", msg.header.frame_id)?;
        dict.set_item("header", header_dict)?;

        dict.set_item("angle_min", msg.angle_min)?;
        dict.set_item("angle_max", msg.angle_max)?;
        dict.set_item("angle_increment", msg.angle_increment)?;
        dict.set_item("time_increment", msg.time_increment)?;
        dict.set_item("scan_time", msg.scan_time)?;
        dict.set_item("range_min", msg.range_min)?;
        dict.set_item("range_max", msg.range_max)?;
        dict.set_item("ranges", msg.ranges)?;
        dict.set_item("intensities", msg.intensities)?;

        Ok(dict.unbind())
    }

    pub(crate) fn register() {
        use crate::MessageTypeInfo;

        let type_info = LaserScan::type_info();
        super::register_type(
            "sensor_msgs/msg/LaserScan",
            serialize_laser_scan,
            deserialize_laser_scan,
            &type_info.hash.to_string(),
        );
    }
}

// geometry_msgs registry implementations
#[cfg(feature = "geometry_msgs")]
mod geometry_msgs_registry {
    use super::*;
    use crate::geometry_msgs::{Vector3, Twist};
    use cdr::{CdrLe, Infinite};

    // Vector3 registration
    fn serialize_vector3(_py: Python, data: &Bound<'_, PyDict>) -> PyResult<Vec<u8>> {
        let x: f64 = data.get_item("x")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'x' field"))?
            .extract()?;
        let y: f64 = data.get_item("y")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'y' field"))?
            .extract()?;
        let z: f64 = data.get_item("z")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'z' field"))?
            .extract()?;

        let msg = Vector3 { x, y, z };
        cdr::serialize::<_, _, CdrLe>(&msg, Infinite)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(
                format!("CDR serialization failed: {}", e)
            ))
    }

    fn deserialize_vector3(py: Python, bytes: &[u8]) -> PyResult<Py<PyDict>> {
        let msg: Vector3 = cdr::deserialize::<Vector3>(bytes)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(
                format!("CDR deserialization failed: {}", e)
            ))?;

        let dict = PyDict::new_bound(py);
        dict.set_item("x", msg.x)?;
        dict.set_item("y", msg.y)?;
        dict.set_item("z", msg.z)?;

        Ok(dict.unbind())
    }

    // Twist registration
    fn serialize_twist(_py: Python, data: &Bound<'_, PyDict>) -> PyResult<Vec<u8>> {
        // Extract linear vector
        let linear_item = data.get_item("linear")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'linear' field"))?;
        let linear_dict: &Bound<'_, PyDict> = linear_item.downcast()?;
        let linear = Vector3 {
            x: linear_dict.get_item("x")?.ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'linear.x'"))?.extract()?,
            y: linear_dict.get_item("y")?.ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'linear.y'"))?.extract()?,
            z: linear_dict.get_item("z")?.ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'linear.z'"))?.extract()?,
        };

        // Extract angular vector
        let angular_item = data.get_item("angular")?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'angular' field"))?;
        let angular_dict: &Bound<'_, PyDict> = angular_item.downcast()?;
        let angular = Vector3 {
            x: angular_dict.get_item("x")?.ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'angular.x'"))?.extract()?,
            y: angular_dict.get_item("y")?.ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'angular.y'"))?.extract()?,
            z: angular_dict.get_item("z")?.ok_or_else(|| pyo3::exceptions::PyKeyError::new_err("Missing 'angular.z'"))?.extract()?,
        };

        let msg = Twist { linear, angular };
        cdr::serialize::<_, _, CdrLe>(&msg, Infinite)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(
                format!("CDR serialization failed: {}", e)
            ))
    }

    fn deserialize_twist(py: Python, bytes: &[u8]) -> PyResult<Py<PyDict>> {
        let msg: Twist = cdr::deserialize::<Twist>(bytes)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(
                format!("CDR deserialization failed: {}", e)
            ))?;

        let dict = PyDict::new_bound(py);

        // Create linear dict
        let linear_dict = PyDict::new_bound(py);
        linear_dict.set_item("x", msg.linear.x)?;
        linear_dict.set_item("y", msg.linear.y)?;
        linear_dict.set_item("z", msg.linear.z)?;
        dict.set_item("linear", linear_dict)?;

        // Create angular dict
        let angular_dict = PyDict::new_bound(py);
        angular_dict.set_item("x", msg.angular.x)?;
        angular_dict.set_item("y", msg.angular.y)?;
        angular_dict.set_item("z", msg.angular.z)?;
        dict.set_item("angular", angular_dict)?;

        Ok(dict.unbind())
    }

    pub(crate) fn register() {
        use crate::MessageTypeInfo;

        // Register Vector3
        let vector3_info = Vector3::type_info();
        super::register_type(
            "geometry_msgs/msg/Vector3",
            serialize_vector3,
            deserialize_vector3,
            &vector3_info.hash.to_string(),
        );

        // Register Twist
        let twist_info = Twist::type_info();
        super::register_type(
            "geometry_msgs/msg/Twist",
            serialize_twist,
            deserialize_twist,
            &twist_info.hash.to_string(),
        );
    }
}

/// Initialize the registry with all available message types
pub fn init_registry() {
    #[cfg(feature = "std_msgs")]
    std_msgs_registry::register();

    #[cfg(feature = "geometry_msgs")]
    geometry_msgs_registry::register();

    #[cfg(feature = "sensor_msgs")]
    sensor_msgs_registry::register();

    // More message types would be registered here...
    // This will be auto-generated in the future
}
