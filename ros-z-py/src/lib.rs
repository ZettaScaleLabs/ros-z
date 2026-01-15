use pyo3::prelude::*;

mod error;
mod macros;
mod node;
mod publisher;
mod qos;
mod registry;
mod ring_buffer;
mod session;
mod subscriber;
mod traits;
mod utils;

use node::*;
use publisher::*;
use session::*;
use subscriber::*;

/// Get list of all registered message types
#[pyfunction]
fn list_registered_types() -> Vec<String> {
    ros_z_msgs::list_registered_types()
}

/// Python bindings for ros-z: Native Rust ROS 2 implementation using Zenoh
#[pymodule]
fn ros_z_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Initialize the message type registry
    ros_z_msgs::init_registry();

    // Register custom exceptions
    m.add("RosZError", m.py().get_type_bound::<error::RosZError>())?;
    m.add(
        "TimeoutError",
        m.py().get_type_bound::<error::TimeoutError>(),
    )?;
    m.add(
        "SerializationError",
        m.py().get_type_bound::<error::SerializationError>(),
    )?;
    m.add(
        "TypeMismatchError",
        m.py().get_type_bound::<error::TypeMismatchError>(),
    )?;

    // Register functions
    m.add_function(wrap_pyfunction!(open_session, m)?)?;
    m.add_function(wrap_pyfunction!(create_node, m)?)?;
    m.add_function(wrap_pyfunction!(registry::register_message_type, m)?)?;
    m.add_function(wrap_pyfunction!(registry::compute_type_hash, m)?)?;
    m.add_function(wrap_pyfunction!(list_registered_types, m)?)?;

    // Register classes
    m.add_class::<PySession>()?;
    m.add_class::<PyNode>()?;
    m.add_class::<PyPublisher>()?;
    m.add_class::<PySubscriber>()?;

    // QoS presets
    m.add(
        "QOS_DEFAULT",
        qos::qos_to_pydict(m.py(), &qos::QOS_DEFAULT)?,
    )?;
    m.add(
        "QOS_SENSOR_DATA",
        qos::qos_to_pydict(m.py(), &qos::QOS_SENSOR_DATA)?,
    )?;
    m.add(
        "QOS_PARAMETERS",
        qos::qos_to_pydict(m.py(), &qos::QOS_PARAMETERS)?,
    )?;
    m.add(
        "QOS_SERVICES",
        qos::qos_to_pydict(m.py(), &qos::QOS_SERVICES)?,
    )?;

    Ok(())
}
