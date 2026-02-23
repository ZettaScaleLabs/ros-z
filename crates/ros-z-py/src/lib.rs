#![allow(unsafe_op_in_unsafe_fn, clippy::useless_conversion, private_interfaces)]

use pyo3::prelude::*;

mod context;
mod error;
mod node;
mod payload_view;
mod pubsub;
mod qos;
mod service;
mod traits;
mod utils;

use context::*;
use node::*;
use payload_view::ZPayloadView;
use pubsub::{PyZPublisher, PyZSubscriber};
use service::*;

/// Get list of all registered message types
#[pyfunction]
fn list_registered_types() -> Vec<String> {
    ros_z_msgs::list_registered_types()
}

/// Python bindings for ros-z: Native Rust ROS 2 implementation using Zenoh
#[pymodule]
fn ros_z_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
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
    m.add_function(wrap_pyfunction!(list_registered_types, m)?)?;

    // Register classes
    m.add_class::<PyZContextBuilder>()?;
    m.add_class::<PyZContext>()?;
    m.add_class::<PyZNodeBuilder>()?;
    m.add_class::<PyZNode>()?;
    m.add_class::<PyZPublisher>()?;
    m.add_class::<PyZSubscriber>()?;
    m.add_class::<PyZClient>()?;
    m.add_class::<PyZServer>()?;
    m.add_class::<ZPayloadView>()?;
    m.add_class::<ros_z::zbuf_view::ZBufView>()?;

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

    // Register the ros_z_msgs submodule with message registry
    let ros_z_msgs_module = PyModule::new_bound(m.py(), "ros_z_msgs")?;
    ros_z_msgs::ros_z_msgs(m.py(), &ros_z_msgs_module)?;
    m.add_submodule(&ros_z_msgs_module)?;

    // Register the submodule in sys.modules so it can be accessed from Rust
    let sys = m.py().import_bound("sys")?;
    let sys_modules = sys.getattr("modules")?;
    sys_modules.set_item("ros_z_py.ros_z_msgs", &ros_z_msgs_module)?;

    Ok(())
}
