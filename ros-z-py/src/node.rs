use pyo3::prelude::*;
use pyo3::types::PyDict;
use ros_z::node::ZNode;
use ros_z::Builder;
use ros_z::entity::{TypeHash, TypeInfo};
use std::sync::Arc;
use crate::session::PySession;
use crate::error::IntoPyErr;
use crate::publisher::PyPublisher;
use crate::subscriber::PySubscriber;
use crate::qos::{qos_from_pydict, QOS_DEFAULT};
use crate::traits::{ZPubWrapper, ZSubWrapper};

#[pyclass]
pub struct PyNode {
    pub(crate) inner: Arc<ZNode>,
}

#[allow(unsafe_op_in_unsafe_fn)]
#[pymethods]
impl PyNode {
    /// Create a publisher for a given topic and message type
    ///
    /// Args:
    ///     topic: Topic name (e.g., "/chatter")
    ///     msg_type: Message type name (e.g., "std_msgs/msg/String")
    ///     qos: Optional QoS profile as dict
    fn create_publisher(
        &self,
        topic: String,
        msg_type: String,
        qos: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<PyPublisher> {
        // Parse QoS or use default
        let qos_profile = if let Some(qos_dict) = qos {
            qos_from_pydict(qos_dict)?
        } else {
            QOS_DEFAULT
        };

        // Get type hash from registry
        let type_hash_str = ros_z_msgs::get_type_hash(&msg_type)
            .map_err(|e| pyo3::exceptions::PyTypeError::new_err(
                format!("{}. Available types: {:?}",
                    e,
                    ros_z_msgs::list_registered_types())
            ))?;

        // Parse type hash
        let type_hash = TypeHash::from_rihs_string(&type_hash_str)
            .ok_or_else(|| pyo3::exceptions::PyValueError::new_err(
                format!("Invalid type hash format: {}", type_hash_str)
            ))?;

        // Create TypeInfo
        let type_info = TypeInfo::new(&msg_type, type_hash);

        // Create publisher based on message type
        // This uses runtime dispatch based on the message type string
        create_publisher_for_type(&self.inner, &topic, &msg_type, type_info, qos_profile)
    }

    /// Create a subscriber for a given topic and message type
    ///
    /// Args:
    ///     topic: Topic name (e.g., "/chatter")
    ///     msg_type: Message type name (e.g., "std_msgs/msg/String")
    ///     qos: Optional QoS profile as dict
    fn create_subscriber(
        &self,
        topic: String,
        msg_type: String,
        qos: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<PySubscriber> {
        // Parse QoS or use default
        let qos_profile = if let Some(qos_dict) = qos {
            qos_from_pydict(qos_dict)?
        } else {
            QOS_DEFAULT
        };

        // Get type hash from registry
        let type_hash_str = ros_z_msgs::get_type_hash(&msg_type)
            .map_err(|e| pyo3::exceptions::PyTypeError::new_err(
                format!("{}. Available types: {:?}",
                    e,
                    ros_z_msgs::list_registered_types())
            ))?;

        // Parse type hash
        let type_hash = TypeHash::from_rihs_string(&type_hash_str)
            .ok_or_else(|| pyo3::exceptions::PyValueError::new_err(
                format!("Invalid type hash format: {}", type_hash_str)
            ))?;

        // Create TypeInfo
        let type_info = TypeInfo::new(&msg_type, type_hash);

        // Create subscriber based on message type
        // This uses runtime dispatch based on the message type string
        create_subscriber_for_type(&self.inner, &topic, &msg_type, type_info, qos_profile)
    }
}

/// Create a ROS 2 node
#[allow(unsafe_op_in_unsafe_fn)]
#[pyfunction]
#[pyo3(signature = (session, name, namespace=None))]
pub fn create_node(
    session: &PySession,
    name: String,
    namespace: Option<String>,
) -> PyResult<PyNode> {
    let namespace = namespace.unwrap_or_else(|| "/".to_string());

    // Create node using the context from PySession
    let node = session.ctx
        .create_node(&name)
        .with_namespace(&namespace)
        .build()
        .map_err(|e| e.into_pyerr())?;

    Ok(PyNode {
        inner: Arc::new(node),
    })
}

/// Helper function to create a publisher for a specific message type
/// Uses runtime dispatch based on the message type string
fn create_publisher_for_type(
    node: &Arc<ZNode>,
    topic: &str,
    msg_type: &str,
    type_info: TypeInfo,
    qos: ros_z::qos::QosProfile,
) -> PyResult<PyPublisher> {
    use ros_z_msgs::{std_msgs, geometry_msgs, sensor_msgs};

    // Match on the message type and create the appropriate publisher
    match msg_type {
        "std_msgs/msg/String" => {
            let pub_builder = node.create_pub_impl::<std_msgs::String>(topic, Some(type_info))
                .with_qos(qos);
            let zpub = pub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZPubWrapper::new(zpub);
            Ok(PyPublisher::new(Box::new(wrapper), msg_type.to_string()))
        }
        "geometry_msgs/msg/Vector3" => {
            let pub_builder = node.create_pub_impl::<geometry_msgs::Vector3>(topic, Some(type_info))
                .with_qos(qos);
            let zpub = pub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZPubWrapper::new(zpub);
            Ok(PyPublisher::new(Box::new(wrapper), msg_type.to_string()))
        }
        "geometry_msgs/msg/Twist" => {
            let pub_builder = node.create_pub_impl::<geometry_msgs::Twist>(topic, Some(type_info))
                .with_qos(qos);
            let zpub = pub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZPubWrapper::new(zpub);
            Ok(PyPublisher::new(Box::new(wrapper), msg_type.to_string()))
        }
        "sensor_msgs/msg/LaserScan" => {
            let pub_builder = node.create_pub_impl::<sensor_msgs::LaserScan>(topic, Some(type_info))
                .with_qos(qos);
            let zpub = pub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZPubWrapper::new(zpub);
            Ok(PyPublisher::new(Box::new(wrapper), msg_type.to_string()))
        }
        _ => Err(pyo3::exceptions::PyTypeError::new_err(
            format!("Message type '{}' is registered but not implemented in publisher factory. \
                     Supported types: std_msgs/msg/String, geometry_msgs/msg/Vector3, geometry_msgs/msg/Twist, sensor_msgs/msg/LaserScan",
                    msg_type)
        ))
    }
}

/// Helper function to create a subscriber for a specific message type
/// Uses runtime dispatch based on the message type string
fn create_subscriber_for_type(
    node: &Arc<ZNode>,
    topic: &str,
    msg_type: &str,
    type_info: TypeInfo,
    qos: ros_z::qos::QosProfile,
) -> PyResult<PySubscriber> {
    use ros_z_msgs::{std_msgs, geometry_msgs, sensor_msgs};

    // Match on the message type and create the appropriate subscriber
    match msg_type {
        "std_msgs/msg/String" => {
            let sub_builder = node.create_sub_impl::<std_msgs::String>(topic, Some(type_info))
                .with_qos(qos);
            let zsub = sub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZSubWrapper::new(zsub);
            Ok(PySubscriber::new(Box::new(wrapper), msg_type.to_string()))
        }
        "geometry_msgs/msg/Vector3" => {
            let sub_builder = node.create_sub_impl::<geometry_msgs::Vector3>(topic, Some(type_info))
                .with_qos(qos);
            let zsub = sub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZSubWrapper::new(zsub);
            Ok(PySubscriber::new(Box::new(wrapper), msg_type.to_string()))
        }
        "geometry_msgs/msg/Twist" => {
            let sub_builder = node.create_sub_impl::<geometry_msgs::Twist>(topic, Some(type_info))
                .with_qos(qos);
            let zsub = sub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZSubWrapper::new(zsub);
            Ok(PySubscriber::new(Box::new(wrapper), msg_type.to_string()))
        }
        "sensor_msgs/msg/LaserScan" => {
            let sub_builder = node.create_sub_impl::<sensor_msgs::LaserScan>(topic, Some(type_info))
                .with_qos(qos);
            let zsub = sub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZSubWrapper::new(zsub);
            Ok(PySubscriber::new(Box::new(wrapper), msg_type.to_string()))
        }
        _ => Err(pyo3::exceptions::PyTypeError::new_err(
            format!("Message type '{}' is registered but not implemented in subscriber factory. \
                     Supported types: std_msgs/msg/String, geometry_msgs/msg/Vector3, geometry_msgs/msg/Twist, sensor_msgs/msg/LaserScan",
                    msg_type)
        ))
    }
}
