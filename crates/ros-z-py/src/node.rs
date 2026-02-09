use crate::error::IntoPyErr;
use crate::pubsub::{PyZPublisher, PyZSubscriber};
use crate::qos::{QOS_DEFAULT, qos_from_pydict};
use crate::service::{PyZClient, PyZServer, ZClientWrapper, ZServerWrapper};
use crate::traits::{ZPubWrapper, ZSubWrapper};
use crate::utils::python_type_to_rust_type;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use ros_z::Builder;
use ros_z::context::ZContext;
use ros_z::entity::{TypeHash, TypeInfo};
use ros_z::node::ZNode;
use std::sync::Arc;

/// Extract type information from a msgspec message class.
/// The class must have `__msgtype__` and `__hash__` class attributes.
fn extract_type_info_from_class(msg_class: &Bound<'_, PyAny>) -> PyResult<(String, TypeInfo)> {
    // Extract __msgtype__ (e.g., "std_msgs/msg/String")
    let msg_type: String = msg_class
        .getattr("__msgtype__")
        .map_err(|_| {
            pyo3::exceptions::PyTypeError::new_err(
                "Message class must have __msgtype__ class attribute",
            )
        })?
        .extract()
        .map_err(|_| {
            pyo3::exceptions::PyTypeError::new_err("Message class __msgtype__ must be a string")
        })?;

    // Extract __hash__ (e.g., "RIHS01_...")
    let type_hash_str: String = msg_class
        .getattr("__hash__")
        .map_err(|_| {
            pyo3::exceptions::PyTypeError::new_err(
                "Message class must have __hash__ class attribute",
            )
        })?
        .extract()
        .map_err(|_| {
            pyo3::exceptions::PyTypeError::new_err("Message class __hash__ must be a string")
        })?;

    // Parse type hash
    let type_hash = TypeHash::from_rihs_string(&type_hash_str).ok_or_else(|| {
        pyo3::exceptions::PyValueError::new_err(format!(
            "Invalid type hash format: {}",
            type_hash_str
        ))
    })?;

    // Convert Python-style type name to Rust-style type name
    let rust_type_name = python_type_to_rust_type(&msg_type);

    // Create TypeInfo
    let type_info = TypeInfo::new(&rust_type_name, type_hash);

    Ok((msg_type, type_info))
}

/// Extract service type information from a service Request class.
/// The class must have `__msgtype__` attribute like "example_interfaces/msg/AddTwoIntsRequest"
/// and `__hash__` attribute containing the service type hash.
/// This function converts it to the service type "example_interfaces/srv/AddTwoInts".
fn extract_service_type_from_request_class(
    request_class: &Bound<'_, PyAny>,
) -> PyResult<(String, TypeInfo)> {
    // Extract __msgtype__ (e.g., "example_interfaces/msg/AddTwoIntsRequest")
    let msg_type: String = request_class
        .getattr("__msgtype__")
        .map_err(|_| {
            pyo3::exceptions::PyTypeError::new_err(
                "Service Request class must have __msgtype__ class attribute",
            )
        })?
        .extract()
        .map_err(|_| {
            pyo3::exceptions::PyTypeError::new_err(
                "Service Request class __msgtype__ must be a string",
            )
        })?;

    // Extract __hash__ (service type hash for service types, e.g., "RIHS01_...")
    let type_hash_str: String = request_class
        .getattr("__hash__")
        .map_err(|_| {
            pyo3::exceptions::PyTypeError::new_err(
                "Service Request class must have __hash__ class attribute",
            )
        })?
        .extract()
        .map_err(|_| {
            pyo3::exceptions::PyTypeError::new_err(
                "Service Request class __hash__ must be a string",
            )
        })?;

    // Parse type hash
    let type_hash = TypeHash::from_rihs_string(&type_hash_str).ok_or_else(|| {
        pyo3::exceptions::PyValueError::new_err(format!(
            "Invalid type hash format: {}",
            type_hash_str
        ))
    })?;

    // Convert "example_interfaces/msg/AddTwoIntsRequest" to "example_interfaces/srv/AddTwoInts"
    let srv_type = msg_type
        .replace("/msg/", "/srv/")
        .trim_end_matches("Request")
        .trim_end_matches("Response")
        .to_string();

    // Convert Python-style type name to Rust-style type name
    let rust_type_name = python_type_to_rust_type(&srv_type);

    // Create TypeInfo using service type hash
    let type_info = TypeInfo::new(&rust_type_name, type_hash);

    Ok((srv_type, type_info))
}

#[pyclass(name = "ZNodeBuilder")]
pub struct PyZNodeBuilder {
    pub(crate) ctx: Arc<ZContext>,
    pub(crate) name: String,
    pub(crate) namespace: Option<String>,
}

#[pymethods]
impl PyZNodeBuilder {
    /// Set the namespace for the node
    pub fn with_namespace(mut slf: PyRefMut<'_, Self>, namespace: String) -> PyRefMut<'_, Self> {
        slf.namespace = Some(namespace);
        slf
    }

    /// Build the node
    pub fn build(&self) -> PyResult<PyZNode> {
        let mut builder = self.ctx.create_node(&self.name);
        if let Some(ref ns) = self.namespace {
            builder = builder.with_namespace(ns);
        }

        let node = builder.build().map_err(|e| e.into_pyerr())?;
        Ok(PyZNode {
            inner: Arc::new(node),
        })
    }
}

#[pyclass(name = "ZNode")]
pub struct PyZNode {
    pub(crate) inner: Arc<ZNode>,
}

#[allow(unsafe_op_in_unsafe_fn)]
#[pymethods]
impl PyZNode {
    /// Create a publisher for a given topic and message type
    ///
    /// Args:
    ///     topic: Topic name (e.g., "/chatter")
    ///     msg_type: Message class (e.g., std_msgs.String)
    ///     qos: Optional QoS profile as dict
    ///
    /// Example:
    ///     from ros_z_msgs_py.types.std_msgs import String
    ///     pub = node.create_publisher("/chatter", String)
    #[pyo3(signature = (topic, msg_type, qos=None))]
    fn create_publisher(
        &self,
        topic: String,
        msg_type: &Bound<'_, PyAny>,
        qos: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<PyZPublisher> {
        // Extract type info from the message class
        let (msg_type_str, type_info) = extract_type_info_from_class(msg_type)?;

        // Parse QoS or use default
        let qos_profile = if let Some(qos_dict) = qos {
            qos_from_pydict(qos_dict)?
        } else {
            QOS_DEFAULT
        };

        // Create publisher based on message type
        create_publisher_for_type(&self.inner, &topic, &msg_type_str, type_info, qos_profile)
    }

    /// Create a subscriber for a given topic and message type
    ///
    /// Args:
    ///     topic: Topic name (e.g., "/chatter")
    ///     msg_type: Message class (e.g., std_msgs.String)
    ///     qos: Optional QoS profile as dict
    ///
    /// Example:
    ///     from ros_z_msgs_py.types.std_msgs import String
    ///     sub = node.create_subscriber("/chatter", String)
    #[pyo3(signature = (topic, msg_type, qos=None))]
    fn create_subscriber(
        &self,
        topic: String,
        msg_type: &Bound<'_, PyAny>,
        qos: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<PyZSubscriber> {
        // Extract type info from the message class
        let (msg_type_str, type_info) = extract_type_info_from_class(msg_type)?;

        // Parse QoS or use default
        let qos_profile = if let Some(qos_dict) = qos {
            qos_from_pydict(qos_dict)?
        } else {
            QOS_DEFAULT
        };

        // Create subscriber based on message type
        create_subscriber_for_type(&self.inner, &topic, &msg_type_str, type_info, qos_profile)
    }

    /// Create a service client for a given service and service type
    ///
    /// Args:
    ///     service: Service name (e.g., "add_two_ints")
    ///     srv_type: Service Request class (e.g., example_interfaces.AddTwoIntsRequest)
    ///
    /// Example:
    ///     from ros_z_msgs_py.types.example_interfaces import AddTwoIntsRequest
    ///     client = node.create_client("add_two_ints", AddTwoIntsRequest)
    fn create_client(&self, service: String, srv_type: &Bound<'_, PyAny>) -> PyResult<PyZClient> {
        // Extract service type info from the Request class
        let (srv_type_str, type_info) = extract_service_type_from_request_class(srv_type)?;

        // Create client based on service type
        create_client_for_type(&self.inner, &service, &srv_type_str, type_info)
    }

    /// Create a service server for a given service and service type
    ///
    /// Args:
    ///     service: Service name (e.g., "add_two_ints")
    ///     srv_type: Service Request class (e.g., example_interfaces.AddTwoIntsRequest)
    ///
    /// Example:
    ///     from ros_z_msgs_py.types.example_interfaces import AddTwoIntsRequest
    ///     server = node.create_server("add_two_ints", AddTwoIntsRequest)
    fn create_server(&self, service: String, srv_type: &Bound<'_, PyAny>) -> PyResult<PyZServer> {
        // Extract service type info from the Request class
        let (srv_type_str, type_info) = extract_service_type_from_request_class(srv_type)?;

        // Create server based on service type
        create_server_for_type(&self.inner, &service, &srv_type_str, type_info)
    }
}

/// Helper function to create a publisher for a specific message type
/// Uses runtime dispatch based on the message type string
fn create_publisher_for_type(
    node: &Arc<ZNode>,
    topic: &str,
    msg_type: &str,
    type_info: TypeInfo,
    qos: ros_z::qos::QosProfile,
) -> PyResult<PyZPublisher> {
    use ros_z_msgs::{geometry_msgs, sensor_msgs, std_msgs};

    // Match on the message type and create the appropriate publisher
    match msg_type {
        "std_msgs/msg/String" => {
            let pub_builder = node
                .create_pub_impl::<std_msgs::String>(topic, Some(type_info))
                .with_qos(qos);
            let zpub = pub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZPubWrapper::new(zpub);
            Ok(PyZPublisher::new(Box::new(wrapper), msg_type.to_string()))
        }
        "geometry_msgs/msg/Vector3" => {
            let pub_builder = node
                .create_pub_impl::<geometry_msgs::Vector3>(topic, Some(type_info))
                .with_qos(qos);
            let zpub = pub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZPubWrapper::new(zpub);
            Ok(PyZPublisher::new(Box::new(wrapper), msg_type.to_string()))
        }
        "geometry_msgs/msg/Twist" => {
            let pub_builder = node
                .create_pub_impl::<geometry_msgs::Twist>(topic, Some(type_info))
                .with_qos(qos);
            let zpub = pub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZPubWrapper::new(zpub);
            Ok(PyZPublisher::new(Box::new(wrapper), msg_type.to_string()))
        }
        "sensor_msgs/msg/LaserScan" => {
            let pub_builder = node
                .create_pub_impl::<sensor_msgs::LaserScan>(topic, Some(type_info))
                .with_qos(qos);
            let zpub = pub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZPubWrapper::new(zpub);
            Ok(PyZPublisher::new(Box::new(wrapper), msg_type.to_string()))
        }
        "std_msgs/msg/ByteMultiArray" => {
            let pub_builder = node
                .create_pub_impl::<std_msgs::ByteMultiArray>(topic, Some(type_info))
                .with_qos(qos);
            let zpub = pub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZPubWrapper::new(zpub);
            Ok(PyZPublisher::new(Box::new(wrapper), msg_type.to_string()))
        }
        _ => Err(pyo3::exceptions::PyTypeError::new_err(format!(
            "Message type '{}' is registered but not implemented in publisher factory. \
                     Supported types: std_msgs/msg/String, std_msgs/msg/ByteMultiArray, geometry_msgs/msg/Vector3, geometry_msgs/msg/Twist, sensor_msgs/msg/LaserScan",
            msg_type
        ))),
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
) -> PyResult<PyZSubscriber> {
    use ros_z_msgs::{geometry_msgs, sensor_msgs, std_msgs};

    // Match on the message type and create the appropriate subscriber
    match msg_type {
        "std_msgs/msg/String" => {
            let sub_builder = node
                .create_sub_impl::<std_msgs::String>(topic, Some(type_info))
                .with_qos(qos);
            let zsub = sub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZSubWrapper::new(zsub);
            Ok(PyZSubscriber::new(Box::new(wrapper), msg_type.to_string()))
        }
        "geometry_msgs/msg/Vector3" => {
            let sub_builder = node
                .create_sub_impl::<geometry_msgs::Vector3>(topic, Some(type_info))
                .with_qos(qos);
            let zsub = sub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZSubWrapper::new(zsub);
            Ok(PyZSubscriber::new(Box::new(wrapper), msg_type.to_string()))
        }
        "geometry_msgs/msg/Twist" => {
            let sub_builder = node
                .create_sub_impl::<geometry_msgs::Twist>(topic, Some(type_info))
                .with_qos(qos);
            let zsub = sub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZSubWrapper::new(zsub);
            Ok(PyZSubscriber::new(Box::new(wrapper), msg_type.to_string()))
        }
        "sensor_msgs/msg/LaserScan" => {
            let sub_builder = node
                .create_sub_impl::<sensor_msgs::LaserScan>(topic, Some(type_info))
                .with_qos(qos);
            let zsub = sub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZSubWrapper::new(zsub);
            Ok(PyZSubscriber::new(Box::new(wrapper), msg_type.to_string()))
        }
        "std_msgs/msg/ByteMultiArray" => {
            let sub_builder = node
                .create_sub_impl::<std_msgs::ByteMultiArray>(topic, Some(type_info))
                .with_qos(qos);
            let zsub = sub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZSubWrapper::new(zsub);
            Ok(PyZSubscriber::new(Box::new(wrapper), msg_type.to_string()))
        }
        _ => Err(pyo3::exceptions::PyTypeError::new_err(format!(
            "Message type '{}' is registered but not implemented in subscriber factory. \
                     Supported types: std_msgs/msg/String, std_msgs/msg/ByteMultiArray, geometry_msgs/msg/Vector3, geometry_msgs/msg/Twist, sensor_msgs/msg/LaserScan",
            msg_type
        ))),
    }
}

/// Helper function to create a service client for a specific service type
/// Uses runtime dispatch based on the service type string
fn create_client_for_type(
    node: &Arc<ZNode>,
    service: &str,
    srv_type: &str,
    type_info: TypeInfo,
) -> PyResult<PyZClient> {
    use ros_z_msgs::example_interfaces::srv;

    // Match on the service type and create the appropriate client
    match srv_type {
        "example_interfaces/srv/AddTwoInts" => {
            let client_builder =
                node.create_client_impl::<srv::AddTwoInts>(service, Some(type_info));
            let zclient = client_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZClientWrapper::new(zclient);
            Ok(PyZClient::new(Box::new(wrapper), srv_type.to_string()))
        }
        _ => Err(pyo3::exceptions::PyTypeError::new_err(format!(
            "Service type '{}' is registered but not implemented in client factory. \
                     Supported types: example_interfaces/srv/AddTwoInts",
            srv_type
        ))),
    }
}

/// Helper function to create a service server for a specific service type
/// Uses runtime dispatch based on the service type string
fn create_server_for_type(
    node: &Arc<ZNode>,
    service: &str,
    srv_type: &str,
    type_info: TypeInfo,
) -> PyResult<PyZServer> {
    use ros_z_msgs::example_interfaces::srv;

    // Match on the service type and create the appropriate server
    match srv_type {
        "example_interfaces/srv/AddTwoInts" => {
            let server_builder =
                node.create_service_impl::<srv::AddTwoInts>(service, Some(type_info));
            let zserver = server_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = ZServerWrapper::new(zserver);
            Ok(PyZServer::new(Box::new(wrapper), srv_type.to_string()))
        }
        _ => Err(pyo3::exceptions::PyTypeError::new_err(format!(
            "Service type '{}' is registered but not implemented in server factory. \
                     Supported types: example_interfaces/srv/AddTwoInts",
            srv_type
        ))),
    }
}
