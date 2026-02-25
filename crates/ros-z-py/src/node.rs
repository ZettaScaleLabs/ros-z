use crate::action::{PyZActionClient, PyZActionServer, get_tokio_rt};
use crate::error::IntoPyErr;
use crate::graph::GraphQueries;
use crate::pubsub::{PyZPublisher, PyZSubscriber};
use crate::qos::extract_qos;
use crate::raw_bytes::{RawBytesAction, RawBytesCdrSerdes, RawBytesMessage, RawBytesService};
use crate::service::{PyZClient, PyZServer};
use crate::traits::{
    GenericClientWrapper, GenericPubWrapper, GenericServerWrapper, GenericSubWrapper,
};
use crate::utils::python_type_to_rust_type;
use pyo3::prelude::*;
use ros_z::Builder;
use ros_z::context::ZContext;
use ros_z::entity::{TypeHash, TypeInfo};
use ros_z::node::ZNode;
use std::sync::Arc;

/// Extract type information from a msgspec message class.
/// The class must have `__msgtype__` and `__hash__` class attributes.
fn extract_type_info_from_class(msg_class: &Bound<'_, PyAny>) -> PyResult<(String, TypeInfo)> {
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

    let type_hash = TypeHash::from_rihs_string(&type_hash_str).ok_or_else(|| {
        pyo3::exceptions::PyValueError::new_err(format!(
            "Invalid type hash format: {}",
            type_hash_str
        ))
    })?;

    let rust_type_name = python_type_to_rust_type(&msg_type);
    let type_info = TypeInfo::new(&rust_type_name, type_hash);

    Ok((msg_type, type_info))
}

/// Extract service type information from a service Request class.
fn extract_service_type_from_request_class(
    request_class: &Bound<'_, PyAny>,
) -> PyResult<(String, TypeInfo)> {
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

    let rust_type_name = python_type_to_rust_type(&srv_type);
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
            name: self.name.clone(),
            namespace: self.namespace.clone().unwrap_or_else(|| "/".to_string()),
        })
    }
}

#[pyclass(name = "ZNode")]
pub struct PyZNode {
    pub(crate) inner: Arc<ZNode>,
    name: String,
    namespace: String,
}

#[allow(unsafe_op_in_unsafe_fn)]
#[pymethods]
impl PyZNode {
    /// Get the node name
    #[getter]
    fn name(&self) -> &str {
        &self.name
    }

    /// Get the node namespace
    #[getter]
    fn namespace(&self) -> &str {
        &self.namespace
    }

    /// Get the fully qualified node name (namespace + name)
    #[getter]
    fn fully_qualified_name(&self) -> String {
        if self.namespace == "/" {
            format!("/{}", self.name)
        } else {
            format!("{}/{}", self.namespace, self.name)
        }
    }

    /// Create a publisher for a given topic and message type.
    ///
    /// Works with any registered message type — no factory limitations.
    #[pyo3(signature = (topic, msg_type, qos=None))]
    fn create_publisher(
        &self,
        topic: String,
        msg_type: &Bound<'_, PyAny>,
        qos: Option<&Bound<'_, PyAny>>,
    ) -> PyResult<PyZPublisher> {
        let (msg_type_str, type_info) = extract_type_info_from_class(msg_type)?;
        let qos_profile = extract_qos(qos)?;

        let pub_builder = self
            .inner
            .create_pub_impl::<RawBytesMessage>(&topic, Some(type_info))
            .with_serdes::<RawBytesCdrSerdes>()
            .with_qos(qos_profile);
        let zpub = pub_builder.build().map_err(|e| e.into_pyerr())?;
        let wrapper = GenericPubWrapper::new(zpub);
        Ok(PyZPublisher::new(Box::new(wrapper), msg_type_str))
    }

    /// Create a subscriber for a given topic and message type.
    ///
    /// Works with any registered message type — no factory limitations.
    #[pyo3(signature = (topic, msg_type, qos=None, callback=None))]
    fn create_subscriber(
        &self,
        _py: Python,
        topic: String,
        msg_type: &Bound<'_, PyAny>,
        qos: Option<&Bound<'_, PyAny>>,
        callback: Option<PyObject>,
    ) -> PyResult<PyZSubscriber> {
        let (msg_type_str, type_info) = extract_type_info_from_class(msg_type)?;
        let qos_profile = extract_qos(qos)?;

        let sub_builder = self
            .inner
            .create_sub_impl::<RawBytesMessage>(&topic, Some(type_info))
            .with_serdes::<RawBytesCdrSerdes>()
            .with_qos(qos_profile);

        if let Some(py_callback) = callback {
            // Callback-based subscription: no queue, callback fires on each message
            let type_name = msg_type_str.clone();
            let _zsub = sub_builder
                .build_with_callback(move |raw_msg: RawBytesMessage| {
                    let payload = raw_msg.0;
                    Python::with_gil(|py| {
                        match ros_z_msgs::deserialize_from_cdr(&type_name, py, &payload) {
                            Ok(obj) => {
                                if let Err(e) = py_callback.call1(py, (obj,)) {
                                    eprintln!("ros_z_py: callback error: {}", e);
                                }
                            }
                            Err(e) => {
                                eprintln!("ros_z_py: deserialization error in callback: {}", e);
                            }
                        }
                    });
                })
                .map_err(|e| e.into_pyerr())?;

            // Callback subs don't have a queue, wrap with a stub
            Ok(PyZSubscriber::new_callback(msg_type_str))
        } else {
            let zsub = sub_builder.build().map_err(|e| e.into_pyerr())?;
            let wrapper = GenericSubWrapper::new(zsub);
            Ok(PyZSubscriber::new(Box::new(wrapper), msg_type_str))
        }
    }

    /// Create a service client
    fn create_client(&self, service: String, srv_type: &Bound<'_, PyAny>) -> PyResult<PyZClient> {
        let (srv_type_str, type_info) = extract_service_type_from_request_class(srv_type)?;

        let client_builder = self
            .inner
            .create_client_impl::<RawBytesService>(&service, Some(type_info));
        let zclient = client_builder.build().map_err(|e| e.into_pyerr())?;
        let wrapper = GenericClientWrapper::new(zclient);
        Ok(PyZClient::new(Box::new(wrapper), srv_type_str))
    }

    // -- Graph discovery methods --

    /// Get all topic names and their types.
    /// Returns list of (topic_name, type_name) tuples.
    fn get_topic_names_and_types(&self) -> Vec<(String, String)> {
        GraphQueries::get_topic_names_and_types(&self.inner.graph)
    }

    /// Get all node names.
    /// Returns list of (name, namespace) tuples.
    fn get_node_names(&self) -> Vec<(String, String)> {
        GraphQueries::get_node_names(&self.inner.graph)
    }

    /// Get all service names and their types.
    /// Returns list of (service_name, type_name) tuples.
    fn get_service_names_and_types(&self) -> Vec<(String, String)> {
        GraphQueries::get_service_names_and_types(&self.inner.graph)
    }

    /// Count publishers for a topic.
    fn count_publishers(&self, topic: String) -> usize {
        GraphQueries::count_publishers(&self.inner.graph, &topic)
    }

    /// Count subscribers for a topic.
    fn count_subscribers(&self, topic: String) -> usize {
        GraphQueries::count_subscribers(&self.inner.graph, &topic)
    }

    /// Create an action client.
    ///
    /// `goal_type`, `result_type`, `feedback_type` must be msgspec classes with
    /// `__msgtype__` and `__hash__` attributes (from `ros_z_msgs_py`).
    ///
    /// Returns a `ZActionClient` for sending goals and receiving results.
    fn create_action_client(
        &self,
        py: Python,
        action_name: String,
        goal_type: &Bound<'_, PyAny>,
        result_type: &Bound<'_, PyAny>,
        feedback_type: &Bound<'_, PyAny>,
    ) -> PyResult<PyZActionClient> {
        let (goal_type_name, _) = extract_type_info_from_class(goal_type)?;
        let (result_type_name, _) = extract_type_info_from_class(result_type)?;
        let (feedback_type_name, _) = extract_type_info_from_class(feedback_type)?;

        let node = Arc::clone(&self.inner);
        let rt = get_tokio_rt();

        let client = py.allow_threads(|| {
            let _guard = rt.enter();
            node.create_action_client::<RawBytesAction>(&action_name)
                .build()
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
        })?;

        Ok(PyZActionClient::new(
            client,
            goal_type_name,
            result_type_name,
            feedback_type_name,
        ))
    }

    /// Create an action server.
    ///
    /// `goal_type`, `result_type`, `feedback_type` must be msgspec classes with
    /// `__msgtype__` and `__hash__` attributes (from `ros_z_msgs_py`).
    ///
    /// Returns a `ZActionServer` for receiving and executing goals.
    fn create_action_server(
        &self,
        py: Python,
        action_name: String,
        goal_type: &Bound<'_, PyAny>,
        result_type: &Bound<'_, PyAny>,
        feedback_type: &Bound<'_, PyAny>,
    ) -> PyResult<PyZActionServer> {
        let (goal_type_name, _) = extract_type_info_from_class(goal_type)?;
        let (result_type_name, _) = extract_type_info_from_class(result_type)?;
        let (feedback_type_name, _) = extract_type_info_from_class(feedback_type)?;

        let node = Arc::clone(&self.inner);
        let rt = get_tokio_rt();

        let server = py.allow_threads(|| {
            let _guard = rt.enter();
            node.create_action_server::<RawBytesAction>(&action_name)
                .build()
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
        })?;

        Ok(PyZActionServer::new(
            server,
            goal_type_name,
            result_type_name,
            feedback_type_name,
        ))
    }

    /// Create a service server
    fn create_server(&self, service: String, srv_type: &Bound<'_, PyAny>) -> PyResult<PyZServer> {
        let (srv_type_str, type_info) = extract_service_type_from_request_class(srv_type)?;

        let server_builder = self
            .inner
            .create_service_impl::<RawBytesService>(&service, Some(type_info));
        let zserver = server_builder.build().map_err(|e| e.into_pyerr())?;
        let wrapper = GenericServerWrapper::new(zserver);
        Ok(PyZServer::new(Box::new(wrapper), srv_type_str))
    }
}
