use anyhow::Result;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use ros_z::msg::{ZMessage, ZService, ZDeserializer};
use ros_z::service::{ZClient, ZServer, QueryKey};
use std::time::Duration;

/// Type-erased client trait for Python interop
pub(crate) trait RawClient: Send + Sync {
    fn send_request_serialized(&self, data: &[u8]) -> Result<()>;
    fn take_response_serialized(&self, timeout: Option<Duration>) -> Result<Vec<u8>>;
    fn try_take_response_serialized(&self) -> Result<Option<Vec<u8>>>;
}


/// Wrapper for ZClient that implements RawClient
pub struct ZClientWrapper<T: ZService> {
    inner: ZClient<T>,
}

impl<T: ZService> ZClientWrapper<T> {
    pub fn new(inner: ZClient<T>) -> Self {
        Self { inner }
    }
}

impl<T> RawClient for ZClientWrapper<T>
where
    T: ZService + Send + Sync + 'static,
    T::Request: ZMessage + Send + Sync + 'static,
    T::Response: ZMessage + Send + Sync + 'static,
    for<'a> <T::Request as ZMessage>::Serdes: ZDeserializer<Output = T::Request, Input<'a> = &'a [u8]>,
    for<'a> <T::Response as ZMessage>::Serdes: ZDeserializer<Output = T::Response, Input<'a> = &'a [u8]>,
{
    fn send_request_serialized(&self, data: &[u8]) -> Result<()> {
        // Deserialize the request from CDR bytes
        let request = <T::Request as ZMessage>::deserialize(data)
            .map_err(|e| anyhow::anyhow!("Failed to deserialize request: {:?}", e))?;
        
        // Send the request (this is async in Rust, but we'll block here for Python)
        let rt = tokio::runtime::Handle::try_current()
            .or_else(|_| {
                // If no runtime exists, create a new one
                tokio::runtime::Runtime::new()
                    .map(|rt| rt.handle().clone())
            })?;
        
        rt.block_on(async {
            self.inner.send_request(&request).await
                .map_err(|e| anyhow::anyhow!("Failed to send request: {}", e))
        })
    }

    fn take_response_serialized(&self, timeout: Option<Duration>) -> Result<Vec<u8>> {
        // Use a very long timeout if none specified (1 hour should be sufficient)
        let timeout_duration = timeout.unwrap_or(Duration::from_secs(3600));
        let response = self.inner.take_response_timeout(timeout_duration)
            .map_err(|e| anyhow::anyhow!("Failed to receive response: {}", e))?;

        // Serialize the response to CDR bytes
        Ok(response.serialize())
    }

    fn try_take_response_serialized(&self) -> Result<Option<Vec<u8>>> {
        // Use a minimal timeout for non-blocking behavior
        match self.inner.take_response_timeout(Duration::from_millis(1)) {
            Ok(response) => Ok(Some(response.serialize())),
            Err(e) => {
                let err_str = e.to_string();
                if err_str.contains("timeout") || err_str.contains("Timeout") || err_str.contains("No sample available") {
                    Ok(None)
                } else {
                    Err(anyhow::anyhow!("Failed to receive response: {}", e))
                }
            }
        }
    }
}

/// Type-erased server trait for Python interop
/// Uses interior mutability since servers need mutable access for request/response handling
pub(crate) trait RawServer: Send + Sync {
    fn take_request_serialized(&self) -> Result<(QueryKey, Vec<u8>)>;
    fn send_response_serialized(&self, data: &[u8], key: &QueryKey) -> Result<()>;
}

/// Wrapper for ZServer that implements RawServer
pub struct ZServerWrapper<T: ZService> {
    inner: std::sync::Mutex<ZServer<T>>,
}

impl<T: ZService> ZServerWrapper<T> {
    pub fn new(inner: ZServer<T>) -> Self {
        Self {
            inner: std::sync::Mutex::new(inner),
        }
    }
}

impl<T> RawServer for ZServerWrapper<T>
where
    T: ZService + Send + Sync + 'static,
    T::Request: ZMessage + Send + Sync + 'static,
    T::Response: ZMessage + Send + Sync + 'static,
    for<'a> <T::Request as ZMessage>::Serdes: ZDeserializer<Output = T::Request, Input<'a> = &'a [u8]>,
    for<'a> <T::Response as ZMessage>::Serdes: ZDeserializer<Output = T::Response, Input<'a> = &'a [u8]>,
{
    fn take_request_serialized(&self) -> Result<(QueryKey, Vec<u8>)> {
        let mut server = self.inner.lock()
            .map_err(|e| anyhow::anyhow!("Failed to lock server: {}", e))?;

        let (key, request) = server.take_request()
            .map_err(|e| anyhow::anyhow!("Failed to receive request: {}", e))?;

        // Serialize the request to CDR bytes
        Ok((key, request.serialize()))
    }

    fn send_response_serialized(&self, data: &[u8], key: &QueryKey) -> Result<()> {
        // Deserialize the response from CDR bytes
        let response = <T::Response as ZMessage>::deserialize(data)
            .map_err(|e| anyhow::anyhow!("Failed to deserialize response: {:?}", e))?;
        
        let mut server = self.inner.lock()
            .map_err(|e| anyhow::anyhow!("Failed to lock server: {}", e))?;
        
        server.send_response(&response, key)
            .map_err(|e| anyhow::anyhow!("Failed to send response: {}", e))
    }
}


/// Python wrapper for service client
#[pyclass(name = "ZClient")]
pub struct PyZClient {
    inner: Box<dyn RawClient>,
    request_type_name: String,
    response_type_name: String,
}

impl PyZClient {
    pub fn new(inner: Box<dyn RawClient>, service_type: String) -> Self {
        // Convert service type to Request/Response type names
        // e.g., "example_interfaces/srv/AddTwoInts" -> "example_interfaces/srv/AddTwoInts_Request"
        let request_type_name = format!("{}_Request", service_type);
        let response_type_name = format!("{}_Response", service_type);
        Self { inner, request_type_name, response_type_name }
    }
}

#[allow(unsafe_op_in_unsafe_fn)]
#[pymethods]
impl PyZClient {
    /// Send a service request
    ///
    /// Args:
    ///     data: Request message (msgspec.Struct)
    unsafe fn send_request(&self, py: Python, data: &Bound<'_, PyAny>) -> PyResult<()> {
        // Serialize Python message to CDR bytes using the registry (Request type)
        let cdr_bytes = ros_z_msgs::serialize_to_cdr(&self.request_type_name, data.py(), data)?;

        // Send the serialized request (release GIL during blocking operation)
        py.allow_threads(|| {
            self.inner
                .send_request_serialized(&cdr_bytes)
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
        })
    }

    /// Receive a service response (blocking)
    ///
    /// Args:
    ///     timeout: Optional timeout in seconds (None = block forever)
    ///
    /// Returns:
    ///     Response message, or None if timeout occurred
    #[pyo3(signature = (timeout=None))]
    unsafe fn take_response(&self, py: Python, timeout: Option<f64>) -> PyResult<Option<PyObject>> {
        let timeout_duration = timeout.map(Duration::from_secs_f64);

        // Release GIL during blocking operation
        let result = py.allow_threads(|| {
            self.inner.take_response_serialized(timeout_duration)
        });

        match result {
            Ok(cdr_bytes) => {
                // Deserialize CDR bytes to Python object using the registry (Response type)
                let obj = ros_z_msgs::deserialize_from_cdr(&self.response_type_name, py, &cdr_bytes)?;
                Ok(Some(obj))
            }
            Err(e) => {
                // Check if it's a timeout error
                let err_str = e.to_string();
                if err_str.contains("timeout") || err_str.contains("Timeout") || err_str.contains("timed out") || err_str.contains("No sample available") {
                    Ok(None)
                } else {
                    Err(pyo3::exceptions::PyRuntimeError::new_err(err_str))
                }
            }
        }
    }

    /// Try to receive a response without blocking
    ///
    /// Returns:
    ///     Response message, or None if no response available
    unsafe fn try_take_response(&self, py: Python) -> PyResult<Option<PyObject>> {
        match self.inner.try_take_response_serialized() {
            Ok(Some(cdr_bytes)) => {
                // Deserialize CDR bytes to Python object using the registry (Response type)
                let obj = ros_z_msgs::deserialize_from_cdr(&self.response_type_name, py, &cdr_bytes)?;
                Ok(Some(obj))
            }
            Ok(None) => Ok(None),
            Err(e) => Err(pyo3::exceptions::PyRuntimeError::new_err(e.to_string())),
        }
    }

    /// Get the service type name (for debugging)
    unsafe fn get_type_name(&self) -> String {
        format!("request={}, response={}", self.request_type_name, self.response_type_name)
    }
}

/// Python wrapper for service server
#[pyclass(name = "ZServer")]
pub struct PyZServer {
    inner: std::sync::Mutex<Box<dyn RawServer>>,
    request_type_name: String,
    response_type_name: String,
}

impl PyZServer {
    pub fn new(inner: Box<dyn RawServer>, service_type: String) -> Self {
        // Convert service type to Request/Response type names
        // e.g., "example_interfaces/srv/AddTwoInts" -> "example_interfaces/srv/AddTwoInts_Request"
        let request_type_name = format!("{}_Request", service_type);
        let response_type_name = format!("{}_Response", service_type);
        Self {
            inner: std::sync::Mutex::new(inner),
            request_type_name,
            response_type_name,
        }
    }
}

#[allow(unsafe_op_in_unsafe_fn)]
#[pymethods]
impl PyZServer {
    /// Receive the next service request (blocking)
    ///
    /// Returns:
    ///     Tuple of (request_id, request_message)
    unsafe fn take_request(&self, py: Python) -> PyResult<(PyObject, PyObject)> {
        // Release GIL during blocking operation to allow other Python threads to run
        let result = py.allow_threads(|| {
            let inner = self.inner.lock()
                .map_err(|e| anyhow::anyhow!("Lock error: {}", e))?;
            inner.take_request_serialized()
        });

        let (key, cdr_bytes) = result
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;

        // Deserialize CDR bytes to Python object using the registry (Request type)
        let obj = ros_z_msgs::deserialize_from_cdr(&self.request_type_name, py, &cdr_bytes)?;

        // Convert QueryKey to Python dict
        let request_id = PyDict::new_bound(py);
        request_id.set_item("sn", key.sn)?;
        request_id.set_item("gid", key.gid.to_vec())?;

        Ok((request_id.into(), obj))
    }

    /// Send a response to a service request
    ///
    /// Args:
    ///     response: Response message (msgspec.Struct)
    ///     request_id: Request ID from take_request()
    unsafe fn send_response(&self, py: Python, response: &Bound<'_, PyAny>, request_id: &Bound<'_, PyDict>) -> PyResult<()> {
        // Serialize Python message to CDR bytes using the registry (Response type)
        let cdr_bytes = ros_z_msgs::serialize_to_cdr(&self.response_type_name, response.py(), response)?;

        // Extract QueryKey from request_id dict
        let sn: i64 = request_id.get_item("sn")?.unwrap().extract()?;
        let gid_vec: Vec<u8> = request_id.get_item("gid")?.unwrap().extract()?;
        let mut gid = [0u8; 16];
        gid.copy_from_slice(&gid_vec[..16]);
        let key = QueryKey { sn, gid };

        // Release GIL during blocking operation
        py.allow_threads(|| {
            let inner = self.inner.lock()
                .map_err(|e| anyhow::anyhow!("Lock error: {}", e))?;
            inner.send_response_serialized(&cdr_bytes, &key)
        }).map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Get the service type name (for debugging)
    unsafe fn get_type_name(&self) -> String {
        format!("request={}, response={}", self.request_type_name, self.response_type_name)
    }
}
