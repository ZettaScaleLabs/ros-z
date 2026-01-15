use pyo3::prelude::*;
use crate::traits::RawSubscriber;
use crate::error::IntoPyErr;
use std::time::Duration;

#[pyclass(name = "ZSubscriber")]
pub struct PyZSubscriber {
    inner: Box<dyn RawSubscriber>,
    type_name: String,
}

impl PyZSubscriber {
    pub fn new(inner: Box<dyn RawSubscriber>, type_name: String) -> Self {
        Self { inner, type_name }
    }
}

#[allow(unsafe_op_in_unsafe_fn)]
#[pymethods]
impl PyZSubscriber {
    /// Receive the next message (blocking)
    ///
    /// Args:
    ///     timeout: Optional timeout in seconds (None = block forever)
    ///
    /// Returns:
    ///     Message as dict, or None if timeout occurred
    #[pyo3(signature = (timeout=None))]
    unsafe fn recv(&self, py: Python, timeout: Option<f64>) -> PyResult<Option<PyObject>> {
        let timeout_duration = timeout.map(|s| Duration::from_secs_f64(s));

        match self.inner.recv_serialized(timeout_duration) {
            Ok(cdr_bytes) => {
                // Deserialize CDR bytes to Python dict using the registry
                let obj = ros_z_msgs::deserialize_from_cdr(&self.type_name, py, &cdr_bytes)?;
                Ok(Some(obj))
            }
            Err(e) => {
                // Check if it's a timeout error
                let err_str = e.to_string();
                if err_str.contains("timeout") || err_str.contains("Timeout") {
                    Ok(None)
                } else {
                    Err(e.into_pyerr())
                }
            }
        }
    }

    /// Try to receive a message without blocking
    ///
    /// Returns:
    ///     Message as dict, or None if no message available
    unsafe fn try_recv(&self, py: Python) -> PyResult<Option<PyObject>> {
        match self.inner.try_recv_serialized().map_err(|e| e.into_pyerr())? {
            Some(cdr_bytes) => {
                // Deserialize CDR bytes to Python dict using the registry
                let obj = ros_z_msgs::deserialize_from_cdr(&self.type_name, py, &cdr_bytes)?;
                Ok(Some(obj))
            }
            None => Ok(None),
        }
    }

    /// Receive raw serialized bytes (for testing/advanced use)
    #[pyo3(signature = (timeout=None))]
    unsafe fn recv_raw(&self, timeout: Option<f64>) -> PyResult<Option<Vec<u8>>> {
        let timeout_duration = timeout.map(|s| Duration::from_secs_f64(s));

        match self.inner.recv_serialized(timeout_duration) {
            Ok(data) => Ok(Some(data)),
            Err(e) => {
                // Check if it's a timeout error
                let err_str = e.to_string();
                if err_str.contains("timeout") || err_str.contains("Timeout") {
                    Ok(None)
                } else {
                    Err(e.into_pyerr())
                }
            }
        }
    }

    /// Try to receive raw serialized bytes without blocking
    unsafe fn try_recv_raw(&self) -> PyResult<Option<Vec<u8>>> {
        self.inner
            .try_recv_serialized()
            .map_err(|e| e.into_pyerr())
    }

    /// Get the type name (for debugging)
    unsafe fn get_type_name(&self) -> String {
        self.type_name.clone()
    }
}
