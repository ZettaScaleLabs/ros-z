use pyo3::prelude::*;
use pyo3::types::PyDict;
use crate::traits::RawPublisher;
use crate::error::IntoPyErr;

#[pyclass]
pub struct PyPublisher {
    inner: Box<dyn RawPublisher>,
    type_name: String,
}

impl PyPublisher {
    pub fn new(inner: Box<dyn RawPublisher>, type_name: String) -> Self {
        Self { inner, type_name }
    }
}

#[pymethods]
impl PyPublisher {
    /// Publish a message
    ///
    /// Serializes the Python dict to CDR bytes using the message registry
    unsafe fn publish(&self, data: &Bound<'_, PyDict>) -> PyResult<()> {
        // Serialize Python dict to CDR bytes using the registry
        let cdr_bytes = ros_z_msgs::serialize_to_cdr(&self.type_name, data.py(), data)?;

        // Publish the serialized bytes
        self.inner
            .publish_serialized(&cdr_bytes)
            .map_err(|e| e.into_pyerr())
    }

    /// Publish raw serialized bytes (for testing/advanced use)
    unsafe fn publish_raw(&self, data: &[u8]) -> PyResult<()> {
        self.inner
            .publish_serialized(data)
            .map_err(|e| e.into_pyerr())
    }

    /// Get the topic name (for debugging)
    unsafe fn get_type_name(&self) -> String {
        self.type_name.clone()
    }
}
