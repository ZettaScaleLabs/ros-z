use crate::error::IntoPyErr;
use crate::payload_view::ZPayloadView;
use crate::traits::{RawPublisher, RawSubscriber};
use pyo3::prelude::*;
use pyo3::types::PyBytes;
use std::time::Duration;
use zenoh_buffers::buffer::SplitBuffer;

#[pyclass(name = "ZPublisher")]
pub struct PyZPublisher {
    inner: Box<dyn RawPublisher>,
    type_name: String,
}

impl PyZPublisher {
    pub fn new(inner: Box<dyn RawPublisher>, type_name: String) -> Self {
        Self { inner, type_name }
    }
}

#[allow(unsafe_op_in_unsafe_fn)]
#[pymethods]
impl PyZPublisher {
    /// Publish a message
    ///
    /// Serializes the Python message (msgspec.Struct) to ZBuf and publishes (zero-copy path)
    unsafe fn publish(&self, _py: Python, data: &Bound<'_, PyAny>) -> PyResult<()> {
        // Serialize Python message directly to ZBuf (zero-copy)
        let zbuf = ros_z_msgs::serialize_to_zbuf(&self.type_name, data)?;

        // Publish the ZBuf directly
        self.inner.publish(zbuf.into()).map_err(|e| e.into_pyerr())
    }

    /// Publish pre-serialized CDR bytes directly
    ///
    /// Use this for zero-copy forwarding of received messages (e.g., in a pong responder).
    /// The bytes should be in CDR format (as returned by recv_serialized/try_recv_serialized).
    fn publish_raw(&self, data: &[u8]) -> PyResult<()> {
        self.inner.publish(data.into()).map_err(|e| e.into_pyerr())
    }

    /// Get the topic name (for debugging)
    unsafe fn get_type_name(&self) -> String {
        self.type_name.clone()
    }
}

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
        let timeout_duration = timeout.map(Duration::from_secs_f64);

        // Release GIL while waiting to allow other Python threads to run
        let result = py.allow_threads(|| self.inner.recv_sample(timeout_duration));

        match result {
            Ok(sample) => {
                // Convert payload to ZBuf for zero-copy deserialization (cheap Arc clones)
                let payload_zbuf: zenoh_buffers::ZBuf = sample.payload().clone().into();
                // Set source for zero-copy sub-ZSlice creation during deserialization
                ros_z_cdr::ZBUF_DESER_SOURCE.with(|cell| {
                    *cell.borrow_mut() = Some(payload_zbuf.clone());
                });
                let cow = payload_zbuf.contiguous();
                let result = ros_z_msgs::deserialize_from_cdr(&self.type_name, py, &cow);
                ros_z_cdr::ZBUF_DESER_SOURCE.with(|cell| {
                    *cell.borrow_mut() = None;
                });
                Ok(Some(result?))
            }
            Err(e) => {
                // Check if it's a timeout error
                let err_str = e.to_string();
                if err_str.contains("timeout")
                    || err_str.contains("Timeout")
                    || err_str.contains("timed out")
                {
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
        match self.inner.try_recv_sample().map_err(|e| e.into_pyerr())? {
            Some(sample) => {
                let payload_zbuf: zenoh_buffers::ZBuf = sample.payload().clone().into();
                ros_z_cdr::ZBUF_DESER_SOURCE.with(|cell| {
                    *cell.borrow_mut() = Some(payload_zbuf.clone());
                });
                let cow = payload_zbuf.contiguous();
                let result = ros_z_msgs::deserialize_from_cdr(&self.type_name, py, &cow);
                ros_z_cdr::ZBUF_DESER_SOURCE.with(|cell| {
                    *cell.borrow_mut() = None;
                });
                Ok(Some(result?))
            }
            None => Ok(None),
        }
    }

    /// Receive serialized bytes (for testing/advanced use)
    ///
    /// Returns bytes object containing the CDR-serialized message.
    #[pyo3(signature = (timeout=None))]
    unsafe fn recv_serialized(
        &self,
        py: Python,
        timeout: Option<f64>,
    ) -> PyResult<Option<Py<PyBytes>>> {
        let timeout_duration = timeout.map(Duration::from_secs_f64);

        // Release GIL while waiting to allow other Python threads to run
        let result = py.allow_threads(|| self.inner.recv_serialized(timeout_duration));

        match result {
            Ok(data) => Ok(Some(PyBytes::new_bound(py, &data).into())),
            Err(e) => {
                // Check if it's a timeout error
                let err_str = e.to_string();
                if err_str.contains("timeout")
                    || err_str.contains("Timeout")
                    || err_str.contains("timed out")
                {
                    Ok(None)
                } else {
                    Err(e.into_pyerr())
                }
            }
        }
    }

    /// Try to receive serialized bytes without blocking
    ///
    /// Returns bytes object containing the CDR-serialized message, or None.
    unsafe fn try_recv_serialized(&self, py: Python) -> PyResult<Option<Py<PyBytes>>> {
        match self
            .inner
            .try_recv_serialized()
            .map_err(|e| e.into_pyerr())?
        {
            Some(data) => Ok(Some(PyBytes::new_bound(py, &data).into())),
            None => Ok(None),
        }
    }

    /// Receive raw payload as a zero-copy buffer view (blocking)
    ///
    /// Returns a ZPayloadView that implements Python's buffer protocol.
    /// Use memoryview() or numpy.frombuffer() for zero-copy access:
    ///
    /// ```python
    /// payload = subscriber.recv_raw_view()
    /// mv = memoryview(payload)  # Zero-copy view!
    /// # Or with numpy:
    /// arr = np.frombuffer(payload, dtype=np.uint8)
    /// ```
    ///
    /// Args:
    ///     timeout: Optional timeout in seconds (None = block forever)
    ///
    /// Returns:
    ///     ZPayloadView, or None if timeout occurred
    #[pyo3(signature = (timeout=None))]
    unsafe fn recv_raw_view(&self, py: Python, timeout: Option<f64>) -> PyResult<Option<PyObject>> {
        let timeout_duration = timeout.map(Duration::from_secs_f64);

        // Release GIL while waiting to allow other Python threads to run
        let result = py.allow_threads(|| self.inner.recv_sample(timeout_duration));

        match result {
            Ok(sample) => {
                let view = ZPayloadView::new(sample);
                Ok(Some(Py::new(py, view)?.into_any()))
            }
            Err(e) => {
                let err_str = e.to_string();
                if err_str.contains("timeout")
                    || err_str.contains("Timeout")
                    || err_str.contains("timed out")
                {
                    Ok(None)
                } else {
                    Err(e.into_pyerr())
                }
            }
        }
    }

    /// Try to receive raw payload as a zero-copy buffer view without blocking
    ///
    /// Returns:
    ///     ZPayloadView, or None if no message available
    unsafe fn try_recv_raw_view(&self, py: Python) -> PyResult<Option<PyObject>> {
        match self.inner.try_recv_sample().map_err(|e| e.into_pyerr())? {
            Some(sample) => {
                let view = ZPayloadView::new(sample);
                Ok(Some(Py::new(py, view)?.into_any()))
            }
            None => Ok(None),
        }
    }

    /// Get the type name (for debugging)
    unsafe fn get_type_name(&self) -> String {
        self.type_name.clone()
    }
}
