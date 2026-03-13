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

    /// Publish a CUDA ZBuf directly (zero-copy GPU transport).
    ///
    /// Use this to send a CUDA device buffer via IPC without any CPU copies.
    /// The subscriber receives a `ZPayloadView` with `is_cuda=True` and can
    /// extract the GPU tensor via `as_dlpack()`.
    ///
    /// Args:
    ///     zbuf: A `CudaZBuf` returned by `PyCudaBuf.into_zbuf()`.
    ///
    /// Example (Python):
    ///     buf = PyCudaBuf.alloc_device(4 * 1024 * 1024, device_id=0)
    ///     # ... fill via cupy/torch ...
    ///     publisher.publish_zbuf(buf.into_zbuf())
    #[cfg(feature = "cuda")]
    fn publish_zbuf(&self, zbuf: &crate::cuda_buf::ZBufWrapper) -> PyResult<()> {
        use zenoh_buffers::ZBuf as ZenohZBuf;
        let zenoh_zbuf: ZenohZBuf = zbuf.zbuf.clone().into_inner();
        self.inner
            .publish(zenoh_zbuf.into())
            .map_err(|e| e.into_pyerr())
    }

    /// Publish raw bytes directly (no CDR framing).
    ///
    /// Used internally by `publish_tensor` for the CPU tensor path (numpy NPY format).
    /// Can also be used for arbitrary byte payloads.
    fn publish_bytes(&self, data: &[u8]) -> PyResult<()> {
        self.inner.publish(data.into()).map_err(|e| e.into_pyerr())
    }

    /// Publish a tensor (torch or numpy) with zero boilerplate.
    ///
    /// Dispatches automatically based on device:
    ///
    /// - **CUDA tensor** (`tensor.device.type == "cuda"`): wraps via
    ///   `PyCudaBuf.from_torch()` and sends as a CUDA IPC payload (zero GPU copies).
    ///   The subscriber receives `is_cuda=True` and can call `as_torch()`.
    ///
    /// - **CPU tensor** (`tensor.device.type == "cpu"` or numpy array): serializes
    ///   using numpy's NPY format (shape + dtype preserved). The subscriber calls
    ///   `as_torch()` or `as_numpy()` to reconstruct.
    ///
    /// **Lifetime note for CUDA tensors:** the source `tensor` must remain live
    /// until the subscriber calls `cudaIpcOpenMemHandle`. This method keeps `tensor`
    /// alive until `publish_zbuf` returns (IPC handle serialized on wire), but the
    /// subscriber process still needs the source allocation to exist when it opens
    /// the handle. For best results use a short `time.sleep` after publish, or
    /// keep `tensor` in scope for the expected round-trip window.
    ///
    /// Args:
    ///     tensor: A `torch.Tensor` (any device) or `numpy.ndarray`.
    ///
    /// Example:
    ///     # CUDA:
    ///     pub.publish_tensor(torch.zeros(480, 640, 3, dtype=torch.float16, device="cuda"))
    ///     # CPU:
    ///     pub.publish_tensor(torch.arange(1024, dtype=torch.float32))
    ///     pub.publish_tensor(np.zeros((256, 256), dtype=np.uint8))
    #[cfg(feature = "cuda")]
    fn publish_tensor(&self, py: Python<'_>, tensor: &Bound<'_, PyAny>) -> PyResult<()> {
        // Detect device type — works for torch.Tensor (has .device) and np.ndarray (no .device)
        let device_type: Option<String> = tensor
            .getattr("device")
            .ok()
            .and_then(|d| d.getattr("type").ok())
            .and_then(|t| t.extract().ok());

        match device_type.as_deref() {
            Some("cuda") => {
                // CUDA zero-copy path
                let mut buf = crate::cuda_buf::PyCudaBuf::from_torch(tensor)?;
                // Pass tensor as keepalive so GC cannot collect it before publish_zbuf returns
                let keepalive = Some(tensor.clone().into());
                let zbuf = buf.into_zbuf(keepalive)?;
                let zenoh_zbuf: zenoh_buffers::ZBuf = zbuf.zbuf.clone().into_inner();
                self.inner
                    .publish(zenoh_zbuf.into())
                    .map_err(|e| e.into_pyerr())
            }
            _ => {
                // CPU path: numpy ndarray or cpu torch.Tensor
                // Convert torch.Tensor → numpy if needed
                let np = py.import_bound("numpy")?;
                let arr = if tensor.hasattr("numpy")? {
                    // torch.Tensor: call .detach().cpu().numpy()
                    tensor
                        .call_method0("detach")?
                        .call_method0("cpu")?
                        .call_method0("numpy")?
                } else {
                    // Already numpy-like: wrap with np.asarray for uniform handling
                    np.call_method1("asarray", (tensor,))?
                };
                // Serialize as numpy NPY format (contains shape + dtype + strides)
                let io = py.import_bound("io")?;
                let buf = io.call_method0("BytesIO")?;
                np.call_method1("save", (&buf, arr))?;
                let payload: Vec<u8> = buf.call_method0("getvalue")?.extract()?;
                self.publish_bytes(&payload)
            }
        }
    }

    /// Get the topic name (for debugging)
    unsafe fn get_type_name(&self) -> String {
        self.type_name.clone()
    }
}

#[pyclass(name = "ZSubscriber")]
pub struct PyZSubscriber {
    /// None for callback-based subscriptions (no queue)
    inner: Option<Box<dyn RawSubscriber>>,
    type_name: String,
}

impl PyZSubscriber {
    pub fn new(inner: Box<dyn RawSubscriber>, type_name: String) -> Self {
        Self {
            inner: Some(inner),
            type_name,
        }
    }

    /// Create a callback-based subscriber (no queue, no recv methods)
    pub fn new_callback(type_name: String) -> Self {
        Self {
            inner: None,
            type_name,
        }
    }

    fn require_queue(&self) -> PyResult<&dyn RawSubscriber> {
        self.inner.as_deref().ok_or_else(|| {
            pyo3::exceptions::PyRuntimeError::new_err(
                "Cannot recv on a callback-based subscriber. Messages are delivered via the callback.",
            )
        })
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
        let inner = self.require_queue()?;
        let timeout_duration = timeout.map(Duration::from_secs_f64);

        // Release GIL while waiting to allow other Python threads to run
        let result = py.allow_threads(|| inner.recv_sample(timeout_duration));

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
        let inner = self.require_queue()?;
        match inner.try_recv_sample().map_err(|e| e.into_pyerr())? {
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
        let inner = self.require_queue()?;
        let timeout_duration = timeout.map(Duration::from_secs_f64);

        // Release GIL while waiting to allow other Python threads to run
        let result = py.allow_threads(|| inner.recv_serialized(timeout_duration));

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
        let inner = self.require_queue()?;
        match inner.try_recv_serialized().map_err(|e| e.into_pyerr())? {
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
        let inner = self.require_queue()?;
        let timeout_duration = timeout.map(Duration::from_secs_f64);

        // Release GIL while waiting to allow other Python threads to run
        let result = py.allow_threads(|| inner.recv_sample(timeout_duration));

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
        let inner = self.require_queue()?;
        match inner.try_recv_sample().map_err(|e| e.into_pyerr())? {
            Some(sample) => {
                let view = ZPayloadView::new(sample);
                Ok(Some(Py::new(py, view)?.into_any()))
            }
            None => Ok(None),
        }
    }

    /// Whether this is a callback-based subscriber (no recv methods)
    #[getter]
    fn is_callback(&self) -> bool {
        self.inner.is_none()
    }

    /// Get the type name (for debugging)
    unsafe fn get_type_name(&self) -> String {
        self.type_name.clone()
    }
}
