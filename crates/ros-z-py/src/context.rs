use crate::error::IntoPyErr;
use crate::node::PyZNodeBuilder;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyTuple};
use ros_z::Builder;
use ros_z::context::{ZContext, ZContextBuilder};
use std::sync::Arc;

/// Read `ZENOH_SHM_ALLOC_SIZE` env var (bytes). Mirrors rmw_zenoh_cpp behavior.
fn read_shm_alloc_size() -> Option<usize> {
    std::env::var("ZENOH_SHM_ALLOC_SIZE")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .filter(|&n| n > 0)
}

/// Read `ZENOH_SHM_MESSAGE_SIZE_THRESHOLD` env var (bytes). Must be a power of two.
/// Mirrors rmw_zenoh_cpp behavior.
fn read_shm_threshold() -> Option<usize> {
    std::env::var("ZENOH_SHM_MESSAGE_SIZE_THRESHOLD")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .filter(|&n| n > 0 && n.is_power_of_two())
}

#[pyclass(name = "ZContextBuilder")]
#[derive(Default)]
pub struct PyZContextBuilder {
    builder: ZContextBuilder,
}

#[pymethods]
impl PyZContextBuilder {
    #[new]
    fn new() -> Self {
        Self::default()
    }

    /// Set the ROS domain ID
    pub fn with_domain_id(mut slf: PyRefMut<'_, Self>, domain_id: usize) -> PyRefMut<'_, Self> {
        slf.builder = std::mem::take(&mut slf.builder).with_domain_id(domain_id);
        slf
    }

    /// Enable Zenoh logging initialization with default level "error"
    pub fn with_logging_enabled(mut slf: PyRefMut<'_, Self>) -> PyRefMut<'_, Self> {
        slf.builder = std::mem::take(&mut slf.builder).with_logging_enabled();
        slf
    }

    /// Connect to specific Zenoh endpoints (e.g., "tcp/127.0.0.1:7447")
    pub fn with_connect_endpoints(
        mut slf: PyRefMut<'_, Self>,
        endpoints: Vec<String>,
    ) -> PyRefMut<'_, Self> {
        slf.builder = std::mem::take(&mut slf.builder).with_connect_endpoints(endpoints);
        slf
    }

    /// Disable multicast scouting (useful for isolated tests)
    pub fn disable_multicast_scouting(mut slf: PyRefMut<'_, Self>) -> PyRefMut<'_, Self> {
        slf.builder = std::mem::take(&mut slf.builder).disable_multicast_scouting();
        slf
    }

    /// Enable Zenoh shared memory transport with default pool size (48 MiB).
    ///
    /// SHM transparently accelerates large message transfers between nodes on the same
    /// host. Both publisher and subscriber must have SHM enabled. Also enable SHM on
    /// the router if messages must be routed.
    ///
    /// Pool size and threshold can be overridden via env vars ``ZENOH_SHM_ALLOC_SIZE``
    /// and ``ZENOH_SHM_MESSAGE_SIZE_THRESHOLD`` (must be power of two), matching
    /// rmw_zenoh_cpp behavior.
    pub fn with_shm_enabled(mut slf: PyRefMut<'_, Self>) -> PyResult<PyRefMut<'_, Self>> {
        let pool_size = read_shm_alloc_size();
        let threshold = read_shm_threshold();

        let builder = std::mem::take(&mut slf.builder);
        let builder = match pool_size {
            Some(size) => builder
                .with_shm_pool_size(size)
                .map_err(|e| e.into_pyerr())?,
            None => builder.with_shm_enabled().map_err(|e| e.into_pyerr())?,
        };
        let builder = match threshold {
            Some(t) => builder.with_shm_threshold(t),
            None => builder,
        };
        slf.builder = builder;
        Ok(slf)
    }

    /// Enable Zenoh shared memory transport with a custom pool size in bytes.
    ///
    /// ``ZENOH_SHM_MESSAGE_SIZE_THRESHOLD`` env var still applies if set.
    pub fn with_shm_pool_size(
        mut slf: PyRefMut<'_, Self>,
        size_bytes: usize,
    ) -> PyResult<PyRefMut<'_, Self>> {
        let threshold = read_shm_threshold();
        let builder = std::mem::take(&mut slf.builder)
            .with_shm_pool_size(size_bytes)
            .map_err(|e| e.into_pyerr())?;
        slf.builder = match threshold {
            Some(t) => builder.with_shm_threshold(t),
            None => builder,
        };
        Ok(slf)
    }

    /// Set the minimum message size (bytes) for SHM transport.
    ///
    /// Messages smaller than this threshold are sent via network. Only effective
    /// after ``with_shm_enabled()`` or ``with_shm_pool_size()``.
    pub fn with_shm_threshold(mut slf: PyRefMut<'_, Self>, threshold: usize) -> PyRefMut<'_, Self> {
        slf.builder = std::mem::take(&mut slf.builder).with_shm_threshold(threshold);
        slf
    }

    /// Build the context
    pub fn build(&mut self) -> PyResult<PyZContext> {
        let builder = std::mem::take(&mut self.builder);
        let ctx = builder.build().map_err(|e| e.into_pyerr())?;
        Ok(PyZContext { ctx: Arc::new(ctx) })
    }
}

#[pyclass(name = "ZContext")]
pub struct PyZContext {
    pub(crate) ctx: Arc<ZContext>,
}

#[allow(unsafe_op_in_unsafe_fn)]
#[pymethods]
impl PyZContext {
    fn __enter__<'a, 'py>(this: &'a Bound<'py, Self>) -> &'a Bound<'py, Self> {
        this
    }

    #[pyo3(signature = (*_args, **_kwargs))]
    fn __exit__(
        &mut self,
        _py: Python,
        _args: &Bound<PyTuple>,
        _kwargs: Option<&Bound<PyDict>>,
    ) -> PyResult<()> {
        self.close()
    }

    fn close(&self) -> PyResult<()> {
        // Zenoh sessions are ref-counted via Arc, no explicit close needed
        // The session will be closed when the last reference is dropped
        Ok(())
    }

    /// Create a node builder
    pub fn create_node(&self, name: String) -> PyZNodeBuilder {
        PyZNodeBuilder {
            ctx: self.ctx.clone(),
            name,
            namespace: None,
        }
    }
}
