use crate::error::IntoPyErr;
use crate::node::PyZNodeBuilder;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyTuple};
use ros_z::Builder;
use ros_z::context::{ZContext, ZContextBuilder};
use std::sync::Arc;

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

    /// Set Zenoh mode: "peer", "client", or "router"
    pub fn with_mode(mut slf: PyRefMut<'_, Self>, mode: String) -> PyRefMut<'_, Self> {
        slf.builder = std::mem::take(&mut slf.builder).with_mode(mode);
        slf
    }

    /// Connect to a router at the given endpoint (e.g., "tcp/192.168.1.1:7447")
    pub fn with_router_endpoint(
        mut slf: PyRefMut<'_, Self>,
        endpoint: String,
    ) -> PyResult<PyRefMut<'_, Self>> {
        slf.builder = std::mem::take(&mut slf.builder)
            .with_router_endpoint(endpoint)
            .map_err(|e| e.into_pyerr())?;
        Ok(slf)
    }

    /// Load Zenoh configuration from a file
    pub fn with_config_file(mut slf: PyRefMut<'_, Self>, path: String) -> PyRefMut<'_, Self> {
        slf.builder =
            std::mem::take(&mut slf.builder).with_config_file(std::path::PathBuf::from(path));
        slf
    }

    /// Add a JSON config override (e.g., key="transport/link/tx/sequence_number_resolution", value="256")
    pub fn with_json(
        mut slf: PyRefMut<'_, Self>,
        key: String,
        value: String,
    ) -> PyRefMut<'_, Self> {
        slf.builder = std::mem::take(&mut slf.builder).with_json(key, value);
        slf
    }

    /// Add a name remap rule (format: "from:=to")
    pub fn with_remap_rule(
        mut slf: PyRefMut<'_, Self>,
        rule: String,
    ) -> PyResult<PyRefMut<'_, Self>> {
        slf.builder = std::mem::take(&mut slf.builder)
            .with_remap_rule(rule)
            .map_err(|e| e.into_pyerr())?;
        Ok(slf)
    }

    /// Add multiple name remap rules (format: ["from1:=to1", "from2:=to2"])
    pub fn with_remap_rules(
        mut slf: PyRefMut<'_, Self>,
        rules: Vec<String>,
    ) -> PyResult<PyRefMut<'_, Self>> {
        slf.builder = std::mem::take(&mut slf.builder)
            .with_remap_rules(rules)
            .map_err(|e| e.into_pyerr())?;
        Ok(slf)
    }

    /// Set the security enclave name
    pub fn with_enclave(mut slf: PyRefMut<'_, Self>, enclave: String) -> PyRefMut<'_, Self> {
        slf.builder = std::mem::take(&mut slf.builder).with_enclave(enclave);
        slf
    }

    /// Connect to a local zenohd router (equivalent to with_connect_endpoints(["tcp/localhost:7447"]))
    pub fn connect_to_local_zenohd(mut slf: PyRefMut<'_, Self>) -> PyRefMut<'_, Self> {
        slf.builder = std::mem::take(&mut slf.builder).connect_to_local_zenohd();
        slf
    }

    /// Enable shared memory (SHM) with a default 10MB pool.
    ///
    /// Messages larger than the SHM threshold are sent via shared memory
    /// instead of Zenoh's default transport, reducing copy overhead.
    pub fn with_shm_enabled(mut slf: PyRefMut<'_, Self>) -> PyResult<PyRefMut<'_, Self>> {
        slf.builder = std::mem::take(&mut slf.builder)
            .with_shm_enabled()
            .map_err(|e| e.into_pyerr())?;
        Ok(slf)
    }

    /// Enable SHM with a custom pool size in bytes.
    ///
    /// Example: `builder.with_shm_pool_size(100 * 1024 * 1024)` for 100MB.
    pub fn with_shm_pool_size(
        mut slf: PyRefMut<'_, Self>,
        size_bytes: usize,
    ) -> PyResult<PyRefMut<'_, Self>> {
        slf.builder = std::mem::take(&mut slf.builder)
            .with_shm_pool_size(size_bytes)
            .map_err(|e| e.into_pyerr())?;
        Ok(slf)
    }

    /// Set the minimum message size (bytes) to use SHM transport.
    ///
    /// Messages smaller than this threshold are sent via the default transport.
    /// Only effective after `with_shm_enabled()` or `with_shm_pool_size()`.
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
        self.shutdown()
    }

    /// Shutdown the context and release all resources
    pub fn shutdown(&self) -> PyResult<()> {
        self.ctx.shutdown().map_err(|e| e.into_pyerr())
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
