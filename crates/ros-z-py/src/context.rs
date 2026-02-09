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
