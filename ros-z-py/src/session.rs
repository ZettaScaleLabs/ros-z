use pyo3::prelude::*;
use pyo3::types::{PyDict, PyTuple};
use std::sync::Arc;
use ros_z::context::ZContext;
use crate::error::IntoPyErr;

#[pyclass]
pub struct PySession {
    pub(crate) ctx: Arc<ZContext>,
}

#[pymethods]
impl PySession {
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
}

/// Open a Zenoh session for ROS 2 communication
#[pyfunction]
#[pyo3(signature = (config=None, domain_id=0))]
pub fn open_session(config: Option<&Bound<'_, PyDict>>, domain_id: usize) -> PyResult<PySession> {
    use ros_z::{Builder, context::ZContextBuilder};

    // Create context builder
    let mut builder = ZContextBuilder::default().with_domain_id(domain_id);

    // Parse Zenoh configuration from Python dict if provided
    if let Some(_cfg_dict) = config {
        // TODO: Parse config dict and apply to builder
        // For now, just use defaults
    }

    // Build the context
    let ctx = builder.build().map_err(|e| e.into_pyerr())?;

    Ok(PySession {
        ctx: Arc::new(ctx),
    })
}
