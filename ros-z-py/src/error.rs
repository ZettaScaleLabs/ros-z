#![allow(unexpected_cfgs)]

use pyo3::prelude::*;

// Custom exception types
pyo3::create_exception!(ros_z_py, RosZError, pyo3::exceptions::PyException);
pyo3::create_exception!(ros_z_py, TimeoutError, RosZError);
pyo3::create_exception!(ros_z_py, SerializationError, RosZError);
pyo3::create_exception!(ros_z_py, TypeMismatchError, RosZError);

/// Trait for converting Rust errors to Python exceptions
pub(crate) trait IntoPyErr {
    fn into_pyerr(self) -> PyErr;
}

impl IntoPyErr for anyhow::Error {
    fn into_pyerr(self) -> PyErr {
        RosZError::new_err(format!("{:#}", self))
    }
}

impl IntoPyErr for zenoh::Error {
    fn into_pyerr(self) -> PyErr {
        RosZError::new_err(format!("Zenoh error: {}", self))
    }
}

impl<T> IntoPyErr for Result<T, anyhow::Error> {
    fn into_pyerr(self) -> PyErr {
        match self {
            Ok(_) => panic!("Tried to convert Ok to error"),
            Err(e) => e.into_pyerr(),
        }
    }
}
