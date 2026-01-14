use pyo3::prelude::*;

/// Trait for converting Python objects to Rust types
pub(crate) trait IntoRust: 'static {
    type Into;
    fn into_rust(self) -> Self::Into;
}

/// Trait for converting Rust types to Python objects
pub(crate) trait IntoPython: Send + Sync + 'static {
    type Into: ToPyObject;
    fn into_python(self) -> Self::Into;

    fn into_pyobject(self, py: Python) -> PyObject
    where
        Self: Sized,
    {
        self.into_python().to_object(py)
    }
}
