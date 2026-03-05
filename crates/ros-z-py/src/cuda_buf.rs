//! Python binding for CUDA device memory allocation.
//!
//! Exposes [`PyCudaBuf`]: a pyclass wrapping [`CudaBufInner`] that lets Python
//! code allocate GPU buffers, fill them via cupy/torch, and publish them as
//! CUDA ZBuf payloads with zero CPU copies.

use pyo3::prelude::*;
use ros_z::{CudaBufInner, ZBuf};

/// A CUDA-backed buffer that can be published as a zero-copy GPU payload.
///
/// Allocate a buffer, fill it via cupy or torch, then pass it to
/// `ZBuf.from_cuda()` (or use `.into_zbuf()`) to build a message payload.
///
/// # Example
///
/// ```python
/// import cupy as cp
/// from ros_z_py import PyCudaBuf
///
/// # Allocate 4 MB on GPU 0
/// buf = PyCudaBuf.alloc_device(4 * 1024 * 1024, device_id=0)
///
/// # Fill via cupy (zero-copy view of the same GPU pointer)
/// tensor = cp.ndarray((buf.cuda_len,), dtype=cp.uint8,
///                     memptr=cp.cuda.MemoryPointer(
///                         cp.cuda.UnownedMemory(buf.device_ptr, buf.cuda_len, buf),
///                         0))
/// tensor[:] = cp.arange(buf.cuda_len, dtype=cp.uint8)
///
/// # Wrap in a ZBuf and publish
/// zbuf = buf.into_zbuf()
/// msg.data = zbuf
/// publisher.publish(msg)
/// ```
#[pyclass(name = "PyCudaBuf")]
pub struct PyCudaBuf {
    pub inner: Option<CudaBufInner>,
}

#[pymethods]
impl PyCudaBuf {
    /// Allocate device-only CUDA memory (cudaMalloc).
    ///
    /// Device memory is GPU-only: you cannot read or write it from CPU.
    /// Use a CUDA kernel, cupy, or torch to fill the buffer.
    /// An IPC handle is captured at allocation time for zero-copy IPC transport.
    ///
    /// Args:
    ///     len: Number of bytes to allocate.
    ///     device_id: CUDA device ordinal (0-indexed).
    #[staticmethod]
    fn alloc_device(len: usize, device_id: i32) -> PyResult<Self> {
        let inner = CudaBufInner::alloc_device(len, device_id)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(Self { inner: Some(inner) })
    }

    /// Allocate pinned host memory (cudaMallocHost).
    ///
    /// Pinned memory is CPU-accessible and DMA-friendly for fast GPU transfers.
    /// Use this when you want to write from CPU and have the GPU read via DMA.
    ///
    /// Args:
    ///     len: Number of bytes to allocate.
    ///     device_id: CUDA device ordinal (0-indexed).
    #[staticmethod]
    fn alloc_pinned(len: usize, device_id: i32) -> PyResult<Self> {
        let inner = CudaBufInner::alloc_pinned(len, device_id)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(Self { inner: Some(inner) })
    }

    /// Raw device pointer as a Python integer (for cupy / torch interop).
    ///
    /// Pass this to `cp.cuda.UnownedMemory` or `torch.cuda.capi.THCPStream_New`
    /// to get a zero-copy view of the GPU buffer.
    ///
    /// Returns:
    ///     int: The raw CUDA device pointer cast to usize.
    #[getter]
    fn device_ptr(&self) -> PyResult<usize> {
        self.inner
            .as_ref()
            .map(|b| b.as_device_ptr() as usize)
            .ok_or_else(|| pyo3::exceptions::PyValueError::new_err("buffer already consumed"))
    }

    /// Length of the allocated buffer in bytes.
    #[getter]
    fn cuda_len(&self) -> PyResult<usize> {
        self.inner
            .as_ref()
            .map(|b| b.cuda_len)
            .ok_or_else(|| pyo3::exceptions::PyValueError::new_err("buffer already consumed"))
    }

    /// Consume this buffer and return a `ZBuf` with `kind = CudaPtr`.
    ///
    /// After calling this, the `PyCudaBuf` is no longer usable.
    /// Pass the returned `ZBuf` to a message's `data` field and publish.
    ///
    /// Returns:
    ///     ZBuf: A ZBuf wrapping the CUDA allocation, ready for publishing.
    fn into_zbuf(&mut self) -> PyResult<ZBufWrapper> {
        let inner = self
            .inner
            .take()
            .ok_or_else(|| pyo3::exceptions::PyValueError::new_err("buffer already consumed"))?;
        Ok(ZBufWrapper(ZBuf::from_cuda(inner)))
    }
}

/// Thin wrapper so Python can hold a `ZBuf` returned by `PyCudaBuf.into_zbuf()`.
///
/// The ros-z Python publisher accepts the inner ZBuf directly via the bridge.
/// Users typically pass `buf.into_zbuf()` straight to `msg.data = ...`.
#[pyclass(name = "CudaZBuf")]
pub struct ZBufWrapper(pub ZBuf);

#[pymethods]
impl ZBufWrapper {
    /// Number of bytes in the ZBuf (0 for device-only memory, as expected).
    fn __len__(&self) -> usize {
        use zenoh_buffers::buffer::Buffer;
        self.0.len()
    }

    fn __repr__(&self) -> String {
        format!("CudaZBuf(len={})", self.__len__())
    }
}
