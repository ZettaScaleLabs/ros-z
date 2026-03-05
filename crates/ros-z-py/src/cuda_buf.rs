//! Python binding for CUDA device memory allocation.
//!
//! Exposes [`PyCudaBuf`]: a pyclass wrapping [`CudaBufInner`] that lets Python
//! code allocate GPU buffers, fill them via cupy/torch, and publish them as
//! CUDA ZBuf payloads with zero CPU copies.

use pyo3::prelude::*;
use ros_z::{CudaBufInner, ZBuf};
use zenoh_cuda::TensorMeta;

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

    /// Wrap an externally-owned CUDA device pointer (e.g. from PyTorch).
    ///
    /// This does NOT allocate memory — it wraps `ptr` in a non-owning
    /// `CudaBufInner` that will NOT call `cudaFree` on drop.
    ///
    /// Use this to publish a torch tensor with zero copies:
    ///
    /// ```python
    /// import torch
    /// tensor = torch.zeros(1024 * 1024, dtype=torch.uint8, device="cuda:0")
    /// # tensor must stay alive until after publish_zbuf() returns
    /// buf = PyCudaBuf.from_device_ptr(tensor.data_ptr(), tensor.nbytes, device_id=0)
    /// publisher.publish_zbuf(buf.into_zbuf())
    /// # safe to let tensor go out of scope now
    /// ```
    ///
    /// Args:
    ///     ptr: Raw CUDA device pointer as integer (e.g. `tensor.data_ptr()`).
    ///     len: Number of bytes.
    ///     device_id: CUDA device ordinal (0-indexed).
    #[staticmethod]
    fn from_device_ptr(ptr: usize, len: usize, device_id: i32) -> PyResult<Self> {
        let inner = CudaBufInner::from_device_ptr_borrowed(ptr as *mut u8, len, device_id)
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

    /// Wrap an existing CUDA tensor (e.g. from PyTorch) with shape and dtype metadata.
    ///
    /// Captures `data_ptr()`, `nbytes`, `shape`, `dtype`, and `stride()` from
    /// the tensor. The resulting buffer does NOT own the allocation — the source
    /// tensor must remain alive until after `publish_zbuf()` returns.
    ///
    /// The returned `PyCudaBuf` wraps the tensor as a `CudaTensor` ZSlice, so
    /// `as_dlpack()` on the subscriber side returns the correct shape and dtype.
    ///
    /// Args:
    ///     tensor: A `torch.Tensor` on a CUDA device.
    ///
    /// Raises:
    ///     ValueError: If the tensor is not on a CUDA device.
    ///     RuntimeError: On CUDA IPC handle errors.
    #[staticmethod]
    pub fn from_torch(tensor: &Bound<'_, PyAny>) -> PyResult<Self> {
        let ptr: usize = tensor.getattr("data_ptr")?.call0()?.extract()?;
        let nbytes: usize = tensor.getattr("nbytes")?.extract()?;
        let device_type: String = tensor.getattr("device")?.getattr("type")?.extract()?;
        if device_type != "cuda" {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "tensor must be on a CUDA device",
            ));
        }
        let device_id: i32 = tensor
            .getattr("device")?
            .getattr("index")?
            .extract::<Option<i32>>()?
            .unwrap_or(0);
        let shape: Vec<i64> = tensor.getattr("shape")?.extract()?;
        let dtype_str: String = tensor.getattr("dtype")?.str()?.extract()?;
        let (dtype_code, dtype_bits, dtype_lanes) = parse_torch_dtype(&dtype_str)?;
        let strides: Vec<i64> = tensor.getattr("stride")?.call0()?.extract()?;
        let strides = if strides == c_contiguous_strides(&shape) {
            None
        } else {
            Some(strides)
        };
        let meta = TensorMeta {
            ndim: shape.len() as i32,
            shape,
            dtype_code,
            dtype_bits,
            dtype_lanes,
            byte_offset: 0,
            strides,
        };
        let inner = CudaBufInner::from_device_ptr_borrowed(ptr as *mut u8, nbytes, device_id)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?
            .with_tensor_meta(meta);
        Ok(Self { inner: Some(inner) })
    }

    /// Attach DLPack tensor metadata (shape and dtype) to this buffer.
    ///
    /// After calling this, `into_zbuf()` will produce a `CudaTensor` ZSlice
    /// and the subscriber can reconstruct the correct shape/dtype via `as_dlpack()`.
    ///
    /// Args:
    ///     shape: List of dimension sizes (e.g. `[480, 640, 3]`).
    ///     dtype: DType string: one of "float32", "float16", "bfloat16",
    ///            "float64", "int8", "int16", "int32", "int64",
    ///            "uint8", "uint16", "uint32", "uint64", "bool".
    ///
    /// Returns:
    ///     Self (for chaining).
    fn with_tensor_meta(&mut self, shape: Vec<i64>, dtype: &str) -> PyResult<()> {
        if self.inner.is_none() {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "buffer already consumed",
            ));
        }
        let (dtype_code, dtype_bits, dtype_lanes) = parse_torch_dtype(dtype)?;
        let meta = TensorMeta {
            ndim: shape.len() as i32,
            shape,
            dtype_code,
            dtype_bits,
            dtype_lanes,
            byte_offset: 0,
            strides: None,
        };
        let old = self.inner.take().unwrap();
        self.inner = Some(old.with_tensor_meta(meta));
        Ok(())
    }

    /// Consume this buffer and return a `CudaZBuf` ready for publishing.
    ///
    /// After calling this, the `PyCudaBuf` is no longer usable.
    ///
    /// **Lifetime contract:** For non-owning buffers (created via `from_torch` or
    /// `from_device_ptr`), the source allocation must remain alive until the
    /// subscriber has opened the IPC handle. Pass the source tensor as `keepalive`
    /// to prevent Python's GC from collecting it prematurely:
    ///
    /// ```python
    /// zbuf = buf.into_zbuf(keepalive=tensor)
    /// publisher.publish_zbuf(zbuf)
    /// # tensor is held alive until zbuf is dropped (after publish_zbuf returns)
    /// ```
    ///
    /// Note: even with keepalive, the publisher process should keep the allocation
    /// alive long enough for the subscriber to call `cudaIpcOpenMemHandle` (~RTT).
    /// `publish_tensor()` handles this automatically.
    ///
    /// Args:
    ///     keepalive: Optional Python object to keep alive (e.g. the source tensor).
    ///
    /// Returns:
    ///     CudaZBuf: Ready for `publisher.publish_zbuf()`.
    #[pyo3(signature = (keepalive=None))]
    #[allow(clippy::wrong_self_convention)] // &mut self required by #[pyclass]; inner is taken
    pub fn into_zbuf(&mut self, keepalive: Option<PyObject>) -> PyResult<ZBufWrapper> {
        let inner = self
            .inner
            .take()
            .ok_or_else(|| pyo3::exceptions::PyValueError::new_err("buffer already consumed"))?;
        let zbuf = if inner.tensor_meta().is_some() {
            ZBuf::from_cuda_tensor(inner)
        } else {
            ZBuf::from_cuda(inner)
        };
        Ok(ZBufWrapper { zbuf, keepalive })
    }
}

/// Map a torch dtype string to DLPack (dtype_code, dtype_bits, dtype_lanes).
///
/// dtype_code: 0=Int, 1=UInt, 2=Float, 4=BFloat16, 5=Complex
fn parse_torch_dtype(dtype: &str) -> PyResult<(u8, u8, u16)> {
    // torch repr is "torch.float32" or just "float32"
    let s = dtype.strip_prefix("torch.").unwrap_or(dtype);
    let result = match s {
        "float16" => (2u8, 16u8, 1u16),
        "float32" => (2, 32, 1),
        "float64" => (2, 64, 1),
        "bfloat16" => (4, 16, 1),
        "int8" => (0, 8, 1),
        "int16" => (0, 16, 1),
        "int32" => (0, 32, 1),
        "int64" => (0, 64, 1),
        "uint8" => (1, 8, 1),
        "uint16" => (1, 16, 1),
        "uint32" => (1, 32, 1),
        "uint64" => (1, 64, 1),
        "bool" => (1, 1, 1),
        other => {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "unsupported dtype '{}'; expected one of: float16, float32, float64, \
                 bfloat16, int8/16/32/64, uint8/16/32/64, bool",
                other
            )));
        }
    };
    Ok(result)
}

/// Compute C-contiguous strides for a given shape (element strides, not byte strides).
fn c_contiguous_strides(shape: &[i64]) -> Vec<i64> {
    let mut strides = vec![1i64; shape.len()];
    for i in (0..shape.len().saturating_sub(1)).rev() {
        strides[i] = strides[i + 1] * shape[i + 1];
    }
    strides
}

/// Wrapper returned by `PyCudaBuf.into_zbuf()`, passed to `publisher.publish_zbuf()`.
///
/// Holds an optional `keepalive` reference to prevent GC of the source tensor
/// while the ZBuf is in flight (see `into_zbuf(keepalive=)` docs).
#[pyclass(name = "CudaZBuf")]
pub struct ZBufWrapper {
    pub zbuf: ZBuf,
    /// Keeps the source Python object (e.g. torch.Tensor) alive until this
    /// CudaZBuf is dropped, preventing premature GC of the GPU allocation.
    #[allow(dead_code)]
    pub keepalive: Option<PyObject>,
}

#[pymethods]
impl ZBufWrapper {
    /// Number of bytes in the ZBuf (0 for device-only memory, as expected).
    fn __len__(&self) -> usize {
        use zenoh_buffers::buffer::Buffer;
        self.zbuf.len()
    }

    fn __repr__(&self) -> String {
        format!("CudaZBuf(len={})", self.__len__())
    }
}
