//! Zero-copy payload view for Python buffer protocol.
//!
//! This module provides `ZPayloadView`, a PyO3 class that wraps a Zenoh Sample
//! and exposes its payload via Python's buffer protocol. This enables true
//! zero-copy access from Python via `memoryview()` or `numpy.frombuffer()`.

use pyo3::exceptions::PyBufferError;
use pyo3::ffi;
use pyo3::prelude::*;
use std::borrow::Cow;
use std::ffi::{CString, c_int, c_void};
use std::ptr;
use zenoh::sample::Sample;

/// Stores either a borrowed pointer (zero-copy) or owned Vec (fallback for fragmented buffers)
enum PayloadBytes {
    /// Zero-copy: pointer into Sample's ZBuf memory
    Borrowed { ptr: *const u8, len: usize },
    /// Fallback: owned Vec for non-contiguous buffers
    Owned(Vec<u8>),
}

// SAFETY: The pointer in Borrowed variant is valid as long as the Sample is alive,
// and we always keep the Sample in the same struct.
unsafe impl Send for PayloadBytes {}
unsafe impl Sync for PayloadBytes {}

/// Zero-copy view of a Zenoh payload via Python buffer protocol.
///
/// This class holds a Zenoh Sample and exposes its payload bytes through
/// Python's buffer protocol. Python code can access the data without copying:
///
/// ```python
/// payload = subscriber.recv_raw_view()
/// mv = memoryview(payload)  # Zero-copy view!
/// data = mv[offset:offset+length]  # Slicing is also zero-copy
///
/// # Or with numpy:
/// import numpy as np
/// arr = np.frombuffer(payload, dtype=np.uint8)  # numpy views ZBuf memory!
/// ```
#[pyclass(name = "ZPayloadView")]
pub struct ZPayloadView {
    /// The Sample that owns the ZBuf memory. Must not be dropped while buffer is in use.
    #[allow(dead_code)]
    sample: Sample,
    /// Cached pointer/length or owned copy of the bytes
    bytes: PayloadBytes,
}

impl ZPayloadView {
    /// Create a new ZPayloadView from a Sample.
    ///
    /// If the payload is contiguous, this stores a pointer (zero-copy).
    /// If fragmented, it copies to an owned Vec (rare case).
    pub fn new(sample: Sample) -> Self {
        let payload = sample.payload();
        let cow = payload.to_bytes();
        let bytes = match cow {
            Cow::Borrowed(b) => PayloadBytes::Borrowed {
                ptr: b.as_ptr(),
                len: b.len(),
            },
            Cow::Owned(v) => PayloadBytes::Owned(v),
        };
        Self { sample, bytes }
    }

    /// Get the payload as a byte slice.
    ///
    /// # Safety
    /// The returned slice is valid as long as `self` is alive.
    pub fn as_slice(&self) -> &[u8] {
        match &self.bytes {
            PayloadBytes::Borrowed { ptr, len } => {
                // SAFETY: ptr is valid as long as sample is alive, and we hold sample
                unsafe { std::slice::from_raw_parts(*ptr, *len) }
            }
            PayloadBytes::Owned(v) => v.as_slice(),
        }
    }

    /// Check if this view is zero-copy (borrowed) or had to copy (owned).
    pub fn is_zero_copy(&self) -> bool {
        matches!(self.bytes, PayloadBytes::Borrowed { .. })
    }
}

#[pymethods]
impl ZPayloadView {
    /// Return the length of the payload in bytes.
    fn __len__(&self) -> usize {
        self.as_slice().len()
    }

    /// Return True if the payload is empty.
    fn __bool__(&self) -> bool {
        !self.as_slice().is_empty()
    }

    /// Check if this view achieved zero-copy.
    ///
    /// Returns True if the payload was contiguous and no copy was needed.
    /// Returns False if the payload was fragmented and had to be copied.
    #[getter]
    fn is_zero_copy_py(&self) -> bool {
        self.is_zero_copy()
    }

    /// Implement Python buffer protocol - get buffer.
    ///
    /// This is called when Python code does `memoryview(payload)` or
    /// `numpy.frombuffer(payload)`.
    ///
    /// # Safety
    /// The view pointer is valid as long as the ZPayloadView Python object is alive.
    /// We ensure this by setting `view.obj` to the Python object reference.
    #[allow(unsafe_op_in_unsafe_fn)]
    unsafe fn __getbuffer__(
        slf: Bound<'_, Self>,
        view: *mut ffi::Py_buffer,
        flags: c_int,
    ) -> PyResult<()> {
        if view.is_null() {
            return Err(PyBufferError::new_err("View is null"));
        }
        if (flags & ffi::PyBUF_WRITABLE) == ffi::PyBUF_WRITABLE {
            return Err(PyBufferError::new_err("Object is not writable"));
        }

        let binding = slf.borrow();
        let bytes = binding.as_slice();

        unsafe {
            // Set obj to keep the ZPayloadView alive while buffer is in use
            (*view).obj = slf.into_any().into_ptr();

            // Set buffer pointer and length
            (*view).buf = bytes.as_ptr() as *mut c_void;
            (*view).len = bytes.len() as isize;
            (*view).readonly = 1;
            (*view).itemsize = 1;

            // Format string: "B" for unsigned bytes
            (*view).format = if (flags & ffi::PyBUF_FORMAT) == ffi::PyBUF_FORMAT {
                CString::new("B").unwrap().into_raw()
            } else {
                ptr::null_mut()
            };

            // 1-dimensional array
            (*view).ndim = 1;
            (*view).shape = if (flags & ffi::PyBUF_ND) == ffi::PyBUF_ND {
                &mut (*view).len
            } else {
                ptr::null_mut()
            };

            (*view).strides = if (flags & ffi::PyBUF_STRIDES) == ffi::PyBUF_STRIDES {
                &mut (*view).itemsize
            } else {
                ptr::null_mut()
            };

            (*view).suboffsets = ptr::null_mut();
            (*view).internal = ptr::null_mut();
        }
        Ok(())
    }

    /// Implement Python buffer protocol - release buffer.
    ///
    /// Called when the buffer view is released. We need to free the format string
    /// that was allocated in __getbuffer__.
    #[allow(unsafe_op_in_unsafe_fn)]
    unsafe fn __releasebuffer__(&self, view: *mut ffi::Py_buffer) {
        unsafe {
            if !(*view).format.is_null() {
                drop(CString::from_raw((*view).format));
            }
        }
    }

    /// Returns True if the payload resides on a CUDA device.
    ///
    /// This is True when the subscriber received a CUDA IPC handle from
    /// a publisher that wrote to GPU memory (via `ZBuf::from(CudaBufInner)`).
    #[cfg(feature = "cuda")]
    #[getter]
    fn is_cuda(&self) -> bool {
        use zenoh_buffers::ZSliceKind;
        let zbuf: zenoh_buffers::ZBuf = self.sample.payload().clone().into();
        zbuf.zslices().any(|zs| zs.kind == ZSliceKind::CudaPtr)
    }

    /// Return a DLPack capsule for the CUDA payload, suitable for `torch.from_dlpack()`.
    ///
    /// Returns `None` if the payload is not a CUDA tensor.
    ///
    /// The returned capsule exposes the GPU memory as a 1-D `uint8` tensor.
    /// No data is copied — the GPU buffer is shared directly with the consumer.
    ///
    /// # Example (Python)
    ///
    /// ```python
    /// view = subscriber.recv_raw_view()
    /// if view.is_cuda:
    ///     tensor = torch.from_dlpack(view.as_dlpack())
    /// ```
    #[cfg(feature = "cuda")]
    fn as_dlpack(&self, py: Python<'_>) -> PyResult<Option<PyObject>> {
        use zenoh_buffers::ZSliceKind;
        use zenoh_cuda::CudaBufInner;

        let zbuf: zenoh_buffers::ZBuf = self.sample.payload().clone().into();
        for zslice in zbuf.zslices() {
            if zslice.kind != ZSliceKind::CudaPtr {
                continue;
            }
            let Some(cuda) = zslice.downcast_ref::<CudaBufInner>() else {
                continue;
            };

            // Heap-allocate the context (shape + ZSlice keepalive).
            let ctx = Box::new(DlpackCtx {
                shape: cuda.cuda_len as i64,
                _zslice: zslice.clone(),
            });
            let ctx_ptr = Box::into_raw(ctx);

            // Build the DLManagedTensor. The shape pointer points into ctx,
            // which is kept alive by manager_ctx until the deleter is called.
            let managed = Box::new(DLManagedTensor {
                dl_tensor: DLTensor {
                    data: cuda.as_device_ptr() as *mut std::ffi::c_void,
                    device: DLDevice {
                        device_type: 2, // kDLCUDA
                        device_id: cuda.device_id,
                    },
                    ndim: 1,
                    dtype: DLDataType {
                        code: 1, // kDLUInt
                        bits: 8,
                        lanes: 1,
                    },
                    shape: unsafe { &mut (*ctx_ptr).shape as *mut i64 },
                    strides: std::ptr::null_mut(),
                    byte_offset: 0,
                },
                manager_ctx: ctx_ptr as *mut std::ffi::c_void,
                deleter: Some(dlpack_deleter),
            });
            let managed_ptr = Box::into_raw(managed);

            // Create a PyCapsule named "dltensor".
            let obj = unsafe {
                let name = b"dltensor\0".as_ptr() as *const std::ffi::c_char;
                let capsule = ffi::PyCapsule_New(
                    managed_ptr as *mut std::ffi::c_void,
                    name,
                    Some(dlpack_capsule_destructor),
                );
                if capsule.is_null() {
                    // Allocation failed — free what we allocated.
                    let _ = Box::from_raw((*managed_ptr).manager_ctx as *mut DlpackCtx);
                    let _ = Box::from_raw(managed_ptr);
                    return Err(pyo3::exceptions::PyMemoryError::new_err(
                        "PyCapsule_New returned null",
                    ));
                }
                PyObject::from_owned_ptr(py, capsule)
            };

            return Ok(Some(obj));
        }
        Ok(None)
    }
}

// DLPack C ABI structures and helpers (gated on `cuda` feature).
//
// DLPack (https://dmlc.github.io/dlpack/latest/) defines a standard tensor
// interchange format used by PyTorch, TensorFlow, CuPy, and others.
// We expose CUDA-backed ZBuf payloads as 1-D uint8 DLPack tensors.

#[cfg(feature = "cuda")]
#[repr(C)]
struct DLDevice {
    /// Device type: 2 = kDLCUDA.
    device_type: i32,
    /// CUDA device ordinal.
    device_id: i32,
}

#[cfg(feature = "cuda")]
#[repr(C)]
struct DLDataType {
    /// Type code: 1 = kDLUInt.
    code: u8,
    /// Number of bits per element.
    bits: u8,
    /// Number of lanes (1 for scalar types).
    lanes: u16,
}

#[cfg(feature = "cuda")]
#[repr(C)]
struct DLTensor {
    /// Raw device pointer.
    data: *mut std::ffi::c_void,
    device: DLDevice,
    ndim: i32,
    dtype: DLDataType,
    /// Pointer to shape array (length = ndim).
    shape: *mut i64,
    /// Pointer to strides array, or null for compact layout.
    strides: *mut i64,
    byte_offset: u64,
}

#[cfg(feature = "cuda")]
#[repr(C)]
struct DLManagedTensor {
    dl_tensor: DLTensor,
    /// Opaque context pointer freed by `deleter`.
    manager_ctx: *mut std::ffi::c_void,
    /// Called by the consumer to release resources. May be null.
    deleter: Option<unsafe extern "C" fn(*mut DLManagedTensor)>,
}

/// Holds the shape value and keeps the ZSlice (and its CudaBufInner Arc) alive.
#[cfg(feature = "cuda")]
struct DlpackCtx {
    shape: i64,
    _zslice: zenoh_buffers::ZSlice,
}

// SAFETY: DlpackCtx contains a ZSlice (which holds Arc<CudaBufInner>).
// CudaBufInner is Send+Sync, so DlpackCtx is too.
#[cfg(feature = "cuda")]
unsafe impl Send for DlpackCtx {}
#[cfg(feature = "cuda")]
unsafe impl Sync for DlpackCtx {}

/// DLManagedTensor deleter — called by the tensor consumer (e.g. PyTorch).
///
/// Per the DLPack spec the deleter owns ALL resources: ctx, AND the
/// DLManagedTensor struct itself.  PyTorch 2.0+ does not free the struct
/// after calling the deleter, but it does expect the struct to be gone
/// before the capsule destructor runs (it renames the capsule to
/// "used_dltensor" as a tombstone).  Freeing the struct here avoids the
/// double-free that would occur if `dlpack_capsule_destructor` also freed it.
#[cfg(feature = "cuda")]
unsafe extern "C" fn dlpack_deleter(managed: *mut DLManagedTensor) {
    if managed.is_null() {
        return;
    }
    let ctx_ptr = (*managed).manager_ctx;
    if !ctx_ptr.is_null() {
        drop(Box::from_raw(ctx_ptr as *mut DlpackCtx));
    }
    // Free the DLManagedTensor itself — per spec the deleter is responsible.
    drop(Box::from_raw(managed));
}

/// PyCapsule destructor — only runs when the capsule is GC'd uncollected.
///
/// If PyTorch consumed the capsule it renames it to "used_dltensor" and
/// calls `dlpack_deleter` which frees everything.  In that case this
/// function finds the "dltensor" lookup failing and returns early.
/// If nobody consumed the capsule (e.g. the subscriber errored out before
/// calling `from_dlpack`) the capsule is still named "dltensor" and we
/// free it here to avoid a leak.
#[cfg(feature = "cuda")]
unsafe extern "C" fn dlpack_capsule_destructor(capsule: *mut ffi::PyObject) {
    let name_cur = b"dltensor\0".as_ptr() as *const std::ffi::c_char;
    let ptr = ffi::PyCapsule_GetPointer(capsule, name_cur) as *mut DLManagedTensor;
    if ptr.is_null() {
        // Capsule was already consumed ("used_dltensor") — deleter freed everything.
        ffi::PyErr_Clear();
        return;
    }
    // Never consumed — free ctx and managed tensor.
    let ctx_ptr = (*ptr).manager_ctx;
    if !ctx_ptr.is_null() {
        drop(Box::from_raw(ctx_ptr as *mut DlpackCtx));
    }
    drop(Box::from_raw(ptr));
}
