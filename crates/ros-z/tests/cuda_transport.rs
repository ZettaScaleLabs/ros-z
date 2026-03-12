//! Integration tests for CUDA tensor transport via ZSlice IPC.
//!
//! All tests are `#[ignore]` because they require a CUDA-capable GPU.
//! Run with:
//!
//!   cargo test -p ros-z --features jazzy,cuda -- --ignored cuda
//!
//! Two-process tests spawn a subscriber child process and communicate via
//! a unique Zenoh topic derived from `std::process::id()`.

#![cfg(feature = "cuda")]

use std::time::Duration;

use ros_z::{Builder, CudaBufInner, ZBuf, context::ZContextBuilder};
use ros_z_msgs::sensor_msgs::PointCloud2;
use zenoh_buffers::{ZSliceKind, buffer::SplitBuffer};

// ────────────────────────────────────────────────────────────
// Helper: build a minimal test context (peer mode, no router)
// ────────────────────────────────────────────────────────────
fn test_ctx() -> ros_z::context::ZContext {
    ZContextBuilder::default().build().expect("ZContext::build")
}

// ────────────────────────────────────────────────────────────
// Test 1: ZBuf::from_cuda sets ZSliceKind::CudaPtr
// ────────────────────────────────────────────────────────────
#[test]
#[ignore = "requires CUDA device"]
fn test_from_cuda_sets_kind() {
    let cuda = CudaBufInner::alloc_device(256, 0).expect("alloc_device");
    let zbuf = ZBuf::from_cuda(cuda);

    let cuda_count = zbuf.cuda_slices().count();
    assert_eq!(cuda_count, 1, "expected exactly one CudaPtr ZSlice");

    let inner = zbuf.cuda_slices().next().unwrap();
    assert_eq!(inner.cuda_len, 256);
    assert_eq!(inner.device_id, 0);
}

// ────────────────────────────────────────────────────────────
// Test 2: from_device_ptr_borrowed does not free on drop
//
// We alloc_device, get the ptr, wrap it borrowed, drop the wrapper,
// then verify the original allocation is still accessible by getting
// its IPC handle again (would crash/fail if the memory was freed).
// ────────────────────────────────────────────────────────────
#[test]
#[ignore = "requires CUDA device"]
fn test_borrowed_does_not_free() {
    let owned = CudaBufInner::alloc_device(512, 0).expect("alloc_device");
    let ptr = owned.as_device_ptr();
    let len = owned.cuda_len;

    {
        let borrowed =
            CudaBufInner::from_device_ptr_borrowed(ptr, len, 0).expect("from_device_ptr_borrowed");
        assert_eq!(borrowed.cuda_len, 512);
        // Drop borrowed here — must NOT call cudaFree
    }

    // Owned allocation still valid: re-wrap and get IPC handle (would segfault if freed)
    let rewrap =
        CudaBufInner::from_device_ptr_borrowed(ptr, len, 0).expect("re-wrap after borrowed drop");
    assert_ne!(rewrap.ipc_handle, [0u8; 64], "IPC handle must be non-zero");

    // owned drops here — calls cudaFree exactly once
    drop(owned);
    drop(rewrap);
}

// ────────────────────────────────────────────────────────────
// Test 3: ZBuf::from_cuda (borrowed) round-trip through ZSlice
// ────────────────────────────────────────────────────────────
#[test]
#[ignore = "requires CUDA device"]
fn test_borrowed_zbuf_roundtrip() {
    let owned = CudaBufInner::alloc_device(1024, 0).expect("alloc_device");
    let ptr = owned.as_device_ptr();
    let len = owned.cuda_len;

    let borrowed =
        CudaBufInner::from_device_ptr_borrowed(ptr, len, 0).expect("from_device_ptr_borrowed");
    let zbuf = ZBuf::from_cuda(borrowed);

    // ZBuf should report a CudaPtr slice
    assert_eq!(zbuf.cuda_slices().count(), 1);
    let inner = zbuf.cuda_slices().next().unwrap();
    assert_eq!(
        inner.as_device_ptr(),
        ptr,
        "pointer preserved through ZSlice"
    );
    assert_eq!(inner.cuda_len, len);

    drop(zbuf); // drops borrowed — no cudaFree
    drop(owned); // drops owned — calls cudaFree
}

// ────────────────────────────────────────────────────────────
// Test 4: pinned memory pub/sub within one process
//
// Pinned memory has a valid CPU address so it serializes as raw bytes.
// This tests that publish → receive preserves the payload.
// ────────────────────────────────────────────────────────────
#[test]
#[ignore = "requires CUDA device"]
fn test_pinned_pubsub_inprocess() {
    let ctx = test_ctx();
    let node = ctx.create_node("cuda_test_pinned").build().unwrap();

    let topic = format!("cuda_test_pinned_{}", std::process::id());
    let publisher = node
        .create_pub::<PointCloud2>(&topic)
        .build()
        .expect("create_pub");
    let subscriber = node
        .create_sub::<PointCloud2>(&topic)
        .build()
        .expect("create_sub");

    std::thread::sleep(Duration::from_millis(300));

    // Allocate pinned memory and write a pattern from CPU
    let pinned = CudaBufInner::alloc_pinned(64, 0).expect("alloc_pinned");
    let slice = unsafe { std::slice::from_raw_parts_mut(pinned.as_device_ptr(), pinned.cuda_len) };
    slice
        .iter_mut()
        .enumerate()
        .for_each(|(i, b)| *b = (i & 0xFF) as u8);

    let zbuf = ZBuf::from_cuda(pinned);
    let msg = PointCloud2 {
        data: zbuf,
        ..Default::default()
    };
    publisher.publish(&msg).expect("publish");

    let received = subscriber
        .recv_timeout(Duration::from_secs(2))
        .expect("recv_timeout");

    // Pinned memory is CPU-accessible; verify the payload bytes
    let bytes = received.data.0.contiguous();
    assert_eq!(bytes.len(), 64);
    for (i, &b) in bytes.as_ref().iter().enumerate() {
        assert_eq!(b, (i & 0xFF) as u8, "byte mismatch at index {i}");
    }
}

// ────────────────────────────────────────────────────────────
// Test 5: cuda_slices() is empty for CPU ZBuf
// ────────────────────────────────────────────────────────────
#[test]
fn test_cuda_slices_empty_for_cpu() {
    let zbuf = ZBuf::from(vec![1u8, 2, 3]);
    assert_eq!(zbuf.cuda_slices().count(), 0);
}

// ────────────────────────────────────────────────────────────
// Test 6: Typed tensor metadata survives ZBuf round-trip
//
// Verifies that TensorMeta (shape, dtype) attached via from_cuda_tensor
// is preserved and retrievable through typed_cuda_slices().
// ────────────────────────────────────────────────────────────
#[test]
#[ignore = "requires CUDA device"]
fn test_typed_tensor_zbuf() {
    use zenoh_cuda::TensorMeta;

    let shape = vec![480i64, 640, 3];
    let meta = TensorMeta {
        ndim: 3,
        shape: shape.clone(),
        dtype_code: 2,  // Float
        dtype_bits: 16, // float16
        dtype_lanes: 1,
        byte_offset: 0,
        strides: None, // C-contiguous
    };

    let buf = CudaBufInner::alloc_device(480 * 640 * 3 * 2, 0)
        .expect("alloc_device")
        .with_tensor_meta(meta);

    // Verify metadata attached before wrapping
    assert_eq!(buf.tensor_meta().unwrap().shape, shape);

    let zbuf = ZBuf::from_cuda_tensor(buf);

    // cuda_slices() sees it
    assert_eq!(zbuf.cuda_slices().count(), 1);

    // typed_cuda_slices() returns the metadata
    let mut it = zbuf.typed_cuda_slices();
    let (inner, meta_out) = it.next().expect("typed_cuda_slices must yield one entry");
    assert!(it.next().is_none(), "only one slice expected");

    assert_eq!(inner.cuda_len, 480 * 640 * 3 * 2);
    assert_eq!(meta_out.ndim, 3);
    assert_eq!(meta_out.shape, shape);
    assert_eq!(meta_out.dtype_code, 2);
    assert_eq!(meta_out.dtype_bits, 16);
    assert_eq!(meta_out.dtype_lanes, 1);
    assert!(meta_out.strides.is_none(), "C-contiguous → no strides");

    // A raw CudaPtr ZBuf must NOT appear in typed_cuda_slices()
    let raw_buf = CudaBufInner::alloc_device(64, 0).expect("alloc_device");
    let raw_zbuf = ZBuf::from_cuda(raw_buf);
    assert_eq!(
        raw_zbuf.typed_cuda_slices().count(),
        0,
        "CudaPtr without metadata must not appear in typed_cuda_slices"
    );
}

// ────────────────────────────────────────────────────────────
// Test 7: Native IPC — device ZBuf encodes a non-zero IPC handle
//
// Verifies the publish side of the native IPC path: ZBuf::from_cuda()
// produces a CudaTensor-or-CudaPtr ZSlice whose embedded IPC handle is
// non-zero, confirming that cudaIpcGetMemHandle succeeded and that the
// codec will have valid bytes to write on the wire.
//
// The full two-process decode (cudaIpcOpenMemHandle on the subscriber)
// is exercised by the ORT cross-language demo (ort_publisher / ort_classifier).
// ────────────────────────────────────────────────────────────
#[test]
#[ignore = "requires CUDA device"]
fn test_native_ipc_handle_non_zero() {
    const LEN: usize = 1024;

    let buf = CudaBufInner::alloc_device(LEN, 0).expect("alloc_device");

    // The IPC handle must be non-zero for a valid cudaMalloc allocation.
    assert_ne!(
        buf.ipc_handle, [0u8; 64],
        "cudaIpcGetMemHandle must produce a non-zero handle"
    );
    assert_eq!(buf.device_id, 0);
    assert_eq!(buf.cuda_len, LEN);

    let zbuf = ZBuf::from_cuda(buf);
    let inner = zbuf.cuda_slices().next().expect("must have one CUDA slice");

    // The IPC handle stored in the ZSlice is the one that will be encoded
    // on the wire and opened by the subscriber.
    assert_ne!(
        inner.ipc_handle, [0u8; 64],
        "IPC handle must survive ZBuf wrapping"
    );
    assert_eq!(inner.cuda_len, LEN);

    // ZSlice kind must be CudaPtr (not Raw / ShmPtr).
    let zs = zbuf.zslices().next().expect("one zslice");
    assert_eq!(
        zs.kind,
        ZSliceKind::CudaPtr,
        "device ZBuf must have CudaPtr kind"
    );
}
