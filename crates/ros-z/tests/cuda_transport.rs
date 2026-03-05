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

use ros_z::{Builder, CudaBufInner, ZBuf, context::ZContextBuilder};
use ros_z_msgs::sensor_msgs::PointCloud2;
use std::time::Duration;
use zenoh_buffers::buffer::SplitBuffer;

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
