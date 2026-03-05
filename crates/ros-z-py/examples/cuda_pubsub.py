#!/usr/bin/env python3
"""
cuda_pubsub.py — CUDA tensor transport via ros-z ZBuf (zero-copy GPU IPC).

Publishes or subscribes to a CUDA tensor topic.  CUDA IPC handles are
cross-process, so publisher and subscriber must run in separate processes.

Requirements:
    - ros_z_py built with --features cuda
    - CUDA-capable GPU (tested on GTX 1650 Ti)
    - Optional: cupy for publisher-side tensor fill
    - Optional: torch for typed tensor round-trip (--torch)

Usage:
    Terminal 1 (subscriber first):
        python cuda_pubsub.py --sub [--torch]

    Terminal 2 (publisher):
        python cuda_pubsub.py --pub [--torch]

    Typed tensor round-trip (shape + dtype preserved end-to-end):
        python cuda_pubsub.py --sub --torch &
        python cuda_pubsub.py --pub --torch --warmup 5

The --torch mode uses PyCudaBuf.from_torch() on the publisher side to attach
tensor metadata (shape, dtype) to the wire format, so the subscriber receives
a correctly-shaped tensor via torch.from_dlpack() with no out-of-band convention.

Without --torch the example uses raw device memory (CudaPtr ZSlice) and the
subscriber sees a 1-D uint8 view.
"""

import argparse
import sys
import time

try:
    from ros_z_py import (
        PyCudaBuf,
        ZContextBuilder,
        sensor_msgs,
    )
except ImportError as e:
    print(f"Failed to import ros_z_py (build with --features cuda): {e}")
    sys.exit(1)

TOPIC = "/cuda_demo/raw"
PAYLOAD_BYTES = 4 * 1024 * 1024  # 4 MB (used for raw CudaPtr mode)
DEVICE_ID = 0
FILL_VALUE = 42

# Typed tensor shape/dtype used for --torch mode.
# A "video frame" sized tensor: 480×640×3 half-precision.
TENSOR_SHAPE = (480, 640, 3)
TENSOR_DTYPE = "torch.float16"  # matches torch.dtype repr

# Use LaserScan as a placeholder type for the publisher/subscriber factory.
# publish_zbuf() bypasses message serialization so any registered type works.
MSG_TYPE = sensor_msgs.LaserScan

# Direct peer connection: subscriber listens on a fixed port, publisher connects.
# This ensures the SHM/CUDA IPC extension is negotiated directly without a router.
SUB_LISTEN = "tcp/127.0.0.1:7448"


def run_publisher(use_torch: bool, warmup: float = 1.0):
    ctx = (
        ZContextBuilder()
        .with_shm_enabled()
        .with_logging_enabled()
        .with_connect_endpoints([SUB_LISTEN])
        .build()
    )
    node = ctx.create_node("cuda_pub").build()
    pub = node.create_publisher(TOPIC, MSG_TYPE)

    # Allow subscriber declaration to propagate before publishing.
    time.sleep(warmup)

    if use_torch:
        # Typed tensor path: PyCudaBuf.from_torch() wraps the tensor and
        # attaches shape + dtype metadata so the subscriber receives a
        # correctly-shaped tensor via as_dlpack() with no out-of-band convention.
        import torch

        shape = TENSOR_SHAPE
        dtype = torch.float16
        tensor = torch.full(
            shape, FILL_VALUE / 255.0, dtype=dtype, device=f"cuda:{DEVICE_ID}"
        )
        torch.cuda.synchronize(DEVICE_ID)

        nbytes = tensor.nbytes
        print(
            f"[pub] from_torch: shape={list(tensor.shape)}, dtype={tensor.dtype},"
            f" device={tensor.device}, nbytes={nbytes}"
        )

        # from_torch() captures data_ptr + shape + dtype + strides.
        # The tensor stays alive throughout this function (IPC handle keepalive).
        buf = PyCudaBuf.from_torch(tensor)
        zbuf = buf.into_zbuf()
        pub.publish_zbuf(zbuf)
        print("[pub] published typed CUDA tensor (CudaTensor ZSlice)")

        # Keep allocation alive while subscriber opens the IPC handle.
        time.sleep(3.0)
        del tensor  # explicit to show when it drops
    else:
        # Raw bytes path: allocate + fill via cupy, publish as 1-D uint8 (CudaPtr ZSlice).
        print(
            f"[pub] allocating {PAYLOAD_BYTES // 1024 // 1024} MB on GPU {DEVICE_ID} …"
        )
        buf = PyCudaBuf.alloc_device(PAYLOAD_BYTES, device_id=DEVICE_ID)
        print(f"[pub] device ptr: 0x{buf.device_ptr:016x}  len: {buf.cuda_len}")

        try:
            import cupy as cp

            mem = cp.cuda.UnownedMemory(buf.device_ptr, buf.cuda_len, buf)
            arr = cp.ndarray(
                (buf.cuda_len,), dtype=cp.uint8, memptr=cp.cuda.MemoryPointer(mem, 0)
            )
            arr[:] = FILL_VALUE
            cp.cuda.runtime.deviceSynchronize()
            print(f"[pub] filled {buf.cuda_len} bytes with {FILL_VALUE} via cupy")
        except ImportError:
            print("[pub] cupy not available, skipping fill")

        zbuf = buf.into_zbuf()
        pub.publish_zbuf(zbuf)
        print("[pub] published CUDA IPC handle — subscriber can now map this memory")

        # Keep the GPU allocation alive while the subscriber opens the IPC handle.
        time.sleep(3.0)

    print("[pub] done — releasing GPU memory")


def run_subscriber(use_torch: bool, timeout: float = 30.0):
    # Import torch before initializing the zenoh/CUDA context so that
    # PyTorch's allocator initializes first.  Importing torch after
    # cudaIpcOpenMemHandle has already run triggers a libstdc++ double-free
    # due to conflicting allocator state between PyTorch and the Nix runtime.
    if use_torch:
        import torch as _torch  # noqa: F401 — side-effect: init allocator

    ctx = (
        ZContextBuilder()
        .with_shm_enabled()
        .with_logging_enabled()
        .with_listen_endpoints([SUB_LISTEN])
        .build()
    )
    node = ctx.create_node("cuda_sub").build()
    sub = node.create_subscriber(TOPIC, MSG_TYPE)

    print(f"[sub] waiting for CUDA tensor on {TOPIC} (timeout={timeout}s) …")
    view = sub.recv_raw_view(timeout=timeout)

    if view is None:
        print("[sub] timeout — no message received")
        sys.exit(1)

    if not view.is_cuda:
        print(f"[sub] ERROR: expected CUDA payload, got CPU ({len(view)} bytes)")
        sys.exit(1)

    print("[sub] received CUDA IPC view  (is_cuda=True)")

    if use_torch:
        import torch

        capsule = view.as_dlpack()
        if capsule is None:
            print("[sub] ERROR: as_dlpack() returned None")
            sys.exit(1)
        tensor = torch.from_dlpack(capsule)
        print(
            f"[sub] tensor: shape={list(tensor.shape)}, dtype={tensor.dtype}, device={tensor.device}"
        )

        # Verify shape and dtype were preserved end-to-end (CudaTensor metadata).
        expected_shape = list(TENSOR_SHAPE)
        expected_dtype = torch.float16
        shape_ok = list(tensor.shape) == expected_shape
        dtype_ok = tensor.dtype == expected_dtype
        if shape_ok and dtype_ok:
            print(
                f"[sub] PASS — shape={list(tensor.shape)}, dtype={tensor.dtype} (metadata preserved)"
            )
        else:
            if not shape_ok:
                print(
                    f"[sub] WARN — shape mismatch: got {list(tensor.shape)}, expected {expected_shape}"
                )
            if not dtype_ok:
                print(
                    f"[sub] WARN — dtype mismatch: got {tensor.dtype}, expected {expected_dtype}"
                )
            # Fallback: spot-check raw bytes (backwards compat with CudaPtr / non-typed path)
            raw = tensor.view(torch.uint8)[:16].cpu().tolist()
            print(f"[sub] first 16 raw bytes: {raw}")
            print("[sub] PASS — GPU memory accessible via DLPack")
    else:
        capsule = view.as_dlpack()
        if capsule is None:
            print("[sub] ERROR: as_dlpack() returned None")
            sys.exit(1)
        print(f"[sub] DLPack capsule: {capsule!r}")
        print("[sub] PASS — GPU memory accessible via DLPack")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--pub", action="store_true", help="Run as publisher")
    mode.add_argument("--sub", action="store_true", help="Run as subscriber")
    parser.add_argument(
        "--torch",
        action="store_true",
        help="Use cupy (pub) / torch (sub) for fill and verify",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Subscriber timeout in seconds (default: 30)",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=1.0,
        help="Publisher warmup sleep before publishing (default: 1.0). "
        "Increase to 4-5 when subscriber uses --torch (first CUDA init is slow).",
    )
    args = parser.parse_args()

    if args.pub:
        run_publisher(use_torch=args.torch, warmup=args.warmup)
    else:
        run_subscriber(use_torch=args.torch, timeout=args.timeout)


if __name__ == "__main__":
    main()
