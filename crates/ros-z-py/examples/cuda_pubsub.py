#!/usr/bin/env python3
"""
cuda_pubsub.py — CUDA tensor transport via ros-z ZBuf (zero-copy GPU IPC).

Publishes or subscribes to a CUDA tensor topic.  CUDA IPC handles are
cross-process, so publisher and subscriber must run in separate processes.

Requirements:
    - ros_z_py built with --features cuda
    - CUDA-capable GPU (tested on GTX 1650 Ti)
    - Optional: cupy for publisher-side tensor fill

Usage:
    Terminal 1 (subscriber first):
        python cuda_pubsub.py --sub [--torch]

    Terminal 2 (publisher):
        python cuda_pubsub.py --pub [--torch]

The publisher allocates device memory, fills it (via cupy if available),
then publishes the IPC handle over Zenoh.  The subscriber maps the GPU
memory and optionally verifies via torch.from_dlpack().
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
PAYLOAD_BYTES = 4 * 1024 * 1024  # 4 MB
DEVICE_ID = 0
FILL_VALUE = 42

# Use LaserScan as a placeholder type for the publisher/subscriber factory.
# publish_zbuf() bypasses message serialization so any registered type works.
MSG_TYPE = sensor_msgs.LaserScan

# Direct peer connection: subscriber listens on a fixed port, publisher connects.
# This ensures the SHM/CUDA IPC extension is negotiated directly without a router.
SUB_LISTEN = "tcp/127.0.0.1:7448"


def run_publisher(use_torch: bool):
    ctx = (
        ZContextBuilder()
        .with_shm_enabled()
        .with_logging_enabled()
        .with_connect_endpoints([SUB_LISTEN])
        .build()
    )
    node = ctx.create_node("cuda_pub").build()
    pub = node.create_publisher(TOPIC, MSG_TYPE)

    print(f"[pub] allocating {PAYLOAD_BYTES // 1024 // 1024} MB on GPU {DEVICE_ID} …")
    buf = PyCudaBuf.alloc_device(PAYLOAD_BYTES, device_id=DEVICE_ID)
    print(f"[pub] device ptr: 0x{buf.device_ptr:016x}  len: {buf.cuda_len}")

    if use_torch:
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

    # Allow subscriber declaration to propagate before publishing.
    time.sleep(1.0)

    zbuf = buf.into_zbuf()
    pub.publish_zbuf(zbuf)
    print("[pub] published CUDA IPC handle — subscriber can now map this memory")

    # Keep the GPU allocation alive while the subscriber opens the IPC handle.
    # cudaIpcOpenMemHandle requires the source allocation to still exist.
    time.sleep(3.0)
    print("[pub] done — releasing GPU memory")


def run_subscriber(use_torch: bool, timeout: float = 30.0):
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
        # If publisher filled with FILL_VALUE, spot-check
        sample = tensor[:16].cpu().tolist()
        print(f"[sub] first 16 bytes: {sample}")
        if all(b == FILL_VALUE for b in sample):
            print(f"[sub] PASS — all bytes are {FILL_VALUE}")
        else:
            print("[sub] (fill not verified — publisher may not have used cupy)")
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
    args = parser.parse_args()

    if args.pub:
        run_publisher(use_torch=args.torch)
    else:
        run_subscriber(use_torch=args.torch, timeout=args.timeout)


if __name__ == "__main__":
    main()
