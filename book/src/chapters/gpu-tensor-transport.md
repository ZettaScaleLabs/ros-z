# GPU Tensor Transport

ros-z supports zero-copy GPU tensor transport over CUDA IPC.
A publisher can send a `torch.Tensor` (or raw CUDA buffer) directly from GPU memory;
the subscriber reconstructs the tensor on the same device with the original shape
and dtype — no CPU copies, no serialization overhead.

> **Experimental.** Requires CUDA hardware and ros-z built with `--features cuda`.
> Both publisher and subscriber must run on the **same host** (CUDA IPC is local-only).

---

## Architecture

ros-z extends Zenoh's ZSlice with two new memory kinds:

| Kind | Use case | Wire payload |
|------|----------|-------------|
| `CudaPtr` | Raw device memory (1-D `uint8` view on subscriber) | IPC handle + length + device ID |
| `CudaTensor` | Typed N-D tensor (shape + dtype preserved) | IPC handle + `TensorMeta` (DLPack) |

On the publisher side, `cudaIpcGetMemHandle` captures a handle to the live GPU
allocation.  The handle travels in the Zenoh payload as a 77-byte wire blob.
On the subscriber side, `cudaIpcOpenMemHandle` remaps the same physical pages into
the subscriber process — zero bytes copied between GPU and CPU.

The Zenoh SHM transport path is extended to pass `CudaPtr`/`CudaTensor` slices
through without converting them to POSIX shared memory.

---

## Publisher quickstart

```python
import torch
from ros_z_py import ZContextBuilder, sensor_msgs

ctx = ZContextBuilder().with_shm_enabled().build()
node = ctx.create_node("gpu_pub").build()
pub = node.create_publisher("/camera/image", sensor_msgs.Image)

tensor = torch.zeros(480, 640, 3, dtype=torch.float16, device="cuda:0")

# One call — zero-copy GPU IPC, shape + dtype preserved end-to-end.
pub.publish_tensor(tensor)
```

`publish_tensor` dispatches on `tensor.device.type`:

- **`"cuda"`** — wraps via `PyCudaBuf.from_torch()`, sends as a `CudaTensor` ZSlice.
- **`"cpu"`** or **`numpy.ndarray`** — serializes with NumPy NPY format (CPU path, see below).

---

## Subscriber quickstart

```python
view = sub.recv_raw_view(timeout=5.0)  # returns ZPayloadView
tensor = view.as_torch()               # torch.Tensor on the originating CUDA device
```

`as_torch()` dispatches on payload type:

- **CUDA payload** — calls `torch.from_dlpack(view.as_dlpack())`.  No data copied.
  Returns a tensor on the originating device with the original shape and dtype.
- **CPU numpy payload** — decodes NPY wire format; returns a CPU `torch.Tensor`.
- **Other CPU bytes** — returns a 1-D `torch.uint8` tensor.

> **Import order:** Import `torch` before creating the `ZContextBuilder` to avoid
> a double-free in the CUDA allocator.  See the `cuda_pubsub.py` example for the
> correct pattern.

---

## Typed tensor round-trip

Shape and dtype travel with the tensor — no out-of-band conventions needed:

```python
# Publisher
tensor = torch.randn(1024, 1024, dtype=torch.bfloat16, device="cuda:0")
pub.publish_tensor(tensor)

# Subscriber
view = sub.recv_raw_view()
t = view.as_torch()
assert t.shape == torch.Size([1024, 1024])
assert t.dtype == torch.bfloat16
assert t.device.type == "cuda"
```

The following dtypes are supported: `float16`, `float32`, `float64`, `bfloat16`,
`int8`, `int16`, `int32`, `int64`, `uint8`, `uint16`, `uint32`, `uint64`, `bool`.

Non-contiguous tensors are supported; strides are preserved in the wire format.

---

## CPU tensor transport

`publish_tensor` also handles CPU tensors and NumPy arrays:

```python
# torch CPU tensor
pub.publish_tensor(torch.arange(1024, dtype=torch.float32))

# numpy array
import numpy as np
pub.publish_tensor(np.zeros((256, 256), dtype=np.uint8))
```

Wire format: NumPy NPY (magic `\x93NUMPY`, shape + dtype + strides header, raw data).
The subscriber receives the same shape and dtype with no extra metadata:

```python
view = sub.recv_raw_view()
t = view.as_torch()    # CPU torch.Tensor with original shape + dtype
a = view.as_numpy()    # numpy.ndarray with original shape + dtype
```

---

## Low-level API

For cases where `publish_tensor` is not flexible enough, use the lower-level
`PyCudaBuf` API directly:

```python
from ros_z_py import PyCudaBuf

# Wrap an existing tensor (non-owning — no cudaFree on drop)
buf = PyCudaBuf.from_torch(tensor)

# Attach metadata manually (alternative to from_torch)
buf = PyCudaBuf.from_device_ptr(tensor.data_ptr(), tensor.nbytes, device_id=0)
buf.with_tensor_meta(list(tensor.shape), "float16")

# Build the ZBuf (keeps tensor alive until zbuf is dropped)
zbuf = buf.into_zbuf(keepalive=tensor)
pub.publish_zbuf(zbuf)
```

Allocating a fresh GPU buffer (fill it with cupy or a CUDA kernel):

```python
buf = PyCudaBuf.alloc_device(4 * 1024 * 1024, device_id=0)
# ... fill buf.device_ptr via cupy / kernel ...
pub.publish_zbuf(buf.into_zbuf())
```

---

## Lifetime and keepalive contract

CUDA IPC sends a *handle*, not a copy.  The subscriber must call
`cudaIpcOpenMemHandle` before the publisher frees the allocation:

1. **`publish_tensor(tensor)`** — holds `tensor` alive until `publish_zbuf` returns
   (IPC handle serialized on wire).  The publisher should **not** delete the tensor
   immediately after returning; keep it alive for at least one expected round-trip.

2. **`into_zbuf(keepalive=tensor)`** — same guarantee for the manual API.
   The source tensor is held until the `CudaZBuf` is garbage-collected.

3. **Subscriber side** — `cudaIpcOpenMemHandle` is called inside `as_dlpack()` /
   `as_torch()`.  If the publisher has already freed the buffer by this point, the
   behaviour is undefined (typically a segfault or silent data corruption).

A safe pattern for tight publish loops:

```python
for tensor in stream:
    pub.publish_tensor(tensor)
    time.sleep(0.005)   # ~5 ms window for subscriber to open the handle
```

---

## Limitations

| Limitation | Detail |
|-----------|--------|
| Same-host only | CUDA IPC handles are not portable across machines |
| CUDA devices only | CUDA IPC does not work for CPU or other accelerators |
| Single-consumer | Opening an IPC handle in multiple subscriber processes simultaneously is undefined by the CUDA spec |
| PyTorch import order | `import torch` must come before `ZContextBuilder()` to avoid allocator conflicts |

---

## Running the example

```bash
# Terminal 1 (subscriber — start first)
python crates/ros-z-py/examples/cuda_pubsub.py --sub --torch

# Terminal 2 (publisher)
python crates/ros-z-py/examples/cuda_pubsub.py --pub --torch --warmup 5
```

Expected output:

```text
[pub] publish_tensor: shape=[480, 640, 3], dtype=torch.float16, device=cuda:0, nbytes=1843200
[pub] published typed CUDA tensor via publish_tensor()

[sub] received CUDA IPC view  (is_cuda=True)
[sub] tensor: shape=[480, 640, 3], dtype=torch.float16, device=cuda:0
[sub] PASS - shape=[480, 640, 3], dtype=torch.float16 (metadata preserved)
```
