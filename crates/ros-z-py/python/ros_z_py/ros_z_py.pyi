"""Type stubs for the ros_z_py Rust extension module."""

from __future__ import annotations

from typing import Any, Callable, Optional

# ---------------------------------------------------------------------------
# CUDA buffer types
# ---------------------------------------------------------------------------

class PyCudaBuf:
    """A CUDA-backed buffer for zero-copy GPU tensor publishing.

    Allocate or wrap a GPU buffer, then call ``into_zbuf()`` and pass the
    result to ``ZPublisher.publish_zbuf()``.  For a higher-level API that
    handles both CUDA and CPU tensors, use ``ZPublisher.publish_tensor()``.
    """

    @staticmethod
    def alloc_device(len: int, device_id: int = 0) -> PyCudaBuf:
        """Allocate device-only memory (``cudaMalloc``).

        Args:
            len: Number of bytes to allocate.
            device_id: CUDA device ordinal (0-indexed).
        """
        ...

    @staticmethod
    def alloc_pinned(len: int, device_id: int = 0) -> PyCudaBuf:
        """Allocate pinned host memory (``cudaMallocHost``).

        Pinned memory is CPU-accessible and DMA-friendly for fast GPU transfers.

        Args:
            len: Number of bytes to allocate.
            device_id: CUDA device ordinal (0-indexed).
        """
        ...

    @staticmethod
    def from_device_ptr(ptr: int, len: int, device_id: int = 0) -> PyCudaBuf:
        """Wrap an externally-owned CUDA device pointer (non-owning).

        Does **not** allocate memory.  ``cudaFree`` will **not** be called on
        drop.  The caller is responsible for keeping the allocation alive until
        after ``publish_zbuf()`` returns.

        Args:
            ptr: Raw CUDA device pointer as integer (e.g. ``tensor.data_ptr()``).
            len: Number of bytes.
            device_id: CUDA device ordinal (0-indexed).
        """
        ...

    @staticmethod
    def from_torch(tensor: Any) -> PyCudaBuf:
        """Wrap a ``torch.Tensor`` on a CUDA device (zero-copy, non-owning).

        Captures ``data_ptr()``, ``nbytes``, shape, dtype, and strides.
        The resulting ``CudaZBuf`` carries full DLPack metadata so the
        subscriber's ``as_dlpack()`` / ``as_torch()`` return a correctly-shaped
        tensor with no out-of-band conventions.

        **Lifetime contract:** the source ``tensor`` must remain alive until the
        subscriber has called ``cudaIpcOpenMemHandle``.  Pass it as
        ``into_zbuf(keepalive=tensor)`` to prevent premature GC, or use
        ``ZPublisher.publish_tensor()`` which handles this automatically.

        Args:
            tensor: A ``torch.Tensor`` on a CUDA device.

        Raises:
            ValueError: If ``tensor`` is not on a CUDA device.
            RuntimeError: On CUDA IPC handle errors.
        """
        ...

    def with_tensor_meta(self, shape: list[int], dtype: str) -> None:
        """Attach DLPack shape and dtype metadata to this buffer.

        After calling this, ``into_zbuf()`` produces a ``CudaTensor`` ZSlice
        and the subscriber receives the correct shape/dtype via ``as_dlpack()``.

        Args:
            shape: Dimension sizes, e.g. ``[480, 640, 3]``.
            dtype: One of: ``"float16"``, ``"float32"``, ``"float64"``,
                ``"bfloat16"``, ``"int8"``, ``"int16"``, ``"int32"``,
                ``"int64"``, ``"uint8"``, ``"uint16"``, ``"uint32"``,
                ``"uint64"``, ``"bool"``.
        """
        ...

    def into_zbuf(self, keepalive: Optional[Any] = None) -> CudaZBuf:
        """Consume this buffer and return a ``CudaZBuf`` ready for publishing.

        After calling this, the ``PyCudaBuf`` is no longer usable.

        Args:
            keepalive: Optional Python object to keep alive until the
                ``CudaZBuf`` is dropped (e.g. the source torch tensor).
                Prevents premature GC of the GPU allocation.

        Returns:
            CudaZBuf ready for ``publisher.publish_zbuf()``.
        """
        ...

    @property
    def device_ptr(self) -> int:
        """Raw CUDA device pointer as a Python integer."""
        ...

    @property
    def cuda_len(self) -> int:
        """Length of the allocated buffer in bytes."""
        ...


class CudaZBuf:
    """Wrapper returned by ``PyCudaBuf.into_zbuf()``.

    Pass directly to ``ZPublisher.publish_zbuf()``.
    """

    def __len__(self) -> int: ...
    def __repr__(self) -> str: ...


# ---------------------------------------------------------------------------
# Payload view (subscriber side)
# ---------------------------------------------------------------------------

class ZPayloadView:
    """Zero-copy view of a Zenoh payload, received on the subscriber side.

    Implements Python's buffer protocol — you can pass it to
    ``memoryview()`` or ``numpy.frombuffer()`` for CPU payloads.
    For GPU payloads use ``is_cuda`` / ``as_dlpack()`` / ``as_torch()``.
    """

    @property
    def is_cuda(self) -> bool:
        """True if the payload resides on a CUDA device (CudaPtr or CudaTensor ZSlice)."""
        ...

    def as_dlpack(self) -> Optional[Any]:
        """Return a DLPack capsule suitable for ``torch.from_dlpack()``.

        Returns ``None`` if the payload is not a CUDA tensor.

        - With ``TensorMeta`` (published via ``from_torch`` or ``publish_tensor``):
          returns a correctly-shaped N-D tensor with the original dtype.
        - Without metadata (raw ``CudaPtr``): returns a 1-D ``uint8`` view.

        No data is copied — the GPU buffer is shared directly.
        """
        ...

    def as_torch(self) -> Any:
        """Return the payload as a ``torch.Tensor``.

        - **CUDA payload**: calls ``torch.from_dlpack(as_dlpack())`` — zero-copy,
          tensor lives on the originating CUDA device with original shape/dtype.
        - **CPU numpy payload** (published via ``publish_tensor(cpu_tensor)``):
          decodes numpy NPY wire format; returns a CPU tensor with original shape/dtype.
        - **Other CPU bytes**: returns a 1-D ``torch.uint8`` tensor.

        Requires ``torch`` to be installed.
        """
        ...

    def as_numpy(self) -> Any:
        """Return the payload as a ``numpy.ndarray``.

        - **CUDA payload**: copies GPU→CPU (device copy involved); prefer
          ``as_torch()`` for GPU workflows.
        - **CPU numpy payload**: decodes NPY format; returns array with original
          shape and dtype.
        - **Other CPU bytes**: returns a 1-D ``uint8`` array.

        Requires ``numpy`` (and ``torch`` for CUDA payloads).
        """
        ...

    def raw_bytes(self) -> bytes:
        """Return raw payload bytes.

        For CPU payloads: the full message bytes (CDR-encoded or raw).
        For CUDA payloads: empty ``bytes`` (device memory has no CPU representation).
        """
        ...

    # Buffer protocol — usable via memoryview() / numpy.frombuffer()
    def __buffer__(self, flags: int) -> memoryview: ...
    def __release_buffer__(self, view: memoryview) -> None: ...
    def __len__(self) -> int: ...
    def __bool__(self) -> bool: ...


# ---------------------------------------------------------------------------
# Publisher / Subscriber
# ---------------------------------------------------------------------------

class ZPublisher:
    """Zenoh publisher for a typed ROS 2 topic."""

    def publish(self, data: Any) -> None:
        """Publish a message (CDR-serialized msgspec.Struct).

        Args:
            data: A ``msgspec.Struct`` message instance (e.g. ``std_msgs.String``).
        """
        ...

    def publish_raw(self, data: bytes) -> None:
        """Publish pre-serialized CDR bytes directly (zero-copy forwarding)."""
        ...

    def publish_bytes(self, data: bytes) -> None:
        """Publish raw bytes with no CDR framing.

        Used internally by ``publish_tensor`` for the CPU tensor path.
        """
        ...

    def publish_zbuf(self, zbuf: CudaZBuf) -> None:
        """Publish a CUDA ZBuf (zero-copy GPU transport).

        Args:
            zbuf: A ``CudaZBuf`` returned by ``PyCudaBuf.into_zbuf()``.
        """
        ...

    def publish_tensor(self, tensor: Any) -> None:
        """Publish a tensor (torch or numpy) with automatic device dispatch.

        - **CUDA tensor** (``tensor.device.type == "cuda"``): wraps via
          ``PyCudaBuf.from_torch()`` and sends as a CUDA IPC payload.
          Subscriber calls ``as_torch()`` to receive the tensor on GPU.
        - **CPU tensor** or **numpy array**: serializes using numpy NPY format
          (shape + dtype preserved). Subscriber calls ``as_torch()`` or
          ``as_numpy()`` to reconstruct.

        Args:
            tensor: A ``torch.Tensor`` (any device) or ``numpy.ndarray``.

        Example::

            # CUDA (zero-copy):
            pub.publish_tensor(torch.zeros(480, 640, 3, dtype=torch.float16, device="cuda"))

            # CPU:
            pub.publish_tensor(torch.arange(1024, dtype=torch.float32))
            pub.publish_tensor(np.zeros((256, 256), dtype=np.uint8))
        """
        ...


class ZSubscriber:
    """Zenoh subscriber for a typed ROS 2 topic."""

    def recv(self, timeout: float = 1.0) -> Optional[Any]:
        """Blocking receive with timeout.

        Args:
            timeout: Seconds to wait (default 1.0).

        Returns:
            Deserialized message, or ``None`` on timeout.
        """
        ...

    def try_recv(self) -> Optional[Any]:
        """Non-blocking receive.

        Returns:
            Deserialized message, or ``None`` if no message is available.
        """
        ...

    def recv_raw_view(self, timeout: float = 1.0) -> Optional[ZPayloadView]:
        """Blocking receive returning a raw ``ZPayloadView`` (zero-copy).

        Use this for CUDA payloads or when you want to avoid CDR deserialization.

        Args:
            timeout: Seconds to wait (default 1.0).

        Returns:
            ``ZPayloadView``, or ``None`` on timeout.
        """
        ...

    def try_recv_raw_view(self) -> Optional[ZPayloadView]:
        """Non-blocking receive returning a raw ``ZPayloadView``.

        Returns:
            ``ZPayloadView``, or ``None`` if no message is available.
        """
        ...


# ---------------------------------------------------------------------------
# Context / Node
# ---------------------------------------------------------------------------

class ZContext:
    """Zenoh session context.  Entry point for creating nodes."""

    def create_node(self, name: str) -> ZNodeBuilder:
        """Create a node builder for the given name."""
        ...


class ZNodeBuilder:
    """Builder for a Zenoh ROS 2 node."""

    def build(self) -> ZNode: ...


class ZNode:
    """A Zenoh ROS 2 node."""

    def create_publisher(self, topic: str, msg_type: type) -> ZPublisher:
        """Create a publisher for the given topic and message type."""
        ...

    def create_subscription(
        self,
        topic: str,
        msg_type: type,
        callback: Callable[[Any], None],
    ) -> ZSubscriber:
        """Create a subscription with a callback."""
        ...

    def create_subscriber(self, topic: str, msg_type: type) -> ZSubscriber:
        """Create a pull-based subscriber (no callback)."""
        ...


class ZContextBuilder:
    """Builder for ``ZContext``."""

    @staticmethod
    def default() -> ZContextBuilder: ...
    def build(self) -> ZContext: ...
