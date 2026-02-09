"""ros-z-py: Python bindings for ros-z, a native Rust ROS 2 implementation using Zenoh."""

# Import all from the Rust extension module
from .ros_z_py import *  # noqa: F403

# Re-export message types from ros_z_msgs_py.types
from ros_z_msgs_py import types

# Re-export individual message packages for convenience
from ros_z_msgs_py.types import (
    std_msgs,
    geometry_msgs,
    sensor_msgs,
    nav_msgs,
    builtin_interfaces,
    action_msgs,
    unique_identifier_msgs,
    example_interfaces,
)

# service_msgs was introduced in ROS 2 Iron (May 2023) as part of the service
# introspection feature. It contains types like ServiceEventInfo for monitoring
# service calls. This package doesn't exist in Humble (May 2022).
try:
    from ros_z_msgs_py.types import service_msgs
except ImportError:
    service_msgs = None  # type: ignore[assignment]

# Update __all__ to include message types
__all__ = list(ros_z_py.__all__ if hasattr(ros_z_py, "__all__") else []) + [  # noqa: F405
    "types",
    "std_msgs",
    "geometry_msgs",
    "sensor_msgs",
    "nav_msgs",
    "builtin_interfaces",
    "action_msgs",
    "unique_identifier_msgs",
    "example_interfaces",
]

# Add service_msgs to __all__ if available (Jazzy+)
if service_msgs is not None:
    __all__.append("service_msgs")

# Copy docstring from the Rust module if available
if hasattr(ros_z_py, "__doc__") and ros_z_py.__doc__:  # noqa: F405
    __doc__ = ros_z_py.__doc__  # noqa: F405
