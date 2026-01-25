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
    service_msgs,
    unique_identifier_msgs,
    example_interfaces,
)

# Update __all__ to include message types
__all__ = list(ros_z_py.__all__ if hasattr(ros_z_py, "__all__") else []) + [  # noqa: F405
    "types",
    "std_msgs",
    "geometry_msgs",
    "sensor_msgs",
    "nav_msgs",
    "builtin_interfaces",
    "action_msgs",
    "service_msgs",
    "unique_identifier_msgs",
    "example_interfaces",
]

# Copy docstring from the Rust module if available
if hasattr(ros_z_py, "__doc__") and ros_z_py.__doc__:  # noqa: F405
    __doc__ = ros_z_py.__doc__  # noqa: F405
