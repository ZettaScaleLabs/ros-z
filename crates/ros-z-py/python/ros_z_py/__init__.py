"""ros-z-py: Python bindings for ros-z, a native Rust ROS 2 implementation using Zenoh."""

from ros_z_py._native import (
    ZContextBuilder,
    ZContext,
    ZNodeBuilder,
    ZNode,
    ZPublisher,
    ZSubscriber,
    ZClient,
    ZServer,
    ZActionClient,
    ActionGoalHandle,
    ZActionServer,
    ServerGoalRequest,
    ServerGoalHandle,
    ZPayloadView,
    ZBufView,
    QosProfile,
    GoalStatus,
    RosZError,
    TimeoutError,
    SerializationError,
    TypeMismatchError,
    QOS_DEFAULT,
    QOS_SENSOR_DATA,
    QOS_PARAMETERS,
    QOS_SERVICES,
    list_registered_types,
)

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

__all__ = [
    "ZContextBuilder",
    "ZContext",
    "ZNodeBuilder",
    "ZNode",
    "ZPublisher",
    "ZSubscriber",
    "ZClient",
    "ZServer",
    "ZActionClient",
    "ActionGoalHandle",
    "ZActionServer",
    "ServerGoalRequest",
    "ServerGoalHandle",
    "ZPayloadView",
    "ZBufView",
    "QosProfile",
    "GoalStatus",
    "RosZError",
    "TimeoutError",
    "SerializationError",
    "TypeMismatchError",
    "QOS_DEFAULT",
    "QOS_SENSOR_DATA",
    "QOS_PARAMETERS",
    "QOS_SERVICES",
    "list_registered_types",
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

# service_msgs was introduced in ROS 2 Iron (May 2023) as part of the service
# introspection feature. It contains types like ServiceEventInfo for monitoring
# service calls. This package doesn't exist in Humble (May 2022).
try:
    from ros_z_msgs_py.types import service_msgs

    __all__ = [*__all__, "service_msgs"]
except ImportError:
    pass
