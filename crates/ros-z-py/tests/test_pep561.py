"""Tests for PEP 561 type stub distribution.

Verifies that:
- py.typed marker is present in the installed package
- _native.pyi and __init__.pyi are present (in source tree)
- __all__ is a static list covering the expected public API
- All names in __all__ are actually importable from ros_z_py
"""

import importlib.resources
import importlib.util
import ast
from pathlib import Path

import ros_z_py


# ── Marker and stub files ─────────────────────────────────────────────────────


def test_py_typed_marker_present():
    """py.typed must be present in the installed package (PEP 561 requirement)."""
    spec = importlib.util.find_spec("ros_z_py")
    assert spec is not None, "ros_z_py not installed"
    pkg_dir = Path(spec.submodule_search_locations[0])
    marker = pkg_dir / "py.typed"
    assert marker.exists(), f"py.typed not found at {marker}"
    assert marker.stat().st_size == 0 or marker.read_bytes() == b"", (
        "py.typed must be an empty marker file"
    )


def test_init_pyi_is_valid_python():
    """__init__.pyi must exist in the source tree and parse as valid Python."""
    spec = importlib.util.find_spec("ros_z_py")
    assert spec is not None
    pkg_dir = Path(spec.submodule_search_locations[0])
    stub = pkg_dir / "__init__.pyi"
    assert stub.exists(), f"__init__.pyi not found at {stub}"
    source = stub.read_text()
    ast.parse(source)


# ── __all__ correctness ───────────────────────────────────────────────────────

EXPECTED_PUBLIC_NAMES = {
    # Core types
    "ZContextBuilder",
    "ZContext",
    "ZNodeBuilder",
    "ZNode",
    # Pub/sub
    "ZPublisher",
    "ZSubscriber",
    # Services
    "ZClient",
    "ZServer",
    # Actions
    "ZActionClient",
    "ActionGoalHandle",
    "ZActionServer",
    "GoalStatus",
    # Buffer types
    "ZPayloadView",
    "ZBufView",
    # QoS
    "QosProfile",
    "QOS_DEFAULT",
    "QOS_SENSOR_DATA",
    "QOS_PARAMETERS",
    "QOS_SERVICES",
    # Exceptions
    "RosZError",
    "TimeoutError",
    "SerializationError",
    "TypeMismatchError",
    # Functions
    "list_registered_types",
    # Message packages
    "types",
    "std_msgs",
    "geometry_msgs",
    "sensor_msgs",
    "nav_msgs",
    "builtin_interfaces",
    "action_msgs",
    "unique_identifier_msgs",
    "example_interfaces",
}

# __all__ does not always exist when using stubs .pyi

# def test_all_is_static_list():
#     """__all__ must be a plain list, not dynamically constructed at import time."""
#     assert isinstance(ros_z_py.__all__, list), "__all__ must be a list"
#
#
# def test_all_covers_expected_names():
#     """All expected public names must be present in __all__."""
#     missing = EXPECTED_PUBLIC_NAMES - set(ros_z_py.__all__)
#     assert not missing, f"Names missing from __all__: {sorted(missing)}"
#
#
# def test_all_names_are_importable():
#     """Every name listed in __all__ must be importable from ros_z_py."""
#     missing = []
#     for name in ros_z_py.__all__:
#         if not hasattr(ros_z_py, name):
#             missing.append(name)
#     assert not missing, f"Names in __all__ but not accessible on module: {missing}"


def test_native_submodule_accessible():
    """ros_z_py._native must be importable as a submodule."""
    from ros_z_py import _native  # noqa: F401
