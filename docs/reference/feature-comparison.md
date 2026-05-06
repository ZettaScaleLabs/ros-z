# Feature Comparison: ros-z vs rclcpp / rclpy

This page summarises which ROS 2 features ros-z supports, which are planned, and which live elsewhere in the stack.

!!! note
    Some ROS 2 middleware features (waitsets, guard conditions, composable nodes) are implemented in **[rmw-zenoh-rs](../experimental/rmw-zenoh-rs.md)** — the Zenoh RMW plugin for standard ROS 2 C++/Python nodes. They are intentionally absent from ros-z, which targets the pure-Rust API surface.

## Communication

| Feature | rclcpp / rclpy | ros-z |
|---------|---------------|-------|
| Publishers | ✅ | ✅ |
| Subscribers | ✅ | ✅ |
| Callback subscribers | ✅ | ✅ |
| Pull-model subscribers (`recv()`) | ❌ | ✅ |
| Services (request / response) | ✅ | ✅ |
| Service clients | ✅ | ✅ |
| Actions (goal / feedback / result) | ✅ | ✅ |
| Action cancellation | ✅ | ✅ |

## Node Features

| Feature | rclcpp / rclpy | ros-z |
|---------|---------------|-------|
| Named nodes | ✅ | ✅ |
| Namespaces | ✅ | ✅ |
| Name remapping | ✅ | ✅ |
| Lifecycle nodes | ✅ | ✅ |
| Graph introspection | ✅ | ✅ |
| Timers | ✅ | ❌ planned |
| Clock / sim time (`/clock`) | ✅ | ❌ planned |
| `/rosout` logging | ✅ | ❌ uses `tracing` only |

## Quality of Service

| Feature | rclcpp / rclpy | ros-z |
|---------|---------------|-------|
| Reliability (reliable / best-effort) | ✅ | ✅ |
| History (keep-last / keep-all) | ✅ | ✅ |
| Durability (volatile / transient-local) | ✅ | ✅ |
| Deadline QoS events | ✅ | ⚠️ Zenoh limitation |
| Liveliness lease events | ✅ | ⚠️ Zenoh limitation |

## Messages & Serialization

| Feature | rclcpp / rclpy | ros-z |
|---------|---------------|-------|
| Typed CDR messages | ✅ | ✅ |
| Custom `.msg` / `.srv` / `.action` types | ✅ | ✅ |
| Message codegen (`ros-z-codegen`) | N/A | ✅ |
| Dynamic (schema-driven) messages | ❌ | ✅ |
| Type description service | ✅ | ✅ |
| Protobuf encoding | ❌ | ✅ (feature flag) |
| Shared memory (SHM) | ✅ (via rmw) | ✅ |

## Parameters

| Feature | rclcpp / rclpy | ros-z |
|---------|---------------|-------|
| Declare / get / set parameters | ✅ | ✅ |
| Parameter event callbacks | ✅ | ✅ |
| YAML parameter files | ✅ | ✅ |
| Remote `ParameterClient` | ✅ | ✅ |
| Parameter descriptor / constraints | ✅ | ⚠️ partial |

## Interoperability

| Feature | rclcpp / rclpy | ros-z |
|---------|---------------|-------|
| ROS 2 CLI (`ros2 topic`, `ros2 service`, …) | ✅ | ✅ via `rmw_zenoh_cpp` |
| Jazzy / Kilted support | ✅ | ✅ |
| Humble support | ✅ | ✅ |
| Cross-distro bridge (Humble ↔ Jazzy) | ❌ | ✅ |
| Python bindings | native | ✅ (`ros-z-py`) |
| Go bindings | ❌ | ✅ (`ros-z-go`) |

## Not in ros-z (lives in rmw-zenoh-rs)

The following are middleware-layer concerns handled by **rmw-zenoh-rs** when you use standard ROS 2 C++/Python nodes over Zenoh. They are not part of the pure-Rust ros-z API:

| Feature | Where |
|---------|-------|
| Waitsets / guard conditions | rmw-zenoh-rs |
| Executor model | rmw-zenoh-rs |
| Composable / component nodes | rmw-zenoh-rs |
| Intra-process communication | rmw-zenoh-rs |

If you need any of these from Rust, combine ros-z nodes with a standard ROS 2 executor via the `rmw_zenoh_cpp` bridge.

## Resources

- **[ROS 2 → ros-z API Mapping](./ros2-to-rosz.md)** — method-by-method translation table
- **[rmw-zenoh-rs](../experimental/rmw-zenoh-rs.md)** — RMW plugin for standard ROS 2 nodes
- **[Cross-Distro Bridge](../user-guide/bridge.md)** — Humble ↔ Jazzy bridging
