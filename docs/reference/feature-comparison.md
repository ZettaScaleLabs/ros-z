# Feature Comparison

Three ways to use ROS 2 over Zenoh, compared against the standard DDS stack.

| | rclcpp / rclpy (DDS) | rclcpp / rclpy + `rmw_zenoh_cpp` | ros-z (pure Rust) |
|-|----------------------|----------------------------------|-------------------|
| **Language** | C++ / Python | C++ / Python | Rust |
| **Transport** | DDS (FastDDS, Cyclone) | Eclipse Zenoh | Eclipse Zenoh |
| **RMW layer** | Yes | Yes | No — direct Zenoh API |
| **ROS 2 install required** | Yes | Yes | No |

`rmw_zenoh_cpp` gives standard rclcpp/rclpy nodes a Zenoh transport without changing any application code. ros-z is a separate Rust API built directly on Zenoh — no ROS 2 installation needed.

## Communication

| Feature | rclcpp / rclpy (DDS) | rmw_zenoh_cpp | ros-z |
|---------|:--------------------:|:-------------:|:-----:|
| Publishers | ✅ | ✅ | ✅ |
| Subscribers | ✅ | ✅ | ✅ |
| Callback subscribers | ✅ | ✅ | ✅ |
| Pull-model subscribers (`recv()`) | ❌ | ❌ | ✅ |
| Services (request / response) | ✅ | ✅ | ✅ |
| Service clients | ✅ | ✅ | ✅ |
| Actions (goal / feedback / result) | ✅ | ✅ | ✅ |
| Action cancellation | ✅ | ✅ | ✅ |

## Node Features

| Feature | rclcpp / rclpy (DDS) | rmw_zenoh_cpp | ros-z |
|---------|:--------------------:|:-------------:|:-----:|
| Named nodes | ✅ | ✅ | ✅ |
| Namespaces | ✅ | ✅ | ✅ |
| Name remapping | ✅ | ✅ | ✅ |
| Lifecycle nodes | ✅ | ✅ | ✅ |
| Graph introspection | ✅ | ✅ | ✅ |
| Executors / spin loop | ✅ | ✅ | ❌ pull model |
| Waitsets / guard conditions | ✅ | ✅ | ❌ |
| Composable / component nodes | ✅ | ✅ | ❌ |
| Intra-process communication | ✅ | ✅ | ❌ |
| Timers | ✅ | ✅ | ❌ planned |
| Clock / sim time (`/clock`) | ✅ | ✅ | ❌ planned |
| `/rosout` logging | ✅ | ✅ | ❌ uses `tracing` only |

## Quality of Service

| Feature | rclcpp / rclpy (DDS) | rmw_zenoh_cpp | ros-z |
|---------|:--------------------:|:-------------:|:-----:|
| Reliability (reliable / best-effort) | ✅ | ✅ | ✅ |
| History (keep-last / keep-all) | ✅ | ✅ | ✅ |
| Durability (volatile / transient-local) | ✅ | ✅ | ✅ |
| Deadline QoS events | ✅ | ⚠️ Zenoh limitation | ⚠️ Zenoh limitation |
| Liveliness lease events | ✅ | ⚠️ Zenoh limitation | ⚠️ Zenoh limitation |

## Messages & Serialization

| Feature | rclcpp / rclpy (DDS) | rmw_zenoh_cpp | ros-z |
|---------|:--------------------:|:-------------:|:-----:|
| Typed CDR messages | ✅ | ✅ | ✅ |
| Custom `.msg` / `.srv` / `.action` types | ✅ | ✅ | ✅ |
| Dynamic (schema-driven) messages | ❌ | ❌ | ✅ |
| Type description service | ✅ | ✅ | ✅ |
| Protobuf encoding | ❌ | ❌ | ✅ (feature flag) |
| Shared memory (SHM) | ✅ | ✅ | ✅ |

## Parameters

| Feature | rclcpp / rclpy (DDS) | rmw_zenoh_cpp | ros-z |
|---------|:--------------------:|:-------------:|:-----:|
| Declare / get / set parameters | ✅ | ✅ | ✅ |
| Parameter event callbacks | ✅ | ✅ | ✅ |
| YAML parameter files | ✅ | ✅ | ✅ |
| Remote `ParameterClient` | ✅ | ✅ | ✅ |
| Parameter descriptor / constraints | ✅ | ✅ | ⚠️ partial |

## Interoperability & Distribution

| Feature | rclcpp / rclpy (DDS) | rmw_zenoh_cpp | ros-z |
|---------|:--------------------:|:-------------:|:-----:|
| ROS 2 CLI (`ros2 topic`, `ros2 service`, …) | ✅ | ✅ | ✅ via `rmw_zenoh_cpp` |
| Jazzy / Kilted support | ✅ | ✅ | ✅ |
| Humble support | ✅ | ✅ | ✅ |
| Cross-distro bridge (Humble ↔ Jazzy) | ❌ | ❌ | ✅ |
| Pre-built binaries (no compile step) | ❌ | ❌ | ✅ |
| Python bindings | native | native | ✅ (`ros-z-py`) |
| Go bindings | ❌ | ❌ | ✅ (`ros-z-go`) |
| No ROS 2 install needed | ❌ | ❌ | ✅ |

## Which to choose?

**Use `rmw_zenoh_cpp`** when you have an existing rclcpp/rclpy codebase and want Zenoh transport without rewriting anything. You get the full ROS 2 feature set — executors, composable nodes, lifecycle — with Zenoh's routing and cross-network capabilities replacing DDS.

**Use ros-z** when you are writing new code in Rust and want a minimal, dependency-light library. No ROS 2 installation, no DDS, no executor overhead. ros-z nodes interoperate transparently with `rmw_zenoh_cpp` nodes over the same Zenoh router.

## Resources

- **[ROS 2 → ros-z API Mapping](./ros2-to-rosz.md)** — method-by-method translation table
- **[rmw-zenoh-rs](../experimental/rmw-zenoh-rs.md)** — the RMW plugin powering `rmw_zenoh_cpp`
- **[ROS 2 Interoperability](../user-guide/interop.md)** — running ros-z alongside rclcpp/rclpy nodes
- **[Cross-Distro Bridge](../user-guide/bridge.md)** — Humble ↔ Jazzy bridging
