# Feature Comparison

Three options for ROS 2 communication, from standard DDS to pure-Rust Zenoh.

| | ROS 2 (DDS) | ROS 2 (Zenoh) | ros-z |
|-|-------------|---------------|-------|
| **API** | rclcpp / rclpy | rclcpp / rclpy | Rust |
| **Transport** | DDS (FastDDS, Cyclone) | Eclipse Zenoh | Eclipse Zenoh |
| **RMW plugin** | rmw_fastrtps_cpp / rmw_cyclonedds_cpp | `rmw_zenoh_cpp` | No RMW — direct Zenoh API |
| **ROS 2 install required** | Yes | Yes | No |

ROS 2 (Zenoh) means `rmw_zenoh_cpp`: a drop-in RMW plugin that gives existing rclcpp/rclpy nodes a Zenoh transport without changing any application code. ros-z is an independent Rust API built directly on Zenoh — no ROS 2 installation needed.

## Communication

| Feature | ROS 2 (DDS) | ROS 2 (Zenoh) | ros-z |
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

| Feature | ROS 2 (DDS) | ROS 2 (Zenoh) | ros-z |
|---------|:--------------------:|:-------------:|:-----:|
| Named nodes | ✅ | ✅ | ✅ |
| Namespaces | ✅ | ✅ | ✅ |
| Name remapping | ✅ | ✅ | ✅ |
| Lifecycle nodes | ✅ | ✅ | ✅ |
| Graph introspection | ✅ | ✅ | ✅ |
| Executors / spin loop | ✅ | ✅ | pull model (in rmw-zenoh-rs) |
| Waitsets / guard conditions | ✅ | ✅ | in rmw-zenoh-rs |
| Composable / component nodes | ✅ | ✅ | in rmw-zenoh-rs |
| Intra-process communication | ✅ | ✅ | in rmw-zenoh-rs |
| Clock API (`ZClock`, system + simulated) | ✅ | ✅ | ✅ |
| Timers (`ZInterval`) | ✅ | ✅ | ✅ |
| ROS 2 `/clock` topic (external sim time) | ✅ | ✅ | ❌ planned |
| `/rosout` logging | ✅ | ✅ | ❌ uses `tracing` only |

## Quality of Service

| Feature | ROS 2 (DDS) | ROS 2 (Zenoh) | ros-z |
|---------|:--------------------:|:-------------:|:-----:|
| Reliability (reliable / best-effort) | ✅ | ✅ | ✅ |
| History (keep-last / keep-all) | ✅ | ✅ | ✅ |
| Durability (volatile / transient-local) | ✅ | ✅ | ✅ |
| Deadline QoS events | ✅ | ⚠️ Zenoh limitation | ⚠️ Zenoh limitation |
| Liveliness lease events | ✅ | ⚠️ Zenoh limitation | ⚠️ Zenoh limitation |

## Messages & Serialization

| Feature | ROS 2 (DDS) | ROS 2 (Zenoh) | ros-z |
|---------|:--------------------:|:-------------:|:-----:|
| Typed CDR messages | ✅ | ✅ | ✅ |
| Custom `.msg` / `.srv` / `.action` types | ✅ | ✅ | ✅ |
| Dynamic (schema-driven) messages | ❌ | ❌ | ✅ |
| Type description service | ✅ | ✅ | ✅ |
| Protobuf encoding | ❌ | ❌ | ✅ (feature flag) |
| Shared memory (SHM) | ✅ | ✅ | ✅ |

## Parameters

| Feature | ROS 2 (DDS) | ROS 2 (Zenoh) | ros-z |
|---------|:--------------------:|:-------------:|:-----:|
| Declare / get / set parameters | ✅ | ✅ | ✅ |
| Parameter event callbacks | ✅ | ✅ | ✅ |
| YAML parameter files | ✅ | ✅ | ✅ |
| Remote `ParameterClient` | ✅ | ✅ | ✅ |
| Parameter descriptor / constraints | ✅ | ✅ | ⚠️ partial |

## Interoperability & Distribution

| Feature | ROS 2 (DDS) | ROS 2 (Zenoh) | ros-z |
|---------|:--------------------:|:-------------:|:-----:|
| ROS 2 CLI (`ros2 topic`, `ros2 service`, …) | ✅ | ✅ | ✅ via `rmw_zenoh_cpp` |
| Jazzy / Kilted support | ✅ | ✅ | ✅ |
| Humble support | ✅ | ✅ | ✅ |
| Cross-distro bridge (Humble ↔ Jazzy) | ❌ | ❌ | ✅ |
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
