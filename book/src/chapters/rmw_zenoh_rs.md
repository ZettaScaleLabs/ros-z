# rmw_zenoh_rs

`rmw_zenoh_rs` is a Rust-based ROS Middleware (RMW) implementation that enables standard ROS 2 C++/Python nodes to communicate via Zenoh. It allows you to use Zenoh as the transport layer for your existing ROS 2 applications without modifying your code. It is **fully interoperable** with `rmw_zenoh_cpp`, allowing seamless communication between nodes using either implementation.

## What is an RMW?

ROS 2 uses a middleware abstraction layer called RMW (ROS Middleware) that allows you to choose different transport implementations. By default, ROS 2 uses DDS-based middleware like Fast-DDS or CycloneDDS. With rmw_zenoh_rs, you can use Zenoh instead.

```text
┌─────────────────────────────────────────────────┐
│              ROS 2 Application                  │
│         (C++ node using rclcpp)                 │
├─────────────────────────────────────────────────┤
│                    RCL                          │
│         (ROS Client Library - C)                │
├─────────────────────────────────────────────────┤
│                    RMW                          │
│      (ROS Middleware Interface - C)             │
├─────────────────────────────────────────────────┤
│              rmw_zenoh_rs                       │  ← This package
│         (Rust implementation)                   │
│  ┌──────────────────────────────────────────┐   │
│  │  FFI Layer (bindgen + cxx)               │   │
│  ├──────────────────────────────────────────┤   │
│  │  ros-z primitives                        │   │
│  │  (pubsub, service, guard_condition)      │   │
│  ├──────────────────────────────────────────┤   │
│  │  Zenoh                                   │   │
│  └──────────────────────────────────────────┘   │
└─────────────────────────────────────────────────┘
```

## Requirements

### System Requirements

- **ROS 2 Jazzy**
- **Rust toolchain** (1.91 or later)
- **Cargo** (comes with Rust)
- **CMake** (3.16 or later)
- **Clang** (for bindgen)

### ROS 2 Dependencies

The following ROS 2 packages are required:

- `rmw` - RMW interface definitions
- `rcutils` - ROS C utilities
- `rcpputils` - ROS C++ utilities
- `fastcdr` - Fast CDR serialization
- `rosidl_typesupport_fastrtps_c` - Type support for C
- `rosidl_typesupport_fastrtps_cpp` - Type support for C++

These are typically installed with ROS 2:

```bash
# Ubuntu/Debian with ROS 2 Jazzy
sudo apt install ros-jazzy-rmw ros-jazzy-rcutils ros-jazzy-rcpputils \
  ros-jazzy-fastcdr ros-jazzy-rosidl-typesupport-fastrtps-c \
  ros-jazzy-rosidl-typesupport-fastrtps-cpp
```

## Building rmw_zenoh_rs

```bash
# Source your ROS 2 installation
source /opt/ros/jazzy/setup.bash

# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone ros-z repository
git clone https://github.com/ZettaScaleLabs/ros-z.git

# Build rmw_zenoh_rs
cd ~/ros2_ws
colcon build --packages-select rmw_zenoh_rs --symlink-install
```

### Build Output

After building, you'll have:

- **`librmw_zenoh_rs.so`** - The RMW library (Linux)
- **RMW plugin registration** - Allows ROS 2 to discover rmw_zenoh_rs

## Using rmw_zenoh_rs

### Setting the RMW Implementation

To use rmw_zenoh_rs, set the `RMW_IMPLEMENTATION` environment variable:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_rs
```

This tells ROS 2 to use rmw_zenoh_rs instead of the default DDS implementation.

### Example: Running ROS 2 Nodes

Start a Zenoh router first, then run your nodes with the RMW implementation set:

```bash
# Terminal 1: Start Zenoh router (required)
zenohd

# Terminal 2: Talker
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_rs
ros2 run demo_nodes_cpp talker

# Terminal 3: Listener
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_rs
ros2 run demo_nodes_cpp listener
```

Your nodes will now communicate via Zenoh instead of DDS!

```admonish warning
Unlike multicast-based discovery, router-based architecture is **required** by default. Both rmw_zenoh_rs and rmw_zenoh_cpp expect a router at `tcp/localhost:7447`. Without a router, nodes will not discover each other.
```

**Why a Zenoh router?**

ros-z, rmw_zenoh_rs, and rmw_zenoh_cpp all use router-based discovery by default, which provides:

- **Better scalability** - Handles large deployments with many nodes
- **Lower network overhead** - More efficient than multicast discovery
- **Cross-network communication** - Nodes can discover each other across network boundaries
- **Production-ready architecture** - Standard approach used in real ROS 2 systems
- **Interoperability** - Required for ros-z nodes to work with rmw_zenoh_cpp nodes

```admonish info
The router runs as a separate process and manages discovery and routing between all Zenoh-based nodes. You only need one router per network, regardless of how many nodes you run.
```

## rmw_zenoh_rs vs rmw_zenoh_cpp

Both `rmw_zenoh_rs` and `rmw_zenoh_cpp` are RMW implementations using Zenoh, but with different design goals:

| Feature | rmw_zenoh_rs | rmw_zenoh_cpp |
| --- | --- | --- |
| **Implementation Language** | Rust (using ros-z) | C++ |
| **Primary Use Case** | Integration with ros-z ecosystem | Standalone Zenoh RMW |
| **ROS 2 Compatibility** | Jazzy | Humble, Kilted, Rolling |
| **Status** | Experimental | Production-ready |
| **Dependencies** | ros-z, Zenoh Rust | Zenoh C++ binding |
| **Performance** | Optimized for Rust stack | Optimized for C++ stack |
| **Interoperability** | ✅ Works with rmw_zenoh_cpp | ✅ Works with rmw_zenoh_rs |

## Configuration

rmw_zenoh_rs uses the same Zenoh configuration as rmw_zenoh_cpp. See [Configuration Options](./config_options.md) for details.

## See Also

- [Zenoh Documentation](https://zenoh.io/docs/)
- [ROS 2 RMW Documentation](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Different-Middleware-Vendors.html)
- [ros-z GitHub Repository](https://github.com/ZettaScaleLabs/ros-z)
