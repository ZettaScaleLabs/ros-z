<div align="center">
    <h1>ROS-Z</h1>
    <p><strong>Making ROS 2 Zenoh-Native</strong></p>
    <sub>Built by the <a href="https://zenoh.io">Zenoh</a> team at <a href="https://www.zettascale.tech">ZettaScale</a></sub>
</div>

## Overview

ROS 2 was designed to be independent from the underlying communication
middleware. This is a nice architectural property, yet it does not come for
free.  What if we were to streamline ROS 2 and implement it natively on Zenoh?
ROS-Z, a Zenoh-native ROS 2 stack, answers this question. ROS-Z preserves
portability for RCL-C/CPP/Py-based applications and provides an extremely
optimised stack for Rust users that interoperates with any Zenoh RMW-based
ROS 2.

![architecture](./assets/architecture.png)

## Goals

**ROS-Z** was started as an experiment to understand what we could gain from
verticalisation while at the same time providing (1) a pure-Rust stack to ROS2
users, and (2) zenoh-native implementation of RCL-C. We hope that the lesson
learned through this experiment will help the ROS community making ROS 2 even
better.

## Status

**ROS-Z** is experimental software. It is tested with ROS 2 Jazzy and should
be interoperable with ROS 2 Rolling, but we make no guarantees with respect to
official distributions.

### Current Implementation Status

- ✅ **Core ROS 2 Concepts**: Nodes, publishers, subscribers, services, actions
- ✅ **Message Types**: Support for all ROS 2 message types via code generation
- ✅ **Quality of Service**: Full QoS policy support
- ✅ **Actions**: Complete ROS 2 action protocol implementation with high performance
- ✅ **Interoperability**: Compatible with ROS 2 RMW implementations via Zenoh
- ✅ **Performance**: Sub-microsecond latencies for core operations

## Building

ROS-Z is designed to work without ROS dependencies by default.
ROS-dependent features are opt-in.

### Default Build (No ROS Dependencies)

The default workspace build includes only the core library and works without
ROS 2:

```bash
# Build the workspace (ros-z + ros-z-codegen)
cargo build

# Run tests
cargo test

# Build example with custom Rust-defined messages (no ros-z-msgs required)
cargo build -p ros-z --example z_custom_message
```

### Building with ros-z-msgs (still no ROS required)

The `ros-z-msgs` package includes bundled message definitions (std_msgs,
geometry_msgs, sensor_msgs, nav_msgs) and can be built without ROS:

```bash
# Build ros-z-msgs with bundled messages (default, no ROS required)
cargo build -p ros-z-msgs

# Build examples that use bundled ros-z-msgs types
cargo build -p ros-z --example z_pubsub       # Uses std_msgs
cargo build -p ros-z --example z_pingpong     # Uses std_msgs
cargo build -p ros-z --example twist_pub      # Uses geometry_msgs
cargo build -p ros-z --example battery_state_sub  # Uses sensor_msgs
cargo build -p ros-z --example laser_scan     # Uses sensor_msgs
```

### With ROS 2 Installation (Optional)

Only needed for RCL bindings or external message packages not bundled with
roslibrust:

```bash
# Build RCL bindings (requires ROS 2)
cargo build -p rcl-z

# Build ros-z-msgs with external message packages (requires ROS 2)
cargo build -p ros-z-msgs --features external_msgs

# Build example that uses external messages (requires ROS 2)
cargo build --example z_srvcli --features external_msgs
```

### Using Nix (Optional)

If you're using Nix, we provide development shells with all dependencies
pre-configured:

```bash
# Default development environment (with ROS 2 Jazzy)
nix develop

# Specific ROS distros
nix develop .#ros-jazzy      # ROS 2 Jazzy
nix develop .#ros-rolling    # ROS 2 Rolling

# Pure-Rust
nix develop .#pureRust

# CI environments (minimal, no dev tools)
nix develop .#ros-jazzy-ci
nix develop .#ros-rolling-ci
nix develop .#pureRust-ci
```

**Note on `ros-z-msgs`:** This package can build without ROS installed! When
ROS is not available, it automatically falls back to using bundled message
definitions from the roslibrust git dependency. The build system searches for
ROS packages in this order:

1. System ROS installation (via `AMENT_PREFIX_PATH` or `CMAKE_PREFIX_PATH`)
2. Common ROS installation paths (`/opt/ros/{rolling,jazzy,iron,humble}`)
3. Roslibrust git dependency (`~/.cargo/git/checkouts/roslibrust-*/assets/`)

This allows `ros-z-msgs` to generate message types even in environments without
ROS 2 installed. The default `common_interfaces` feature includes `std_msgs`,
`geometry_msgs`, and `sensor_msgs` which are all available in roslibrust
assets. Note that `example_interfaces` is **not** included in the default build
as it requires a ROS 2 installation.

### Workspace Structure

**Default workspace members** (no ROS dependencies):

- **ros-z**: Core Zenoh-native ROS 2 library
- **ros-z-codegen**: Message and service code generation utilities

**Optional packages** (excluded from default build):

- **ros-z-msgs**: Auto-generated ROS 2 message types (default features work
  without ROS; external message packages require ROS 2)
- **ros-z-tests**: Integration tests (requires ros-z-msgs)

**ROS-dependent packages** (excluded from default build):

- **rcl-z**: RCL C bindings (requires ROS 2 RCL libraries)

### Examples

Examples are categorized by their dependencies:

- **Custom Rust messages** (no ros-z-msgs required):
  - `z_custom_message` - Demonstrates pub/sub and service with custom
    Rust-defined messages

- **Bundled messages** (no ROS required, uses roslibrust assets):
  - `z_pubsub` - Publisher/subscriber using std_msgs
  - `z_pingpong` - Ping-pong latency benchmark using std_msgs
  - `twist_pub` - Twist publisher using geometry_msgs
  - `battery_state_sub` - Battery state subscriber using sensor_msgs
  - `laser_scan` - Laser scan publisher using sensor_msgs

- **External messages** (requires ROS 2 installation):
  - `z_srvcli` - Service client/server using example_interfaces
    - Build with: `cargo build --example z_srvcli --features external_msgs`

- **Advanced** (requires protobuf feature):
  - `protobuf_demo` - Demonstrates protobuf serialization with both ROS
    messages and custom protobuf messages
    - Build with: `cargo build -p protobuf_demo`

- **Actions** (ROS 2 action protocol implementation):
  - `action_fibonacci` - Fibonacci action server and client example
  - `action_cancelable_task` - Demonstrates action cancellation
  - `action_concurrent_goals` - Shows concurrent goal handling
  - `action_manual_control` - Manual goal state management example

## ROS 2 Actions

ROS-Z includes a complete implementation of the ROS 2 action protocol, providing
high-performance action servers and clients that are fully compatible with ROS 2
action interfaces.

### Key Features

- **Full ROS 2 Compliance**: Implements the complete ROS 2 action protocol including
  goal states, result codes, and feedback mechanisms
- **High Performance**: Sub-microsecond operation latencies with minimal memory footprint
- **Type Safety**: Compile-time guarantees for action message types
- **Async/Await**: Modern Rust async programming model
- **Interoperability**: Works with any ROS 2 action client/server

### Action Examples

```rust
use ros_z::{Builder, action::ZAction};
use serde::{Deserialize, Serialize};

// Define your action messages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FibonacciGoal {
    pub order: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FibonacciResult {
    pub sequence: Vec<u32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FibonacciFeedback {
    pub sequence: Vec<u32>,
}

// Define the action
pub struct FibonacciAction;

impl ZAction for FibonacciAction {
    type Goal = FibonacciGoal;
    type Result = FibonacciResult;
    type Feedback = FibonacciFeedback;

    fn name() -> &'static str {
        "fibonacci"
    }
}

// Use in async code
let ctx = Builder::default().build()?;
let node = ctx.create_node("fibonacci_client").build()?;

// Create action client
let client = node.create_action_client::<FibonacciAction>("fibonacci").build()?;

// Send goal
let goal = FibonacciGoal { order: 10 };
let goal_handle = client.send_goal(goal).await?;

// Wait for result
let result = goal_handle.result().await?;
println!("Fibonacci sequence: {:?}", result.sequence);
```

### Performance Characteristics

The ROS-Z action implementation delivers excellent performance:

- **Goal ID Generation**: 1.6M ops/sec (0.63 μs/op)
- **State Transitions**: 159M ops/sec (0.006 μs/op)
- **Message Serialization**: 1.3M ops/sec (0.77 μs/op)
- **Memory Usage**: Minimal - UUID-based goal IDs, compact structures

These characteristics make ROS-Z actions suitable for high-throughput robotics
applications requiring real-time performance.

## Feature Flags

### ros-z

- `protobuf` - Enable protobuf serialization support (requires `prost`)
- `rcl-z` - Enable RCL integration features
- `external_msgs` - Enable examples that require external ROS message packages
  (propagates to `ros-z-msgs/external_msgs`)

### ros-z-msgs

Messages are organized into two categories:

**Bundled messages** (no ROS installation required):

- `bundled_msgs` (default) - All bundled message packages
- `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs` - Individual bundled
  packages
- `common_interfaces` - Convenience feature for std_msgs, geometry_msgs,
  sensor_msgs

**External messages** (require ROS 2 installation):

- `external_msgs` - All external message packages
- `example_interfaces` - Individual external package

**Other features**:

- `all_msgs` - Enable both bundled and external messages (requires ROS 2)
- `protobuf` - Generate protobuf types (requires `ros-z/protobuf`)

### ros-z-codegen

- `protobuf` - Enable protobuf code generation support

### ros-z-tests

- `ros-msgs` - Enable tests with ros-z-msgs dependency
- `interop-tests` - ROS interoperability tests (requires `ros-msgs`)
