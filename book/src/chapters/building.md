# Building ROS-Z

This guide covers all the ways to build ros-z, from the simplest no-dependency build to advanced configurations with ROS 2 integration.

## Philosophy

ROS-Z is designed to work **without ROS dependencies by default**. ROS-dependent features are opt-in, allowing you to:

- Build pure-Rust applications without ROS 2 installed
- Use bundled message definitions for common ROS types
- Optionally integrate with existing ROS 2 installations when needed

## Quick Build

The simplest way to get started:

```bash
# Build the core library
cargo build

# Run tests
cargo test

# Try an example
cargo run --example z_pubsub
```

## Workspace Structure

The ros-z workspace is organized into several packages:

### Default Workspace Members

Built with `cargo build` (no ROS dependencies required):

- **ros-z**: Core Zenoh-native ROS 2 library
- **ros-z-codegen**: Message and service code generation utilities

### Optional Packages

Excluded from default build but available when needed:

- **ros-z-msgs**: Auto-generated ROS 2 message types
  - Default features work without ROS (uses bundled messages)
  - External message packages require ROS 2 installation
- **ros-z-tests**: Integration tests (requires ros-z-msgs)

### ROS-Dependent Packages

Only needed when integrating with ROS 2:

- **rcl-z**: RCL C bindings (requires ROS 2 RCL libraries)

## Build Scenarios

### Scenario 1: Pure Rust (No ROS Dependencies)

Perfect for getting started or building standalone applications:

```bash
# Build the workspace (ros-z + ros-z-codegen)
cargo build

# Run tests
cargo test

# Build example with custom Rust-defined messages (no ros-z-msgs required)
cargo build -p ros-z --example z_custom_message
```

### Scenario 2: Using Bundled Messages (Still No ROS Required)

The `ros-z-msgs` package includes bundled message definitions from roslibrust (`std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`):

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

### Scenario 3: With ROS 2 Installation

Only needed for RCL bindings or external message packages not bundled with roslibrust:

```bash
# Build RCL bindings (requires ROS 2)
cargo build -p rcl-z

# Build ros-z-msgs with external message packages (requires ROS 2)
cargo build -p ros-z-msgs --features external_msgs

# Build example that uses external messages (requires ROS 2)
cargo build --example z_srvcli --features external_msgs
```

## Message Package Resolution

The `ros-z-msgs` build system automatically finds ROS message definitions in this order:

1. **System ROS installation** (via `AMENT_PREFIX_PATH` or `CMAKE_PREFIX_PATH`)
2. **Common ROS installation paths** (`/opt/ros/{rolling,jazzy,iron,humble}`)
3. **Roslibrust git dependency** (`~/.cargo/git/checkouts/roslibrust-*/assets/`)

This automatic fallback means `ros-z-msgs` can generate message types even without ROS 2 installed.

### Bundled vs External Messages

**Bundled messages** (available in roslibrust assets):

- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `nav_msgs`
- Included in default `common_interfaces` feature
- No ROS 2 installation required

**External messages** (require ROS 2):

- `example_interfaces`
- Other custom packages from your ROS 2 installation
- Enabled with `external_msgs` feature

## Using Nix (Optional)

If you're using Nix, we provide development shells with all dependencies pre-configured:

```bash
# Default development environment (with ROS 2 Jazzy)
nix develop

# Specific ROS distros
nix develop .#ros-jazzy      # ROS 2 Jazzy
nix develop .#ros-rolling    # ROS 2 Rolling

# Pure-Rust environment (no ROS)
nix develop .#pureRust

# CI environments (minimal, no dev tools)
nix develop .#ros-jazzy-ci
nix develop .#ros-rolling-ci
nix develop .#pureRust-ci
```

The Nix shells provide:

- Correct ROS 2 environment variables
- All required build dependencies
- Development tools (rust-analyzer, clippy, etc.)
- Consistent builds across machines

## Examples by Dependency Level

### Level 1: Custom Messages Only

No external dependencies required:

- `z_custom_message` - Pub/sub and services with Rust-defined messages

### Level 2: Bundled Messages

Uses roslibrust bundled messages (no ROS required):

- `z_pubsub` - Publisher/subscriber using std_msgs
- `z_pingpong` - Ping-pong latency benchmark using std_msgs
- `twist_pub` - Twist publisher using geometry_msgs
- `battery_state_sub` - Battery state subscriber using sensor_msgs
- `laser_scan` - Laser scan publisher using sensor_msgs

### Level 3: External Messages

Requires ROS 2 installation:

- `z_srvcli` - Service client/server using example_interfaces
  - Build with: `cargo build --example z_srvcli --features external_msgs`

### Level 4: Advanced Features

Requires additional features:

- `protobuf_demo` - Protobuf serialization with ROS and custom messages
  - Build with: `cargo build -p protobuf_demo`

## Troubleshooting

Having build issues? See the dedicated [Troubleshooting](./troubleshooting.md) page for comprehensive solutions to common problems.

## Next Steps

- See [Feature Flags](./feature_flags.md) for detailed feature documentation
- Explore [Examples](./examples_overview.md) to see what you can build
- Check [Troubleshooting](./troubleshooting.md) if you encounter issues
