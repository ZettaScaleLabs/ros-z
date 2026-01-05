# Building ros-z

**ros-z is designed to work without ROS 2 dependencies by default, enabling pure Rust development while optionally integrating with existing ROS 2 installations.** This flexible approach lets you choose your dependency level based on project requirements.

```admonish success
Start with zero dependencies and add ROS 2 integration only when you need it. This gradual approach reduces complexity and speeds up initial development.
```

## Philosophy

ros-z follows a **dependency-optional** design:

- Build pure Rust applications without ROS 2 installed
- Use bundled message definitions for common types
- Opt-in to ROS 2 integration when needed
- Pay only for what you use

## Adding ros-z to Your Project

Get started by adding ros-z to your `Cargo.toml`. Choose the dependency setup that matches your needs:

### Scenario 1: Pure Rust with Custom Messages

**Use when:** You want to define your own message types without ROS 2 dependencies

**Add to your `Cargo.toml`:**

```toml
[dependencies]
ros-z = "0.x"
tokio = { version = "1", features = ["full"] }  # Async runtime required
```

**What you get:**

- Full ros-z functionality
- Custom message support via derive macros
- Zero external dependencies
- Fast build times

### Scenario 2: Using Bundled ROS Messages

**Use when:** You need standard ROS 2 message types but don't have ROS 2 installed

**Add to your `Cargo.toml`:**

```toml
[dependencies]
ros-z = "0.x"
ros-z-msgs = "0.x"  # Includes std_msgs, geometry_msgs, sensor_msgs, nav_msgs
tokio = { version = "1", features = ["full"] }
```

**Bundled message packages:**

- `std_msgs` - Primitive types (String, Int32, Float64, etc.)
- `geometry_msgs` - Spatial data (Point, Pose, Transform, Twist)
- `sensor_msgs` - Sensor data (LaserScan, Image, Imu, PointCloud2)
- `nav_msgs` - Navigation (Path, OccupancyGrid, Odometry)

### Scenario 3: Full ROS 2 Integration

**Use when:** You need all ROS 2 message types or custom packages from your workspace

**Requirements:** ROS 2 installation (Jazzy, Rolling, Iron, or Humble)

**Add to your `Cargo.toml`:**

```toml
[dependencies]
ros-z = "0.x"
ros-z-msgs = { version = "0.x", features = ["external_msgs"] }
tokio = { version = "1", features = ["full"] }
```

**Build your project:**

```bash
# Ensure ROS 2 is sourced
source /opt/ros/jazzy/setup.bash

# Build your project
cargo build
```

**Additional packages available:**

- `example_interfaces` - Tutorial services and actions
- `action_msgs` - Action communication types
- Any custom messages from your ROS 2 workspace

```admonish tip
Start with Scenario 1 or 2 for development, then move to Scenario 3 when you need full ROS 2 interoperability.
```

## Running Examples

Once you've added ros-z to your project, you can run the included examples to see it in action.

```admonish important
**All examples require a Zenoh router to be running first:**
```bash
cargo run --example zenoh_router
```

Leave this running in a separate terminal, then run any example in another terminal.

**Available examples:**

```bash
# Pure Rust example with custom messages (no ros-z-msgs needed)
cargo run --example z_custom_message -- --mode status-pub

# Examples using bundled messages (requires ros-z-msgs)
cargo run --example z_pubsub          # Publisher/Subscriber with std_msgs
cargo run --example twist_pub         # Publishing geometry_msgs
cargo run --example battery_state_sub # Receiving sensor_msgs

# Examples requiring ROS 2 (requires external_msgs feature)
cargo run --example z_srvcli --features external_msgs
```

See the [Zenoh Configuration](./zenoh_config.md) chapter for router setup details and alternative configurations.

## Using Nix (Optional)

Pre-configured development environments with all dependencies:

```bash
# Default: ROS 2 Jazzy with full tooling
nix develop

# Specific ROS distros
nix develop .#ros-jazzy      # ROS 2 Jazzy
nix develop .#ros-rolling    # ROS 2 Rolling

# Pure Rust (no ROS)
nix develop .#pureRust
```

```admonish tip
Use Nix for consistent development environments across team members and CI/CD pipelines.
```

## Development

This section is for contributors working on ros-z itself. If you're using ros-z in your project, you can skip this section.

### Package Organization

The ros-z repository is organized as a Cargo workspace with multiple packages:

| Package | Default Build | Purpose | Dependencies |
|---------|---------------|---------|--------------|
| **ros-z** | Yes | Core Zenoh-native ROS 2 library | None |
| **ros-z-codegen** | Yes | Message generation utilities | None |
| **ros-z-msgs** | No | Pre-generated message types | Optional ROS 2 |
| **ros-z-tests** | No | Integration tests | ros-z-msgs |
| **rcl-z** | No | RCL C bindings | ROS 2 required |

```admonish note
Only `ros-z` and `ros-z-codegen` build by default. Other packages are optional for development, testing, and running examples.
```

### Building the Repository

When contributing to ros-z, you can build different parts of the workspace:

```bash
# Build core library
cargo build

# Run tests
cargo test

# Build with bundled messages for examples
cargo build -p ros-z-msgs

# Build all packages (requires ROS 2)
source /opt/ros/jazzy/setup.bash
cargo build --all
```

### Message Package Resolution

The build system automatically locates ROS message definitions:

**Search order:**

1. System ROS installation (`AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`)
2. Common ROS paths (`/opt/ros/{rolling,jazzy,iron,humble}`)
3. Roslibrust git checkout (`~/.cargo/git/checkouts/roslibrust-*/assets/`)

This fallback mechanism enables builds without ROS 2 installed.

### Common Development Commands

```bash
# Fast iterative development
cargo check                # Quick compile check
cargo build                # Debug build
cargo build --release      # Optimized build
cargo test                 # Run tests
cargo clippy              # Lint checks

# Clean builds
cargo clean                # Remove all build artifacts
cargo clean -p ros-z-msgs  # Clean specific package
```

```admonish warning
After changing feature flags or updating ROS 2, run `cargo clean -p ros-z-msgs` to force message regeneration.
```

## Troubleshooting Builds

**Build too slow?**

```bash
# Use parallel builds (automatic on most systems)
cargo build -j $(nproc)

# Build only what you need
cargo build -p ros-z-msgs --features std_msgs,geometry_msgs
```

**Can't find ROS packages?**

```bash
# Ensure ROS 2 is sourced
source /opt/ros/jazzy/setup.bash

# Verify environment
echo $AMENT_PREFIX_PATH

# Check package exists
ros2 pkg prefix example_interfaces
```

**Linker errors?**

```bash
# Clear cache and rebuild
cargo clean
source /opt/ros/jazzy/setup.bash
cargo build -p rcl-z
```

For comprehensive troubleshooting, see the [Troubleshooting Guide](./troubleshooting.md).

## Resources

- **[Feature Flags](./feature_flags.md)** - Detailed feature documentation
- **[Troubleshooting](./troubleshooting.md)** - Common build issues
- **[Message Generation](./message_generation.md)** - How messages work

**Start with the simplest build and add dependencies incrementally as your project grows.**
