# Building ros-z

**ros-z is designed to work without ROS 2 dependencies by default, enabling pure Rust development while optionally integrating with existing ROS 2 installations.** This flexible approach lets you choose your dependency level based on project requirements.

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

**Use when:** You need standard ROS 2 message types (no ROS 2 installation required)

**Add to your `Cargo.toml`:**

```toml
[dependencies]
ros-z = "0.x"
ros-z-msgs = "0.x"  # Includes core_msgs by default
tokio = { version = "1", features = ["full"] }
```

**Default message packages (core_msgs):**

- `std_msgs` - Primitive types (String, Int32, Float64, etc.)
- `geometry_msgs` - Spatial data (Point, Pose, Transform, Twist)
- `sensor_msgs` - Sensor data (LaserScan, Image, Imu, PointCloud2)
- `nav_msgs` - Navigation (Path, OccupancyGrid, Odometry)
- `example_interfaces` - Tutorial services (AddTwoInts)
- `action_tutorials_interfaces` - Tutorial actions (Fibonacci)

### Scenario 3: All Message Packages

**Use when:** You need all available message types including test messages

**Requirements:** None (all messages are vendored)

**Add to your `Cargo.toml`:**

```toml
[dependencies]
ros-z = "0.x"
ros-z-msgs = { version = "0.x", features = ["all_msgs"] }
tokio = { version = "1", features = ["full"] }
```

**Build your project:**

```bash
cargo build
```

**All available packages:**

- `std_msgs` - Basic types
- `geometry_msgs` - Spatial data
- `sensor_msgs` - Sensor data
- `nav_msgs` - Navigation
- `example_interfaces` - Tutorial services (AddTwoInts)
- `action_tutorials_interfaces` - Tutorial actions (Fibonacci)
- `test_msgs` - Test types

```admonish tip
The default `core_msgs` feature includes everything except `test_msgs`. Use `all_msgs` only if you need test message types.
```

## ROS 2 Distribution Compatibility

**ros-z defaults to ROS 2 Jazzy compatibility**, which is the recommended distribution for new projects. If you need to target a different distribution like Humble, see the [ROS 2 Distribution Compatibility](./distro_compatibility.md) chapter for detailed instructions.

**Quick reference:**

```bash
# Default (Jazzy) - works out of the box
cargo build

# For Humble - use --no-default-features
cargo build --no-default-features --features humble

# For Rolling/Iron - just add the feature
cargo build --features rolling
```

The distribution choice affects type hash support and interoperability with ROS 2 nodes. See the [Distribution Compatibility chapter](./distro_compatibility.md) for full details.

## Development

This section is for contributors working on ros-z itself. If you're using ros-z in your project, you can skip this section.

### Package Organization

The ros-z repository is organized as a Cargo workspace with multiple packages:

| Package | Default Build | Purpose | Dependencies |
|---------|---------------|---------|--------------|
| **ros-z** | Yes | Core Zenoh-native ROS 2 library | None |
| **ros-z-codegen** | Yes | Message generation utilities | None |
| **ros-z-msgs** | No | Pre-generated message types | None (all vendored) |
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
3. Bundled assets (built-in message definitions in ros-z-codegen)

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

## Next Steps

- **[ROS 2 Distribution Compatibility](./distro_compatibility.md)** - Target Jazzy, Humble, or other distributions
- **[Running Examples](./examples.md)** - Try out the included examples
- **[Networking](./networking.md)** - Set up Zenoh router and session config
- **[Message Generation](./message_generation.md)** - Understand how messages work
- **[Troubleshooting](./troubleshooting.md)** - Solutions to common build issues

**Start with the simplest build and add dependencies incrementally as your project grows.**
