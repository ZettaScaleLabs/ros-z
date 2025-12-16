# Message Generation

ros-z provides a powerful code generation system that automatically creates Rust types from ROS 2 message definitions. This chapter explains how message generation works and how to use it effectively.

## Overview

Message generation in ros-z converts `.msg`, `.srv`, and `.action` files into Rust structs with proper serialization support and type information. The system is designed to work both with and without a ROS 2 installation.

**Key features:**

- Automatic code generation at build time
- Support for both bundled and system ROS messages
- CDR serialization compatible with ROS 2 DDS
- Optional protobuf serialization
- Proper type hashing for ROS 2 interoperability

## Architecture

The message generation system consists of several components:

### Components

#### roslibrust_codegen

- Third-party library for parsing `.msg` and `.srv` files
- Resolves message dependencies and calculates ROS 2 type hashes
- Generates base Rust structs with serde support
- Bundled message definitions available without ROS installation

#### ros-z-codegen

- ros-z's code generation orchestrator
- Coordinates message discovery and generation
- Provides adapters for different serialization formats
- Generates ros-z-specific trait implementations

#### Adapters

- `roslibrust_adapter`: Generates CDR-compatible types (default)
- `protobuf_adapter`: Generates protobuf types (optional)

### Generated Traits

For each message type, ros-z generates:

```rust,ignore
impl MessageTypeInfo for PackageName::MessageName {
    fn type_name() -> &'static str {
        "package_name::msg::dds_::MessageName_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::from_rihs_string("RIHS01_...")
            .expect("Invalid RIHS hash string")
    }

    fn type_info() -> TypeInfo {
        TypeInfo::new(Self::type_name(), Self::type_hash())
    }
}

impl WithTypeInfo for PackageName::MessageName {}
```

These traits enable ros-z to:

- Identify message types at runtime
- Validate type compatibility with other ROS 2 nodes
- Generate proper DDS topic names

## Build-Time Generation

Message generation happens during the build process via `build.rs` scripts.

### The ros-z-msgs Build Process

The `ros-z-msgs` crate's build script:

1. **Discovers packages** based on enabled features
2. **Searches for message definitions** in this order:
   - System ROS installation (`AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`)
   - Common ROS paths (`/opt/ros/{rolling,jazzy,iron,humble}`)
   - Roslibrust bundled assets (`~/.cargo/git/checkouts/roslibrust-*/assets/`)
3. **Generates Rust code** with proper trait implementations
4. **Outputs to** `$OUT_DIR/generated.rs`

### Package Discovery

```rust,ignore
// Simplified build.rs flow
fn discover_ros_packages() -> Vec<PathBuf> {
    let bundled = get_bundled_packages();  // std_msgs, geometry_msgs, etc.
    let external = get_external_packages(); // example_interfaces, etc.

    // Try system installation first
    if let Some(packages) = discover_system_packages(&bundled, &external) {
        return packages;
    }

    // Fallback to bundled roslibrust assets
    discover_bundled_packages(&bundled)
}
```

### Configuration

The generator is configured via `GeneratorConfig`:

```rust,ignore
let config = GeneratorConfig {
    generate_cdr: true,        // CDR-compatible types (default)
    generate_protobuf: false,  // Protobuf types (optional)
    generate_type_info: true,  // MessageTypeInfo traits
    output_dir: out_dir,
};
```

## Using ros-z-msgs

The `ros-z-msgs` crate provides pre-generated message types.

### Basic Usage

```rust,ignore
use ros_z::context::ZContextBuilder;
use ros_z_msgs::ros::std_msgs::String as RosString;

let ctx = ZContextBuilder::default().build()?;
let node = ctx.create_node("my_node").build()?;
let pub = node.create_pub::<RosString>("/topic").build()?;

let msg = RosString {
    data: "Hello, ROS!".to_string(),
};
pub.publish(&msg)?;
```

### Available Message Packages

See [Feature Flags](./feature_flags.md) for the complete list of available packages.

**Bundled packages** (no ROS required):

- `std_msgs` - Standard message types
- `geometry_msgs` - Geometry primitives
- `sensor_msgs` - Sensor data types
- `nav_msgs` - Navigation messages

**External packages** (requires ROS 2):

- `example_interfaces` - Example services and actions
- Custom packages from your ROS installation

### Namespace Structure

Generated messages are organized by package:

```rust,ignore
ros_z_msgs::ros::std_msgs::String
ros_z_msgs::ros::geometry_msgs::Twist
ros_z_msgs::ros::sensor_msgs::LaserScan
```

Services follow a similar pattern:

```rust,ignore
ros_z_msgs::ros::example_interfaces::AddTwoInts
ros_z_msgs::ros::example_interfaces::AddTwoIntsRequest
ros_z_msgs::ros::example_interfaces::AddTwoIntsResponse
```

## Manual Custom Messages

For custom message types without generating from `.msg` files, you can manually implement the required traits.

### Defining a Custom Message

```rust,ignore
use serde::{Deserialize, Serialize};
use ros_z::{MessageTypeInfo, WithTypeInfo, entity::TypeHash};

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct RobotStatus {
    pub robot_id: String,
    pub battery_percentage: f64,
    pub position_x: f64,
    pub position_y: f64,
    pub is_moving: bool,
}

impl MessageTypeInfo for RobotStatus {
    fn type_name() -> &'static str {
        "custom_msgs::msg::dds_::RobotStatus_"
    }

    fn type_hash() -> TypeHash {
        // Use zero hash for custom messages
        // Or generate proper hash if interoperating with ROS 2
        TypeHash::zero()
    }
}

impl WithTypeInfo for RobotStatus {}
```

### When to Use Manual Messages

**Use manual messages when:**

- Prototyping new message types
- Building standalone applications
- You don't need ROS 2 type compatibility

**Use generated messages when:**

- Interoperating with existing ROS 2 systems
- Using standard ROS message types
- You need proper type hashing

### Type Hash Considerations

The `type_hash()` is critical for ROS 2 interoperability:

- **`TypeHash::zero()`**: Works for ros-z-to-ros-z communication
- **Proper RIHS hash**: Required for interoperability with other ROS 2 nodes
- Generated messages automatically have correct hashes

For a complete working example with custom messages and services, see the [Custom Messages](./custom_messages.md) section.

## Serialization Formats

ros-z supports multiple serialization formats.

### CDR (Default)

Common Data Representation is the standard ROS 2 serialization format:

- Full compatibility with ROS 2 DDS
- Efficient binary serialization
- Used by all ROS 2 implementations
- Enabled by default

Generated types use standard Rust `serde` with CDR encoding:

```rust,ignore
// Generated code uses serde
#[derive(Serialize, Deserialize)]
pub struct String {
    pub data: std::string::String,
}
```

### Protobuf (Optional)

Protocol Buffers support is available as an optional feature:

```bash
cargo build -p ros-z-msgs --features protobuf
```

**Benefits:**

- Schema evolution support
- Efficient binary encoding
- Language-agnostic format
- Familiar to many developers

**Tradeoffs:**

- Not standard ROS 2 format
- Requires feature flag
- Additional dependencies

The protobuf adapter generates `.proto` files and corresponding Rust types:

```rust,ignore
// Generated protobuf types have the same traits
impl MessageTypeInfo for proto::std_msgs::String { ... }
```

## Extending ros-z-msgs

To add new message packages to ros-z-msgs:

### 1. Add Feature Flag

Edit `ros-z-msgs/Cargo.toml`:

```toml
[features]
# Add your package to bundled or external
bundled_msgs = ["std_msgs", "geometry_msgs", "your_package"]
your_package = []
```

### 2. Update Build Script

Edit `ros-z-msgs/build.rs`:

```rust,no_run
fn get_bundled_packages() -> Vec<&'static str> {
    let mut names = vec!["builtin_interfaces"];

    #[cfg(feature = "your_package")]
    names.push("your_package");

    names
}
```

### 3. Rebuild

```bash
cargo build -p ros-z-msgs --features your_package
```

The build system will:

- Search for the package in ROS installation or roslibrust assets
- Parse all message definitions
- Generate Rust types with proper traits
- Output to the generated module

## Advanced Topics

### Filtering Messages

The code generator automatically filters:

- **Deprecated actionlib messages** - Old ROS 1 action format
- **wstring fields** - Poor Rust support
- **Duplicate definitions** - Keeps first occurrence

### Custom Code Generation

For custom build scripts, use `ros-z-codegen` directly:

```rust,ignore
use ros_z_codegen::{MessageGenerator, GeneratorConfig};

let config = GeneratorConfig {
    generate_cdr: true,
    generate_protobuf: false,
    generate_type_info: true,
    output_dir: out_dir.clone(),
};

let generator = MessageGenerator::new(config);
generator.generate_from_msg_files(&package_paths)?;
```

### Type Hash Calculation

ros-z uses the RIHS (ROS IDL Hash) algorithm for type hashing:

- Hashes include message structure and dependencies
- Changes to message definitions change the hash
- Ensures type safety across ROS 2 network
- Automatically calculated during code generation

The hash appears in generated code:

```rust,ignore
TypeHash::from_rihs_string("RIHS01_1234567890abcdef...")
```

## Troubleshooting

### Package Not Found

If a package isn't found during generation:

1. Check ROS 2 is sourced: `echo $AMENT_PREFIX_PATH`
2. Verify package is installed: `ros2 pkg list | grep your_package`
3. For bundled packages, ensure roslibrust git dependency is present

### Build Failures

Common issues:

- **Missing dependencies**: Enable required feature flags
- **Type conflicts**: Check for duplicate manual message definitions
- **Hash errors**: Usually indicates roslibrust version mismatch

See [Troubleshooting](./troubleshooting.md) for more solutions.

## Examples

### Using Generated Messages

The [demo_talker](./demo_talker.md) example demonstrates generated message usage:

```rust,ignore
{{#include ../../../ros-z/examples/demo_nodes/talker.rs}}
```

## Next Steps

- Review [Feature Flags](./feature_flags.md) for available message packages
- Explore [Building](./building.md) for build configuration options
- Check [Publishers and Subscribers](./pubsub.md) for using messages in your applications
