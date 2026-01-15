# ROS 2 Distribution Compatibility

ros-z supports multiple ROS 2 distributions through compile-time feature flags. This chapter explains the differences between distributions and how to target specific ROS 2 versions.

## Supported Distributions

| Distribution | Status | Type Hash Support | Default |
|--------------|--------|-------------------|---------|
| **Jazzy Jalisco** | ✅ Fully Supported | ✅ Yes | **Yes** |
| **Humble Hawksbill** | ✅ Supported | ❌ No (placeholder) | No |
| Rolling Ridley | ✅ Supported | ✅ Yes | No |
| Iron Irwini | ✅ Supported | ✅ Yes | No |

**Default**: ros-z defaults to **Jazzy** compatibility, which is the recommended distribution for new projects.

## Distribution Differences

### Type Hash Support

The most significant difference between distributions is **type hash support**:

**Jazzy/Rolling/Iron** (Modern):

- Supports real type hashes computed from message definitions
- Format: `RIHS01_<64-hex-chars>` (ROS IDL Hash Standard version 1)
- Enables type safety checks during pub/sub matching
- Type hashes are embedded in Zenoh key expressions for discovery

**Humble** (Legacy):

- Does not support real type hashes
- Uses constant placeholder: `"TypeHashNotSupported"`
- No type safety validation during discovery
- Compatible with rmw_zenoh_cpp v0.1.8

### Example Key Expressions

**Jazzy:**

```text
@ros2_lv/0/<zid>/<nid>/<eid>/MP/%/<namespace>/<node>/chatter/std_msgs%msg%String/RIHS01_1234567890abcdef.../...
```

**Humble:**

```text
@ros2_lv/0/<zid>/<nid>/<eid>/MP/%/<namespace>/<node>/chatter/std_msgs%msg%String/TypeHashNotSupported/...
```

## Building for Different Distributions

### Using Jazzy (Default)

By default, ros-z builds with Jazzy compatibility. No special flags needed:

```bash
# Build with default (Jazzy)
cargo build

# Run examples
cargo run --example demo_nodes_talker

# Run tests
cargo nextest run
```

### Using Humble

To build for Humble, use `--no-default-features --features humble`:

```bash
# Build for Humble
cargo build --no-default-features --features humble

# Build with additional features
cargo build --no-default-features --features humble,external_msgs

# Run examples for Humble
cargo run --no-default-features --features humble --example demo_nodes_talker

# Run tests for Humble
cargo nextest run --no-default-features --features humble
```

### Using Other Distributions

For Rolling or Iron, simply specify the distro feature:

```bash
# Build for Rolling
cargo build --features rolling

# Build for Iron
cargo build --features iron
```
