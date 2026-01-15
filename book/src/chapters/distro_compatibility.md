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

### Other Differences

| Feature | Humble | Jazzy/Rolling |
|---------|--------|---------------|
| Shared Memory (SHM) | ❌ Not supported | ✅ Supported (>3KB messages) |
| Query timeout | 5 seconds | 10 minutes (600 seconds) |
| Default durability | TransientLocal | Volatile |
| rmw_zenoh version | v0.1.8 | v0.2.9+ |

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

## Using with Nix

The Nix flake provides separate development shells for each distribution:

```bash
# Enter Jazzy shell (default)
nix develop .#ros-jazzy

# Enter Humble shell
nix develop .#ros-humble

# Build in Humble shell
nix develop .#ros-humble -c cargo build --no-default-features --features humble
```

Inside the Nix shell, the ROS 2 environment is automatically configured with the correct version.

## Package-Specific Configuration

### Library Crates (`ros-z`, `ros-z-msgs`)

When depending on ros-z in your `Cargo.toml`:

```toml
[dependencies]
# Default (Jazzy)
ros-z = "0.1"

# For Humble support, users enable the feature:
# cargo build --no-default-features --features humble
ros-z = { version = "0.1", default-features = false }
```

### Application Crates

If your application needs to support multiple distributions:

```toml
[dependencies]
ros-z = { version = "0.1", default-features = false }

[features]
default = ["jazzy"]
humble = ["ros-z/humble"]
jazzy = ["ros-z/jazzy"]
rolling = ["ros-z/rolling"]
```

Then users can build with:

```bash
# For Jazzy (default)
cargo build

# For Humble
cargo build --no-default-features --features humble
```

## Cross-Distribution Communication

### Same Distribution

Communication between ros-z nodes and ROS 2 nodes works seamlessly when using the same distribution:

| ros-z Distro | ROS 2 Distro | Result |
|--------------|--------------|--------|
| Jazzy | Jazzy | ✅ Full compatibility |
| Humble | Humble | ✅ Full compatibility |

### Cross-Distribution

Cross-distribution communication is **limited**:

| ros-z Distro | ROS 2 Distro | Result |
|--------------|--------------|--------|
| Jazzy | Humble | ⚠️ May not discover each other |
| Humble | Jazzy | ⚠️ May not discover each other |

**Why?** The type hash format in Zenoh key expressions differs:

- Jazzy nodes advertise with `RIHS01_<hash>`
- Humble nodes advertise with `TypeHashNotSupported`
- Subscribers filter by exact key expression match, causing discovery to fail

## CI/CD Integration

### GitHub Actions Example

For testing multiple distributions in CI:

```yaml
jobs:
  test:
    strategy:
      matrix:
        distro: [humble, jazzy]
    steps:
      - uses: actions/checkout@v4

      - name: Build and test
        run: |
          if [ "${{ matrix.distro }}" = "humble" ]; then
            cargo build --no-default-features --features humble
            cargo nextest run --no-default-features --features humble
          else
            cargo build --features ${{ matrix.distro }}
            cargo nextest run --features ${{ matrix.distro }}
          fi

```

## Interoperability Testing

The ros-z test suite includes interoperability tests with actual ROS 2 nodes:

```bash
# Test Jazzy interop (requires ROS 2 Jazzy + rmw_zenoh_cpp)
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
cargo nextest run -p ros-z-tests --features interop-tests,jazzy

# Test Humble interop (requires ROS 2 Humble + rmw_zenoh_cpp)
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
cargo nextest run -p ros-z-tests --no-default-features --features interop-tests,humble
```

These tests verify:

- ros-z publisher → ROS 2 subscriber
- ROS 2 publisher → ros-z subscriber
- ros-z service client → ROS 2 service server
- ROS 2 service client → ros-z service server
- ros-z action client → ROS 2 action server
- ROS 2 action client → ros-z action server

## Choosing a Distribution

### For New Projects

#### Recommendation: Use Jazzy (default)

Jazzy is the latest stable release with:

- ✅ Modern type hash support for type safety
- ✅ Shared memory optimization for large messages
- ✅ Better performance with longer query timeouts
- ✅ Active development and support

### For Existing Projects

**Use Humble if:**

- You have existing ROS 2 Humble infrastructure
- You need LTS (Long Term Support) until 2027
- You're targeting Ubuntu 22.04 (Jammy)

**Migrate to Jazzy if:**

- You want the latest features
- You can upgrade to Ubuntu 24.04 (Noble)
- You need shared memory support

## Troubleshooting

### Feature Conflict Error

```text
error: the following features cannot be enabled at the same time
```

**Solution:** Use `--no-default-features` when enabling Humble:

```bash
cargo build --no-default-features --features humble
```

### Discovery Failures Between Distros

**Symptom:** ros-z Jazzy node can't see ROS 2 Humble node

**Cause:** Type hash mismatch in key expressions

**Solution:** Use matching distributions on both sides:

- ros-z Humble ↔ ROS 2 Humble
- ros-z Jazzy ↔ ROS 2 Jazzy

### Missing Type Hash in Humble

**Symptom:** Warnings about missing type descriptions when using Humble

**Expected behavior:** Humble doesn't support type hashes. These warnings are informational and can be ignored.

## Implementation Details

For developers interested in the technical implementation:

- Type hashes are handled by the `TypeHash` struct in `ros-z/src/entity.rs`
- Humble detection uses `#[cfg(feature = "humble-compat")]`
- Key expression generation adapts based on `#[cfg(feature = "no-type-hash")]`
- Distro defaults are defined in `ros-z/src/config.rs`

See [`HUMBLE_SUPPORT.md`](../../HUMBLE_SUPPORT.md) for detailed implementation notes.

## Summary

- **Default: Jazzy** - Recommended for new projects
- **Humble: Legacy** - Use `--no-default-features --features humble`
- **Type hashes** - Key difference affecting discovery
- **Same distro** - Required for reliable communication
- **Nix shells** - Provide pre-configured environments per distro
