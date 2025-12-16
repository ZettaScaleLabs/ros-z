# Feature Flags

ros-z uses Cargo feature flags to enable optional functionality and control dependencies. This allows you to build exactly what you need without unnecessary dependencies.

## Overview

Feature flags control three main aspects:

1. **Message packages** - Which ROS message types are available
2. **Serialization** - Additional serialization formats (protobuf)
3. **Integration** - RCL bindings and external dependencies

## ros-z Features

The main `ros-z` crate supports these features:

### `protobuf`

Enables protobuf serialization support using `prost`.

```bash
cargo build -p ros-z --features protobuf
```

**Use case:** When you need efficient binary serialization compatible with protobuf ecosystems.

**Dependencies:** Adds `prost` and related protobuf libraries.

### `rcl-z`

Enables RCL integration features for interoperability with C/C++ ROS 2 applications.

```bash
cargo build -p ros-z --features rcl-z
```

**Use case:** When building applications that need to work with existing RCL-based code.

**Dependencies:** Requires `rcl-z` package and ROS 2 installation.

### `external_msgs`

Enables examples that require external ROS message packages (propagates to `ros-z-msgs/external_msgs`).

```bash
cargo build -p ros-z --features external_msgs
```

**Use case:** Running examples that use message types not bundled with roslibrust (like `example_interfaces`).

**Dependencies:** Requires ROS 2 installation with the referenced message packages.

## ros-z-msgs Features

The `ros-z-msgs` crate generates Rust types from ROS message definitions. Features control which messages are generated.

### Default Features

The default build includes `common_interfaces`:

```bash
cargo build -p ros-z-msgs
```

This enables:

- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`

All of these work without a ROS 2 installation (uses bundled definitions).

### Bundled Message Features

These features use message definitions bundled with roslibrust (no ROS required):

#### `bundled_msgs`

Enables all bundled message packages:

```bash
cargo build -p ros-z-msgs --features bundled_msgs
```

Includes:

- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `nav_msgs`

#### Individual Bundled Packages

Enable specific message packages:

```bash
# Just std_msgs
cargo build -p ros-z-msgs --features std_msgs

# Just geometry_msgs
cargo build -p ros-z-msgs --features geometry_msgs

# Just sensor_msgs
cargo build -p ros-z-msgs --features sensor_msgs

# Just nav_msgs
cargo build -p ros-z-msgs --features nav_msgs
```

#### `common_interfaces` (default)

Convenience feature that enables:

- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`

```bash
cargo build -p ros-z-msgs --features common_interfaces
```

### External Message Features

These features require a ROS 2 installation:

#### External Message Packages

Enables all external message packages:

```bash
cargo build -p ros-z-msgs --features external_msgs
```

Currently includes:

- `example_interfaces`

**Requirements:** ROS 2 must be sourced and packages must be installed.

#### Individual External Packages

```bash
# Just example_interfaces
cargo build -p ros-z-msgs --features example_interfaces
```

#### `all_msgs`

Enables both bundled and external messages:

```bash
cargo build -p ros-z-msgs --features all_msgs
```

**Requirements:** ROS 2 installation required.

### Other Features

#### Protobuf Types

Generate protobuf types in addition to ROS message types:

```bash
cargo build -p ros-z-msgs --features protobuf
```

**Requirements:** Requires `ros-z/protobuf` feature to be enabled as well.

## ros-z-codegen Features

The `ros-z-codegen` crate provides code generation utilities.

### Protobuf Code Generation

Enable protobuf code generation support:

```bash
cargo build -p ros-z-codegen --features protobuf
```

**Use case:** When developing tools that need to generate protobuf code from ROS messages.

## ros-z-tests Features

The `ros-z-tests` integration test suite supports these features:

### `ros-msgs`

Enable tests with ros-z-msgs dependency:

```bash
cargo test -p ros-z-tests --features ros-msgs
```

### `interop-tests`

ROS interoperability tests (requires `ros-msgs`):

```bash
cargo test -p ros-z-tests --features interop-tests
```

**Requirements:** ROS 2 installation and `ros-msgs` feature.

## Common Feature Combinations

### Minimal Build

Just the core library:

```bash
cargo build -p ros-z
```

### Standard Development

Core library with common message types:

```bash
cargo build -p ros-z-msgs --features common_interfaces
cargo build -p ros-z
```

### Full ROS Integration

Everything including external messages:

```bash
cargo build -p ros-z-msgs --features all_msgs
cargo build -p ros-z --features external_msgs
cargo build -p rcl-z
```

### Protobuf Development

Core library with protobuf support:

```bash
cargo build -p ros-z-codegen --features protobuf
cargo build -p ros-z-msgs --features protobuf
cargo build -p ros-z --features protobuf
```

## Feature Dependencies

Some features depend on others:

```text
external_msgs (ros-z)
  └─> external_msgs (ros-z-msgs)

all_msgs (ros-z-msgs)
  ├─> bundled_msgs
  └─> external_msgs

common_interfaces (ros-z-msgs)
  ├─> std_msgs
  ├─> geometry_msgs
  └─> sensor_msgs

interop-tests (ros-z-tests)
  └─> ros-msgs
```

## Checking Active Features

To see which features are enabled:

```bash
# Show features for a specific package
cargo tree -p ros-z-msgs -e features

# Show all features in the workspace
cargo tree -e features
```

## Next Steps

- Learn how to [build with specific features](./building.md)
- Explore [examples](./examples_overview.md) that use different features
