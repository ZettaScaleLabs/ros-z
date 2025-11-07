# Protobuf Support in ros-z-msgs

This document describes the protobuf generation feature in ros-z-msgs.

## Overview

ros-z-msgs can generate **both** CDR-serializable (serde-based) and protobuf-serializable (prost-based) message types from ROS2 .msg files:

- **CDR types** (default): Located in the root modules (`geometry_msgs::Vector3`)
- **Protobuf types** (optional): Located in the `proto` module (`proto::geometry_msgs::Vector3`)

## Requirements

To build with protobuf support, you need:

1. **Protocol Buffers Compiler (`protoc`)**:
   ```bash
   # Debian/Ubuntu
   sudo apt-get install protobuf-compiler

   # macOS
   brew install protobuf

   # Or download from: https://github.com/protocolbuffers/protobuf/releases
   ```

2. **Enable the `protobuf` feature**:
   ```toml
   [dependencies]
   ros-z-msgs = { version = "*", features = ["protobuf"] }
   ```

## Generated Code

### .proto Files

During the build, ros-z-codegen:
1. Parses ROS2 .msg files using roslibrust
2. Generates .proto files for each package (e.g., `geometry_msgs.proto`)
3. Uses prost_build to compile .proto files to Rust code
4. Adds `MessageTypeInfo` and `ZMessage` trait implementations

Generated .proto files can be found in: `target/debug/build/ros-z-msgs-*/out/proto/`

Example generated proto:
```protobuf
syntax = "proto3";

package geometry_msgs;

message Vector3 {
  double x = 1;
  double y = 2;
  double z = 3;
}

message Twist {
  Vector3 linear = 1;
  Vector3 angular = 2;
}
```

### Rust Types

The protobuf types are generated with:
- `#[derive(Clone, PartialEq, prost::Message)]` from prost
- `impl MessageTypeInfo` for ROS2 type metadata
- `impl ZMessage` with `Serdes = ProtobufSerdes<T>`

## Usage

### CDR vs Protobuf

```rust
use ros_z::{Builder, context::ZContextBuilder, MessageTypeInfo};

// CDR serialization (default, compatible with ROS2 DDS)
use ros_z_msgs::geometry_msgs::Vector3 as Vector3Cdr;

let zpub_cdr = node.create_pub::<Vector3Cdr>("/topic")
    .with_type_info(Vector3Cdr::type_info())
    .build()?;  // Uses CdrSerdes by default

// Protobuf serialization (requires 'protobuf' feature)
#[cfg(feature = "protobuf")]
{
    use ros_z_msgs::proto::geometry_msgs::Vector3 as Vector3Proto;
    use ros_z::msg::ProtobufSerdes;

    let zpub_proto = node.create_pub::<Vector3Proto>("/topic_proto")
        .with_type_info(Vector3Proto::type_info())
        // ProtobufSerdes is automatically used via ZMessage impl
        .build()?;

    let msg = Vector3Proto { x: 1.0, y: 2.0, z: 3.0 };
    zpub_proto.publish(&msg)?;
}
```

### Why Two Type Systems?

- **CDR** (default): Binary compatible with ROS2 DDS communication
- **Protobuf**: Provides cross-language compatibility, versioning, and schema evolution

You can choose the serialization format based on your use case:
- Use CDR for ROS2 interoperability
- Use Protobuf for custom protocols or non-ROS systems

## Type Mapping

ROS types are mapped to protobuf as follows:

| ROS Type | Protobuf Type | Notes |
|----------|---------------|-------|
| bool | bool | |
| byte, uint8, char | uint32 | Proto3 has no uint8 |
| int8, int16 | int32 | Proto3 has no int8/int16 |
| uint16 | uint32 | |
| int32 | int32 | |
| uint32 | uint32 | |
| int64 | int64 | |
| uint64 | uint64 | |
| float32 | float | |
| float64 | double | |
| string | string | |
| T[] | repeated T | All ROS arrays become repeated |
| OtherMsg | OtherMsg | Message types referenced directly |

## Limitations

1. **Lossy Type Mapping**: Some ROS types (uint8, int8, int16, uint16) are widened in protobuf
2. **Array Bounds**: ROS bounded arrays (`T[<=N]`) become unbounded `repeated T` in proto
3. **Requires protoc**: The Protocol Buffers compiler must be installed
4. **Build Time**: Protobuf generation adds to compile time

## Architecture

```
.msg file
    ↓ roslibrust_codegen
    ├─→ CDR code (serde) ──→ geometry_msgs::Vector3
    │
    └─→ .proto file ──→ prost_build ──→ proto::geometry_msgs::Vector3
                              ↓
                        + MessageTypeInfo impl
                        + ZMessage impl
```

## Troubleshooting

### Error: "Could not find `protoc`"

Install the Protocol Buffers compiler:
```bash
sudo apt-get install protobuf-compiler  # Debian/Ubuntu
brew install protobuf                   # macOS
```

Or set the `PROTOC` environment variable to the path of your protoc binary.

### Why are there two Vector3 types?

This is intentional! CDR and protobuf types are separate:
- `geometry_msgs::Vector3` - uses serde (CDR-compatible)
- `proto::geometry_msgs::Vector3` - uses prost (protobuf)

They have the same fields but different serialization.

## Future Work

- Support for ROS2 services with protobuf
- Cross-package message references in proto files
- Protobuf schema validation
- Automatic field number stability
