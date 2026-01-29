# Protobuf Demo

This example demonstrates two ways to use protobuf serialization with ros-z:

## 1. ROS Messages with Protobuf

Uses auto-generated ROS message types from `ros-z-msgs` with protobuf serialization:

- Messages are defined in ROS .msg files
- Automatically implements `MessageTypeInfo` trait
- Generated with both CDR and Protobuf support

## 2. Custom Protobuf Messages

Uses custom protobuf messages defined in `.proto` files:

- Pure protobuf messages (not ROS-specific)
- Demonstrates ros-z's transport-agnostic design
- Can send ANY protobuf message over Zenoh

## Running the Example

```bash
cd examples/protobuf_demo
cargo run
```

## Key Files

- `proto/sensor_data.proto` - Custom protobuf message definition
- `build.rs` - Compiles .proto files at build time
- `src/main.rs` - Demonstrates both approaches

## How It Works

The example:

1. Publishes ROS `geometry_msgs/Vector3` using protobuf serialization
2. Publishes custom `SensorData` messages (pure protobuf)
3. Both use `.with_serdes::<ProtobufSerdes<T>>()` to select protobuf format

This shows that ros-z can work with:

- ROS standard messages
- Custom ROS messages
- Pure protobuf messages (no ROS dependency)
