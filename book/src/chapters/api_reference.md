# API Reference

For detailed API documentation, see the generated Rust docs:

```bash
cargo doc --open
```

## Core Modules

### `ros_z::context`

- `ZContext`: The main context for creating nodes
- `ZContextBuilder`: Builder for configuring contexts

### `ros_z::node`

- Node creation and management
- Publisher/subscriber/service/client creation

### `ros_z::qos`

- `QosProfile`: Quality of Service configuration
- `QosHistory`: Message history policies
- `QosReliability`: Reliability policies
- `QosDurability`: Durability policies

### `ros_z::Builder`

The `Builder` trait provides the fluent API for creating publishers, subscribers, services, and clients.

## Message Packages

### `ros_z_msgs`

Standard ROS 2 messages organized by package:

- `std_msgs`: Basic message types
- `geometry_msgs`: Geometric primitives
- `sensor_msgs`: Sensor data types
- `example_interfaces`: Example service types

## Online Resources

- [GitHub Repository](https://github.com/ZettaScaleLabs/ros-z)
- [Zenoh Documentation](https://zenoh.io/docs/)
- [ROS 2 Documentation](https://docs.ros.org/)
