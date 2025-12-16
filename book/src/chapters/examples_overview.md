# Examples Overview

ros-z provides a comprehensive set of examples demonstrating various ROS 2 patterns and use cases. All examples are runnable via Cargo and showcase real-world ROS 2 functionality.

## Running Examples

All examples follow the standard Cargo pattern:

```bash
cargo run --example <example_name> -- [OPTIONS]
```

For example:

```bash
# Run the talker with default settings
cargo run --example demo_nodes_talker

# Run with custom options
cargo run --example demo_nodes_talker -- --topic /my_topic --period 0.5

# Run a service client
cargo run --example demo_nodes_add_two_ints_client -- --a 10 --b 20
```

## Directory Structure

```text
examples/
├── demo_nodes/           # Classic ROS demo implementations
│   ├── talker.rs         # Publisher demo
│   ├── listener.rs       # Subscriber demo
│   ├── add_two_ints_server.rs    # Service server
│   └── add_two_ints_client.rs    # Service client
├── z_pubsub.rs          # Combined pub/sub example
├── z_srvcli.rs          # Combined service example
├── z_custom_message.rs  # Custom message definition
├── z_pingpong.rs        # Latency measurement tool
├── twist_pub.rs         # Twist message publisher
├── battery_state_sub.rs # Battery state subscriber
└── laser_scan.rs        # Laser scan pub/sub
```

## Example Categories

### Demo Nodes

Classic ROS 2 demo implementations:

- **[Talker](./demo_talker.md)**: Simple string publisher
- **[Listener](./demo_listener.md)**: Simple string subscriber
- **[Add Two Ints Server](./demo_add_two_ints_server.md)**: Service server example
- **[Add Two Ints Client](./demo_add_two_ints_client.md)**: Service client example

Run a complete pub/sub demo:

```bash
# Terminal 1 - Start listener
cargo run --example demo_nodes_listener

# Terminal 2 - Start talker
cargo run --example demo_nodes_talker

# Terminal 3 - Verify with ROS 2 CLI (if available)
ros2 topic echo /chatter
```

### Basic Examples

Self-contained examples demonstrating core patterns:

- **z_pubsub.rs**: Combined publisher/subscriber in one program
- **z_srvcli.rs**: Combined service server/client in one program
- **z_custom_message.rs**: Demonstrates custom message definitions

### Advanced Examples

Real-world ROS 2 message types and patterns:

- **[Twist Publisher](./twist_pub.md)**: Velocity commands (geometry_msgs)
- **[Battery State Subscriber](./battery_state_sub.md)**: Battery monitoring (sensor_msgs)
- **[Laser Scan](./laser_scan.md)**: Lidar data handling (sensor_msgs)
- **[Zenoh PingPong](./z_pingpong.md)**: Latency measurement and performance testing

## Testing Examples

All examples are tested as part of the test suite:

```bash
# Test library and all examples
cargo test --all

# Run a specific example
cargo run --example demo_nodes_talker

# Build all examples
cargo build --examples
```

## Common Options

Most examples support these command-line options:

- `--endpoint <ENDPOINT>`: Connect to specific Zenoh router (e.g., `tcp/localhost:7447`)
- `--topic <TOPIC>`: Use custom topic name
- `--mode <MODE>`: Select operation mode (varies by example)
- `--help`: Show all available options

Examples:

```bash
# Connect to specific router
cargo run --example demo_nodes_talker -- --endpoint tcp/192.168.1.100:7447

# Use custom topic
cargo run --example demo_nodes_listener -- --topic /my_custom_topic
```

## Interoperability with ROS 2

All examples are compatible with standard ROS 2 tools when connected to the same Zenoh network:

```bash
# List available topics
ros2 topic list

# Echo messages from ros-z talker
ros2 topic echo /chatter

# Call ros-z service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# Publish to ros-z listener from ROS 2
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from ROS 2'"
```

## Next Steps

- Start with the [Demo Nodes](./demo_nodes.md) to understand basic patterns
- Explore [Advanced Examples](./advanced.md) for real-world message types
- Review [Message Generation](./message_generation.md) to understand how message types are created
