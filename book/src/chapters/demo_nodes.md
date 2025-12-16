# Demo Nodes

The demo nodes examples showcase common ROS 2 patterns in ros-z. These examples are compatible with the standard ROS 2 demo nodes, allowing interoperability testing.

## Available Demos

### Publisher/Subscriber

- **[Talker](./demo_talker.md)**: Publishes "Hello World" messages to the `chatter` topic
- **[Listener](./demo_listener.md)**: Subscribes to the `chatter` topic and prints received messages

These demonstrate the basic pub/sub pattern with:

- Custom QoS profiles
- Configurable publish rates
- Optional message count limits (useful for testing)

### Services

- **[Add Two Ints Server](./demo_add_two_ints_server.md)**: Provides an addition service
- **[Add Two Ints Client](./demo_add_two_ints_client.md)**: Calls the addition service

These demonstrate the request-response service pattern with:

- Synchronous service calls
- Request/response handling
- Error management

## Running the Demos

All demos support command-line arguments for configuration:

```bash
# See available options
cargo run --example demo_nodes_talker -- --help

# Run with custom settings
cargo run --example demo_nodes_talker -- --topic /custom --period 0.5
```

## Common Options

Most demos support these options:

- `--endpoint`: Zenoh router endpoint (e.g., `tcp/localhost:7447`)
- `--mode`: Zenoh session mode (`peer`, `client`, or `router`)

## Interoperability

These demos are designed to work with standard ROS 2 demo nodes:

```bash
# Terminal 1: ros-z talker
cargo run --example demo_nodes_talker

# Terminal 2: Standard ROS 2 listener
ros2 run demo_nodes_cpp listener
```

This allows testing ros-z compatibility with the broader ROS 2 ecosystem.

## Demo Source Files

All demo source files are located in `ros-z/examples/demo_nodes/` and are well-documented with inline comments. They serve as both runnable examples and test fixtures.
