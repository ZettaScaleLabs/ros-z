# Demo Listener

The listener demo is a simple subscriber node that receives messages from a topic.

## Complete Example

```rust,no_run
{{#include ../../../ros-z/examples/demo_nodes/listener.rs}}
```

## Key Points

- **Line 27**: Creates a node named "listener"
- **Line 31-34**: Configures QoS with `KeepLast(10)` history depth
- **Line 35**: Creates a subscriber for `RosString` messages on the topic
- **Line 41-79**: Main receive loop with optional timeout and max count support
- **Line 50-55**: Uses `recv_timeout()` for bounded waits or `async_recv()` for indefinite waits
- **Line 58-68**: Processes received messages and tracks them in a vector
- **Line 25**: Returns all received messages, making it testable

## Usage

Run the listener:

```bash
cargo run --example demo_nodes_listener
```

With custom options:

```bash
# Listen to a different topic
cargo run --example demo_nodes_listener -- --topic /my_topic

# Connect to a specific Zenoh router
cargo run --example demo_nodes_listener -- --endpoint tcp/localhost:7447
```

## Features

The listener example demonstrates:

- **Testability**: The `run_listener()` function can be used in tests with `max_count` and `timeout` parameters
- **Flexible Receiving**: Supports both indefinite listening and bounded operation
- **Message Tracking**: Returns all received messages for verification

## Pairing with Talker

This example works perfectly with the [talker demo](./demo_talker.md). Run them together:

```bash
# Terminal 1 - start listener first
cargo run --example demo_nodes_listener

# Terminal 2 - start talker
cargo run --example demo_nodes_talker
```

The listener will print each message it receives from the talker.
