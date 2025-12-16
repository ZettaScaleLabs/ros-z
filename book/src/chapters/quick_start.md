# Quick Start

This guide will help you get started with ros-z quickly.

## Installation

Add ros-z to your `Cargo.toml`:

```toml
[dependencies]
ros-z = "*"
ros-z-msgs = "*"
```

## Basic Pub/Sub Example

Here's a complete example showing both a publisher and subscriber:

NOTE: An async runtime is needed. Tokio is used in this example.

```rust,ignore
{{#include ../../../ros-z/examples/z_pubsub.rs}}
```


## Running the Example

Build and run the examples:

```bash
# Build the project
cargo build

# In terminal 1 - start the listener
cargo run --example z_pubsub -- -r listener

# In terminal 2 - start the talker
cargo run --example z_pubsub -- -r talker
```

## Key Concepts

- **ZContext**: The main entry point for creating ROS nodes. Create it using `ZContextBuilder`.
- **Node**: A ROS node created from a context using `ctx.create_node("name")`.
- **Publisher**: Created with `node.create_pub::<MessageType>("topic")` for publishing messages.
- **Subscriber**: Created with `node.create_sub::<MessageType>("topic")` for receiving messages.

## Next Steps

- Learn about [Publishers and Subscribers](./pubsub.md)
- Explore [Services](./services.md)
- Try the [Demo Nodes](./demo_nodes.md)
