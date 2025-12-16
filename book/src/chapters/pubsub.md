# Publishers and Subscribers

The publish-subscribe pattern is the most common communication mechanism in ROS 2. ros-z provides a simple and type-safe API for creating publishers and subscribers.

## Overview

- **Publishers** send messages to a topic
- **Subscribers** receive messages from a topic
- Messages are strongly typed using Rust structs
- Multiple publishers and subscribers can use the same topic

## Basic Pattern

The typical workflow is:

1. Create a `ZContext` using `ZContextBuilder`
2. Create a node from the context
3. Create publishers and/or subscribers on the node
4. Publish or receive messages

## Quality of Service (QoS)

ros-z supports ROS 2 QoS profiles to control message delivery:

- **History**: Keep last N messages or all messages
- **Reliability**: Best effort or reliable delivery
- **Durability**: Volatile or transient local

Example with custom QoS:

```rust,ignore,ignore
use ros_z::qos::{QosProfile, QosHistory};

let qos = QosProfile {
    history: QosHistory::KeepLast(10),
    ..Default::default()
};

let publisher = node
    .create_pub::<RosString>("topic")
    .with_qos(qos)
    .build()?;
```

## Examples

- [Simple Publisher and Subscriber](./simple_pub.md) - Combined talker/listener example
- [Demo Talker](./demo_talker.md) - Dedicated publisher node
- [Demo Listener](./demo_listener.md) - Dedicated subscriber node
- [Custom Messages](./custom_messages.md) - Using custom message types

## Async vs Blocking

ros-z provides both async and blocking APIs:

- `async_publish()` / `async_recv()` - For async/await code
- `publish()` / `recv()` / `recv_timeout()` - For blocking code

Choose based on your application architecture and performance requirements.
