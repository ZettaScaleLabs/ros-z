# Introduction

**ros-z is a native Rust ROS 2 implementation powered by Zenoh, delivering high-performance robotics communication with type safety and zero-cost abstractions.** Build reliable robot applications using modern Rust idioms while maintaining full ROS 2 compatibility.

```admonish success
ros-z combines Rust's safety guarantees with Zenoh's efficient networking to create a modern, performant alternative to traditional ROS 2 middleware. Perfect for production robotics applications.
```

## Core Capabilities

| Feature | Description | Benefit |
|---------|-------------|---------|
| **Native Rust** | Pure Rust implementation with no C/C++ dependencies | Memory safety, concurrency without data races |
| **Zenoh Transport** | High-performance pub-sub engine | Low latency, efficient bandwidth usage |
| **ROS 2 Compatible** | Works seamlessly with standard ROS 2 tools | Integrate with existing robotics ecosystems |
| **Type Safety** | Compile-time message validation | Catch errors before deployment |
| **Modern API** | Idiomatic Rust patterns | Ergonomic developer experience |

## Communication Patterns

ros-z supports all essential ROS 2 communication patterns:

**Publishers & Subscribers:**

```rust,ignore
let pub = node.create_pub::<String>("topic").build()?;
let sub = node.create_sub::<String>("topic").build()?;
```

**Services:**

```rust,ignore
let server = node.create_service::<AddTwoInts>("add").build()?;
let client = node.create_client::<AddTwoInts>("add").build()?;
```

**Actions:**

```rust,ignore
// Long-running tasks with feedback and cancellation
let action_server = node.create_action_server::<Fibonacci>("fibonacci").build()?;
let action_client = node.create_action_client::<Fibonacci>("fibonacci").build()?;
```

```admonish tip
Start with pub-sub for data streaming, use services for request-response operations, and leverage actions for long-running tasks that need progress feedback.
```

## Why ros-z?

**Safety:** Rust's ownership model prevents common robotics bugs like data races, null pointer dereferences, and buffer overflows at compile time.

**Productivity:** Cargo ecosystem integration, excellent tooling, and expressive type system accelerate development without sacrificing reliability.

```admonish note
ros-z is designed for both new projects and gradual migration. Deploy ros-z nodes alongside existing ROS 2 C++/Python nodes with full interoperability.
```

## Resources

- **[Quick Start](./chapters/quick_start.md)** - Get running in 5 minutes
- **[Publishers & Subscribers](./chapters/pubsub.md)** - Master the fundamental pattern
- **[Services](./chapters/services.md)** - Request-response communication
- **[Examples](./chapters/examples_overview.md)** - Production-ready code samples
- **[Troubleshooting](./chapters/troubleshooting.md)** - Common issues and solutions

**Ready to build safer, faster robotics applications? Start with the [Quick Start Guide](./chapters/quick_start.md).**
