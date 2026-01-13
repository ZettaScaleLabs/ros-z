# Introduction

**ros-z is a native Rust ROS 2 implementation powered by Zenoh, delivering high-performance robotics communication with type safety and zero-cost abstractions.** Build reliable robot applications using modern Rust idioms while maintaining full ROS 2 compatibility.

## Architecture

ros-z provides three integration paths to suit different use cases:

<div style="position: relative; width: 100%; height: 700px; margin: 20px 0; overflow: hidden;">
    <iframe src="architecture.html" style="width: 1600px; height: 1000px; border: none; transform: scale(0.75); transform-origin: top left;" title="Interactive ros-z Architecture" scrolling="no"></iframe>
</div>


## Why Choose ros-z?

| Feature | Description | Benefit |
|---------|-------------|---------|
| **Native Rust** | Pure Rust implementation with no C/C++ dependencies | Memory safety, concurrency without data races |
| **Zenoh Transport** | High-performance pub-sub engine | Low latency, efficient bandwidth usage |
| **ROS 2 Compatible** | Works seamlessly with standard ROS 2 tools | Integrate with existing robotics ecosystems |
| **Multiple Serializations** | Support for various data representations: CDR (ROS default), Protobuf | Flexible message encoding for different performance and interoperability needs |
| **Type Safety** | Compile-time message validation | Catch errors before deployment |
| **Modern API** | Idiomatic Rust patterns | Ergonomic developer experience |
| **Safety First** | Ownership model prevents common bugs | No data races, null pointers, or buffer overflows at compile time |
| **High Productivity** | Cargo ecosystem with excellent tooling | Fast development without sacrificing reliability |

```admonish note
ros-z is designed for both new projects and gradual migration. Deploy ros-z nodes alongside existing ROS 2 C++/Python nodes with full interoperability.
```

## Communication Patterns

ros-z supports all essential ROS 2 communication patterns:

| Pattern | Use Case | Learn More |
|---------|----------|------------|
| **Pub/Sub** | Continuous data streaming, sensor data, status updates | [Pub/Sub](./chapters/pubsub.md) |
| **Services** | Request-response operations, remote procedure calls | [Services](./chapters/services.md) |
| **Actions** | Long-running tasks with feedback and cancellation support | [Actions](./chapters/actions.md) |

```admonish tip
Start with pub/sub for data streaming, use services for request-response operations, and leverage actions for long-running tasks that need progress feedback.
```

## Ergonomic API Design

ros-z provides flexible, idiomatic Rust APIs that adapt to your preferred programming style:

**Flexible Builder Pattern:**

```rust,ignore
let pub = node.create_pub::<Vector3>("vector")
    // Quality of Service settings
    .with_qos(QosProfile {
        reliability: QosReliability::Reliable,
        ..Default::default()
    })
    // custom serialization
    .with_serdes::<ProtobufSerdes<Vector3>>()
    .build()?;
```

**Async & Sync Patterns:**

```rust,ignore
// Publishers: sync and async variants
zpub.publish(&msg)?;
zpub.async_publish(&msg).await?;

// Subscribers: sync and async receiving
let msg = zsub.recv()?;
let msg = zsub.async_recv().await?;
```

**Callback or Polling Style for Subscribers:**

```rust,ignore
// Callback style - process messages with a closure
let sub = node.create_sub::<RosString>("topic")
    .build_with_callback(|msg| {
        println!("Received: {}", msg);
    })?;

// Polling style - receive messages on demand
let sub = node.create_sub::<RosString>("topic").build()?;
while let Ok(msg) = sub.recv() {
    println!("Received: {}", msg);
}
```


## Next Step

**Ready to build safer, faster robotics applications? Start with the [Quick Start Guide](./chapters/quick_start.md).**
