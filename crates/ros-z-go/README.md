# ros-z-go: Go Bindings for ros-z

Go bindings for [ros-z](https://github.com/ZettaScaleLabs/ros-z), enabling Go applications to communicate with ROS 2 nodes via Zenoh.

## Features

- **Pub/Sub**: Publishers and subscribers with QoS presets
- **Memory Safe**: Uses `cgo.Handle` and `runtime.Pinner` for safe CGO interop
- **Idiomatic Go**: Channel-based message delivery with `Handler[T]` interface
- **Structured Errors**: Type-safe error handling with `RoszError`
- **Zero ROS 2 Dependencies**: Pure Rust + Go stack (ROS 2 optional for interop)

## Quick Start

### Installation

```bash
# Build Rust FFI library
just build-rust

# Generate bundled message types (no ROS 2 needed)
just codegen-bundled
```

### Publisher Example

```go
package main

import (
    "fmt"
    "time"
    "github.com/ZettaScaleLabs/ros-z/crates/ros-z-go/rosz"
    "github.com/ZettaScaleLabs/ros-z/crates/ros-z-go/generated/std_msgs"
)

func main() {
    ctx, _ := rosz.NewContext().WithDomainID(0).Build()
    defer ctx.Close()

    node, _ := ctx.CreateNode("talker").Build()
    defer node.Close()

    pub, _ := node.CreatePublisher("chatter").Build(&std_msgs.String{})
    defer pub.Close()

    for i := 0; ; i++ {
        msg := &std_msgs.String{Data: fmt.Sprintf("Hello #%d", i)}
        if err := pub.Publish(msg); err != nil {
            fmt.Printf("Publish failed: %v\n", err)
        }
        time.Sleep(100 * time.Millisecond)
    }
}
```

### Subscriber with Callback

```go
node.CreateSubscriber("chatter").
    BuildWithCallback(&std_msgs.String{}, func(data []byte) {
        var msg std_msgs.String
        msg.DeserializeCDR(data)
        fmt.Printf("Received: %s\n", msg.Data)
    })
```

### Subscriber with Channel

```go
handler := rosz.NewFifoChannel[[]byte](10)
callback, drop, ch := handler.ToCbDropHandler()
sub, _ := node.CreateSubscriber("chatter").
    BuildWithCallback(&std_msgs.String{}, callback)
defer drop()

for data := range ch {
    var msg std_msgs.String
    msg.DeserializeCDR(data)
    fmt.Printf("Received: %s\n", msg.Data)
}
```

## API Overview

### Context & Node

```go
ctx, err := rosz.NewContext().WithDomainID(0).Build()
defer ctx.Close()

node, err := ctx.CreateNode("my_node").Build()
defer node.Close()
```

### Publisher & Subscriber

```go
// Publisher
pub, err := node.CreatePublisher("topic").Build(&MyMsg{})
pub.Publish(&MyMsg{Data: "hello"})

// Subscriber (callback)
sub, err := node.CreateSubscriber("topic").
    BuildWithCallback(&MyMsg{}, func(data []byte) {
        // Process message
    })

// Typed subscriber
sub, err := rosz.BuildWithTypedCallback(
    node.CreateSubscriber("topic"),
    func(msg *MyMsg) { fmt.Println(msg.Data) },
)
```

## Handler Interface

ros-z-go supports three message delivery patterns:

### 1. Closure (Direct Callback)

- **Pros**: Zero allocation, lowest latency
- **Cons**: Blocks Zenoh thread, no concurrency
- **Use**: Simple processing, < 1ms per message

### 2. FifoChannel (Buffered, Blocking)

```go
handler := rosz.NewFifoChannel[[]byte](10)
```

- **Pros**: Backpressure control, enables batching
- **Cons**: Blocks sender when full
- **Use**: Reliable delivery, I/O-bound work

### 3. RingChannel (Non-blocking, Drops Oldest)

```go
handler := rosz.NewRingChannel[[]byte](5)
```

- **Pros**: Never blocks, always fresh data
- **Cons**: Can drop messages
- **Use**: Real-time systems, position updates

## Testing

```bash
# Pure Go tests (no FFI)
just test-go-pure

# FFI unit tests (requires build-rust)
just test-go-ffi

# All Go tests
just test-go
```

## Architecture

```text
Go Application
      ↓
rosz Package (Go API)
      ↓
CGO (cgo.Handle, runtime.Pinner)
      ↓
C FFI Layer (callback_bridge.c)
      ↓
Rust ros-z FFI (ffi/)
      ↓
Rust ros-z Core
      ↓
Zenoh
```

Memory safety: `cgo.Handle` (Go 1.17+) for type-safe callback storage, `runtime.Pinner` (Go 1.21+) to prevent GC relocation during CGO calls, automatic cleanup on subscriber destruction.

Serialization: all messages use CDR (Common Data Representation) for ROS 2 compatibility.

## Examples

Full examples in [`examples/`](./examples/):

- `publisher/` — talker publishing `std_msgs/String` at 10 Hz
- `subscriber/` — listener with typed callback
- `subscriber_channel/` — channel-based delivery patterns

```bash
just demo   # publisher + subscriber in parallel
```

## Requirements

- **Go**: 1.23+
- **Rust**: stable toolchain (`rustup`)
- **cbindgen**: `cargo install cbindgen`
- **just**: `cargo install just`

## References

- **ros-z Documentation**: <https://zettascalelabs.github.io/ros-z/>
- **Zenoh**: <https://zenoh.io/>
- **ROS 2**: <https://docs.ros.org/>

## Acknowledgement

This work is sponsored by

<p align="left">
  <a href="https://www.dexory.com/">
    <img height="50" src="https://raw.githubusercontent.com/ZettaScaleLabs/rmw-zenoh-mcap-writer/2a324eb8a2f0cddc01fe52f504f78b4c44b36d90/images/Dexory_logo.png" alt="Dexory"/>
  </a>
</p>
