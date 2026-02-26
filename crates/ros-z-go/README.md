# ros-z-go: Go Bindings for ros-z

Go bindings for [ros-z](https://github.com/ZettaScaleLabs/ros-z), enabling Go applications to communicate with ROS 2 nodes via Zenoh.

## Features

- **Full ROS 2 API**: Publishers, Subscribers, Services, Actions
- **Memory Safe**: Uses `cgo.Handle` and `runtime.Pinner` for safe CGO interop
- **Idiomatic Go**: Channel-based message delivery with `Handler[T]` interface
- **Structured Errors**: Type-safe error handling with `RoszError`
- **Zero ROS 2 Dependencies**: Pure Rust + Go stack (ROS 2 optional for interop)

## Quick Start

### Installation

```bash
# Build Rust FFI library
just build-rust

# Build Go bindings
cd crates/ros-z-go && go build ./...
```

### Publisher Example

```go
package main

import (
    "fmt"
    "time"
    "github.com/ZettaScaleLabs/ros-z-go/rosz"
    "github.com/ZettaScaleLabs/ros-z-go/testdata"
)

func main() {
    ctx, _ := rosz.NewContext().WithDomainID(0).Build()
    defer ctx.Close()

    node, _ := ctx.CreateNode("talker").Build()
    defer node.Close()

    pub, _ := node.CreatePublisher("chatter").Build(&testdata.String{})
    defer pub.Close()

    for i := 0; ; i++ {
        msg := &testdata.String{Data: fmt.Sprintf("Hello #%d", i)}
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
    BuildWithCallback(&testdata.String{}, func(data []byte) {
        var msg testdata.String
        msg.DeserializeCDR(data)
        fmt.Printf("Received: %s\n", msg.Data)
    })
```

### Subscriber with Channel

```go
handler := rosz.NewFifoChannel[[]byte](10)
sub, _ := node.CreateSubscriber("chatter").
    BuildWithHandler(&testdata.String{}, handler)

_, _, ch := handler.ToCbDropHandler()
for data := range ch {
    var msg testdata.String
    msg.DeserializeCDR(data)
    fmt.Printf("Received: %s\n", msg.Data)
}
```

## API Overview

### Context & Node

```go
// Create context with domain ID
ctx, err := rosz.NewContext().WithDomainID(42).Build()
defer ctx.Close()

// Create node
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

// Subscriber (channel)
handler := rosz.NewRingChannel[[]byte](5)
sub, err := node.CreateSubscriber("topic").
    BuildWithHandler(&MyMsg{}, handler)
```

### Service Client & Server

```go
// Service client
client, err := node.CreateServiceClient("add_two_ints").Build(svc)
response, err := client.Call(request)

// Service server
server, err := node.CreateServiceServer("add_two_ints").
    Build(svc, func(reqData []byte) ([]byte, error) {
        var req MyRequest
        req.DeserializeCDR(reqData)
        // Process request
        resp := &MyResponse{Sum: req.A + req.B}
        return resp.SerializeCDR()
    })
```

### Action Client & Server

```go
// Action client
client, err := node.CreateActionClient("fibonacci").Build(action)
goalHandle, err := client.SendGoal(goal)
result, err := goalHandle.GetResult()

// Action server
server, err := node.CreateActionServer("fibonacci").Build(
    action,
    func(goalData []byte) bool { /* accept/reject */ return true },
    func(handle *rosz.ServerGoalHandle, goalData []byte) ([]byte, error) {
        // Execute goal, publish feedback via handle.PublishFeedback()
        return resultBytes, nil
    },
)
```

## Error Handling

### Structured Errors

```go
resp, err := client.Call(req)
if err != nil {
    if roszErr, ok := err.(rosz.RoszError); ok {
        switch roszErr.Code() {
        case rosz.ErrorCodeServiceTimeout:
            // Retry logic
        case rosz.ErrorCodeActionGoalRejected:
            // Handle rejection
        default:
            // General error
        }
    }
}
```

### Error Codes

- `ErrorCodeSuccess` (0)
- `ErrorCodeServiceTimeout` (-10)
- `ErrorCodeServiceCallFailed` (-9)
- `ErrorCodeActionGoalRejected` (-11)
- `ErrorCodePublishFailed` (-4)
- See `rosz/error.go` for complete list

### Convenience Methods

```go
if err.(rosz.RoszError).IsTimeout() { /* ... */ }
if err.(rosz.RoszError).IsRejected() { /* ... */ }
```

## Handler Interface

ros-z-go supports three message delivery patterns:

### 1. Closure (Direct Callback)

```go
handler := rosz.NewClosure(
    func(data []byte) { processMessage(data) },
    func() { cleanup() },
)
```

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
just test-go-pure  # 30 tests

# FFI unit tests
just test-go-ffi   # 26 tests

# All Go tests
just test-go       # 56 tests

# Integration tests (requires zenohd)
just test-integration  # 11 tests
```

## Architecture

### Layers

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

### Memory Safety

- **cgo.Handle (Go 1.17+)**: Type-safe callback storage with GC integration
- **runtime.Pinner (Go 1.21+)**: Prevents GC relocation during CGO calls
- Automatic cleanup on subscriber/server destruction

### Serialization

All messages use CDR (Common Data Representation) format for ROS 2 compatibility:

```go
type Message interface {
    TypeName() string
    TypeHash() string
    SerializeCDR() ([]byte, error)
    DeserializeCDR([]byte) error
}
```

## Examples

Full examples available in [`examples/`](./examples/):

- `publisher/` - Publisher example
- `subscriber/` - Subscriber with callback
- `service_client/` - Service client
- `service_server/` - Service server
- `action_client/` - Action client
- `action_server/` - Action server

Run examples:

```bash
just build-rust
cd crates/ros-z-go/examples/publisher && go run .
```

## Requirements

- **Go**: 1.23+ (for `runtime.Pinner`, generics)
- **Rust**: 1.75+ (for FFI layer)
- **OS**: Linux (tested), macOS (should work), Windows (untested)

## Contributing

See the main [ros-z repository](https://github.com/ZettaScaleLabs/ros-z) for contribution guidelines.

## License

Same as ros-z (check main repository).

## References

- **ros-z Documentation**: <https://zettascalelabs.github.io/ros-z/>
- **Zenoh**: <https://zenoh.io/>
- **ROS 2**: <https://docs.ros.org/>
