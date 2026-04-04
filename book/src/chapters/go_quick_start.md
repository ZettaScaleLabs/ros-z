# Go Quick Start

Get a Go publisher and subscriber running in five minutes.

## Prerequisites

- Go 1.23+
- Rust toolchain (`rustup`)
- `cbindgen` — `cargo install cbindgen`
- `just` — `cargo install just`
- `zenohd` — `cargo install zenohd` (see [Networking](./networking.md) for details)

## 1. Generate message types

Message types are not checked in. Generate the common types from bundled IDL — no ROS 2 install needed:

```bash
just -f crates/ros-z-go/justfile codegen-bundled
```

## 2. Build the Rust FFI library

```bash
just -f crates/ros-z-go/justfile build-rust
```

This compiles `libros_z.a` into `target/release/` and auto-generates the C header via cbindgen.

Verify both are present:

```bash
just -f crates/ros-z-go/justfile verify
```

## 3. Write a publisher

Create `hello_pub/main.go`:

```go
package main

import (
    "fmt"
    "log"
    "time"

    "github.com/ZettaScaleLabs/ros-z/crates/ros-z-go/rosz"
    "github.com/ZettaScaleLabs/ros-z/crates/ros-z-go/generated/std_msgs"
)

func main() {
    ctx, err := rosz.NewContext().WithDomainID(0).Build()
    if err != nil {
        log.Fatal(err)
    }
    defer ctx.Close()

    node, err := ctx.CreateNode("go_talker").Build()
    if err != nil {
        log.Fatal(err)
    }
    defer node.Close()

    pub, err := node.CreatePublisher("chatter").Build(&std_msgs.String{})
    if err != nil {
        log.Fatal(err)
    }
    defer pub.Close()

    for i := 0; ; i++ {
        msg := &std_msgs.String{Data: fmt.Sprintf("Hello #%d", i)}
        if err := pub.Publish(msg); err != nil {
            log.Printf("publish error: %v", err)
        }
        fmt.Printf("Published: %s\n", msg.Data)
        time.Sleep(500 * time.Millisecond)
    }
}
```

Create `hello_pub/go.mod`:

```text
module hello_pub

go 1.23

require github.com/ZettaScaleLabs/ros-z/crates/ros-z-go v0.0.0

replace github.com/ZettaScaleLabs/ros-z/crates/ros-z-go => /path/to/ros-z/crates/ros-z-go
```

## 4. Write a subscriber

Create `hello_sub/main.go`:

```go
package main

import (
    "log"

    "github.com/ZettaScaleLabs/ros-z/crates/ros-z-go/rosz"
    "github.com/ZettaScaleLabs/ros-z/crates/ros-z-go/generated/std_msgs"
)

func main() {
    ctx, err := rosz.NewContext().WithDomainID(0).Build()
    if err != nil {
        log.Fatal(err)
    }
    defer ctx.Close()

    node, err := ctx.CreateNode("go_listener").Build()
    if err != nil {
        log.Fatal(err)
    }
    defer node.Close()

    sub, err := node.CreateSubscriber("chatter").
        BuildWithCallback(&std_msgs.String{}, func(data []byte) {
            msg := &std_msgs.String{}
            if err := msg.DeserializeCDR(data); err != nil {
                log.Printf("deserialize error: %v", err)
                return
            }
            log.Printf("Received: %s", msg.Data)
        })
    if err != nil {
        log.Fatal(err)
    }
    defer sub.Close()

    select {} // block forever
}
```

## 5. Run

You need a Zenoh router running first — publishers and subscribers only discover each other through a router:

```bash
# Terminal 1: router
cargo run --example zenoh_router

# Terminal 2: subscriber
cd hello_sub
CGO_LDFLAGS="-L/path/to/ros-z/target/release" go run main.go

# Terminal 3: publisher
cd hello_pub
CGO_LDFLAGS="-L/path/to/ros-z/target/release" go run main.go
```

You should see the subscriber printing messages published by the publisher.

```admonish tip
The bundled examples already have the right `go.mod` and `CGO_LDFLAGS` wiring.
Run the demo shortcut to see them in action:

    just -f crates/ros-z-go/justfile demo
```

## 6. Try the built-in examples

The repo ships ready-to-run examples for all patterns:

```bash
# Publisher + subscriber in parallel (Ctrl+C to stop)
just -f crates/ros-z-go/justfile demo

# Pub/sub
just -f crates/ros-z-go/justfile run-example publisher
just -f crates/ros-z-go/justfile run-example subscriber
just -f crates/ros-z-go/justfile run-example subscriber_channel   # channel-based / range loop

# Services
just -f crates/ros-z-go/justfile run-example service_server
just -f crates/ros-z-go/justfile run-example service_client
just -f crates/ros-z-go/justfile run-example service_client_errors  # timeouts, retries, sentinels

# Actions
just -f crates/ros-z-go/justfile run-example action_server
just -f crates/ros-z-go/justfile run-example action_client
just -f crates/ros-z-go/justfile run-example action_client_errors   # rejected goals, cancellation
```

For production-grade patterns (graceful shutdown, circuit breaker, metrics) see
`crates/ros-z-go/examples/production_service/README.md`.

## What next

- **[Go Bindings](./go_bindings.md)** — full API reference: services, actions, typed helpers, graph introspection, QoS, error handling
- **[Interop Tests](./go_interop_tests.md)** — how the test suite works and how to run it
- **[Message Generation](./message_generation.md)** — generate types from a full ROS 2 install
