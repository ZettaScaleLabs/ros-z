# Go Quick Start

Get a Go publisher and subscriber running in five minutes.

## Prerequisites

- Go 1.23+
- An Eclipse Zenoh router — see [Networking](../user-guide/networking.md)

No Rust toolchain required when using the pre-built library.

## 1. Get the library

### Option A — Pre-built library (recommended)

Download the static library and C header for your platform from the [Releases page](https://github.com/ZettaScaleLabs/ros-z/releases):

| Platform | Jazzy / Kilted / Rolling | Humble |
|---|---|---|
| Linux x86_64 | `libros_z-jazzy-x86_64-unknown-linux-gnu.a` | `libros_z-humble-x86_64-unknown-linux-gnu.a` |
| Linux aarch64 | `libros_z-jazzy-aarch64-unknown-linux-gnu.a` | `libros_z-humble-aarch64-unknown-linux-gnu.a` |
| macOS aarch64 | `libros_z-jazzy-aarch64-apple-darwin.a` | `libros_z-humble-aarch64-apple-darwin.a` |

Each release also includes `ros_z_ffi.h` (the C header required for CGO).

```bash
# Clone the repo for the Go package source — no Rust build needed
git clone https://github.com/ZettaScaleLabs/ros-z.git
cd ros-z

# Download the pre-built library and header — replace <version> and pick your platform file
curl -Lo crates/ros-z-go/libros_z.a \
  https://github.com/ZettaScaleLabs/ros-z/releases/download/<version>/libros_z-jazzy-x86_64-unknown-linux-gnu.a
curl -Lo crates/ros-z-go/rosz/ros_z_ffi.h \
  https://github.com/ZettaScaleLabs/ros-z/releases/download/<version>/ros_z_ffi.h
```

### Option B — Build from source

Requires Rust 1.85+, `cbindgen`, and `just`:

```bash
git clone https://github.com/ZettaScaleLabs/ros-z.git
cd ros-z
just -f crates/ros-z-go/justfile quickstart
```

This generates message types, compiles `libros_z.a`, and verifies both are present.

## 2. Write a publisher

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

Create `hello_pub/go.mod` — the `replace` directive points Go to the local `rosz` package:

```text
module hello_pub

go 1.23

require github.com/ZettaScaleLabs/ros-z/crates/ros-z-go v0.0.0

replace github.com/ZettaScaleLabs/ros-z/crates/ros-z-go => /path/to/ros-z/crates/ros-z-go
```

Replace `/path/to/ros-z` with the absolute path where you cloned the repo.

## 3. Write a subscriber

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

    _, err = node.CreateSubscriber("chatter").
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

    select {} // block forever
}
```

Create `hello_sub/go.mod` with the same `replace` directive as above.

## 4. Run

You need a Zenoh router running first — it acts as the rendezvous point for all nodes.

**Start the router** (pick one):

```bash
# Option A: download zenohd from https://github.com/eclipse-zenoh/zenoh/releases
./zenohd

# Option B: install via cargo (one-time)
cargo install zenohd && zenohd
```

**Run the subscriber and publisher** — set `CGO_LDFLAGS` to point at the library:

```bash
ROSZ=/path/to/ros-z

# Terminal 2: subscriber
cd hello_sub
CGO_LDFLAGS="-L$ROSZ/crates/ros-z-go -lros_z -lm" \
CGO_CFLAGS="-I$ROSZ/crates/ros-z-go/rosz" \
go run main.go

# Terminal 3: publisher
cd hello_pub
CGO_LDFLAGS="-L$ROSZ/crates/ros-z-go -lros_z -lm" \
CGO_CFLAGS="-I$ROSZ/crates/ros-z-go/rosz" \
go run main.go
```

You should see the subscriber printing messages from the publisher.

!!! tip
    Set `ROSZ` in your shell profile to avoid repeating the path.

## What's next

- **[Go Bindings](./go.md)** — full API reference: typed helpers, graph introspection, QoS, error handling
- **[Message Generation](../user-guide/message-generation.md)** — generate types from a full ROS 2 install
- **[ROS 2 Interoperability](../user-guide/interop.md)** — connect a ros-z subscriber to a live ROS 2 talker
