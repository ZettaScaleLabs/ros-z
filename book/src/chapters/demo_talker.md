# Demo Talker

The talker demo is a simple publisher node that sends "Hello World" messages to a topic.

## Complete Example

```rust,no_run
{{#include ../../../ros-z/examples/demo_nodes/talker.rs}}
```

## Key Points

- **Line 24**: Creates a node named "talker" using the builder pattern
- **Line 28-31**: Configures a custom QoS profile with `KeepLast(7)` history
- **Line 32**: Creates a publisher for `RosString` messages on the "chatter" topic
- **Line 36-59**: Main loop that publishes messages at regular intervals
- **Line 46**: Uses `async_publish()` to send messages asynchronously
- **Line 49-53**: Optional max count feature allows limiting the number of messages

## Usage

Run the talker:

```bash
cargo run --example demo_nodes_talker
```

With custom options:

```bash
# Custom topic and publish rate
cargo run --example demo_nodes_talker -- --topic /my_topic --period 0.5

# Connect to a specific Zenoh router
cargo run --example demo_nodes_talker -- --endpoint tcp/localhost:7447
```

## Testing

Pair this with the [listener demo](./demo_listener.md) to see the complete pub/sub workflow:

```bash
# Terminal 1
cargo run --example demo_nodes_listener

# Terminal 2
cargo run --example demo_nodes_talker
```

You should see the talker publishing messages and the listener receiving them.
