# Demo Nodes

This directory contains basic examples referenced from [ROS 2 demo_nodes_cpp](https://github.com/ros2/demos/tree/rolling/demo_nodes_cpp).

## Examples

### Talker

A simple publisher node that publishes "Hello World" messages to the `/chatter` topic.

**Features:**

- Publishes `std_msgs/String` messages
- Uses async API for non-blocking publishing
- Publishes at 1 Hz
- Uses QoS history depth of 7 (KeepLast)

**Run:**

```bash
# Default settings (topic: chatter, period: 1.0s)
cargo run --example demo_nodes_talker

# Custom topic and faster publishing
cargo run --example demo_nodes_talker -- --topic /my_topic --period 0.5

# Connect to specific Zenoh router
cargo run --example demo_nodes_talker -- --endpoint tcp/localhost:7447

# See all options
cargo run --example demo_nodes_talker -- --help
```

**Available Options:**

- `-t, --topic <TOPIC>`: Topic name to publish to [default: chatter]
- `-p, --period <PERIOD>`: Publishing period in seconds [default: 1.0]
- `-m, --mode <MODE>`: Zenoh session mode (peer, client, router) [default: peer]
- `-e, --endpoint <ENDPOINT>`: Zenoh router endpoint to connect to

### Listener

A simple subscriber node that listens to messages on a topic.

**Features:**

- Subscribes to `std_msgs/String` messages
- Uses async API for receiving messages
- Uses QoS history depth of 10 (KeepLast)
- Prints received messages to console
- Configurable topic and Zenoh settings

**Run:**

```bash
# Default settings (topic: chatter)
cargo run --example demo_nodes_listener

# Custom topic
cargo run --example demo_nodes_listener -- --topic /my_topic

# Connect to specific Zenoh router
cargo run --example demo_nodes_listener -- --endpoint tcp/localhost:7447

# See all options
cargo run --example demo_nodes_listener -- --help
```

**Available Options:**

- `-t, --topic <TOPIC>`: Topic name to subscribe to [default: chatter]
- `-m, --mode <MODE>`: Zenoh session mode (peer, client, router) [default: peer]
- `-e, --endpoint <ENDPOINT>`: Zenoh router endpoint to connect to

## Testing

To test the examples, run both the talker and listener in separate terminals:

Terminal 1:

```bash
cargo run --example demo_nodes_talker
```

Terminal 2:

```bash
cargo run --example demo_nodes_listener
```

The listener should print messages like:

```bash
I heard: [Hello World: 1]
I heard: [Hello World: 2]
...
```
