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

### Add Two Ints Server

A simple service server that adds two integers and returns the result.

**Features:**

- Provides `example_interfaces/AddTwoInts` service
- Handles addition requests synchronously
- Uses async API for service handling
- Prints received requests and sent responses

**Run:**

```bash
# Default settings (handles unlimited requests)
cargo run --example demo_nodes_add_two_ints_server

# Handle only one request and exit
cargo run --example demo_nodes_add_two_ints_server -- --count 1

# Connect to specific Zenoh router
cargo run --example demo_nodes_add_two_ints_server -- --endpoint tcp/localhost:7447

# See all options
cargo run --example demo_nodes_add_two_ints_server -- --help
```

**Available Options:**

- `-c, --count <COUNT>`: Number of requests to handle before exiting (0 for unlimited) [default: 0]
- `-m, --mode <MODE>`: Zenoh session mode (peer, client, router) [default: peer]
- `-e, --endpoint <ENDPOINT>`: Zenoh router endpoint to connect to

### Add Two Ints Client

A simple service client that sends addition requests to the server.

**Features:**

- Calls `example_interfaces/AddTwoInts` service
- Supports both synchronous and asynchronous response waiting
- Uses async API for client operations
- Configurable numbers to add
- Prints sent requests and received responses

**Run:**

```bash
# Default settings (sync mode, adds 2 + 3)
cargo run --example demo_nodes_add_two_ints_client

# Async mode
cargo run --example demo_nodes_add_two_ints_client -- --async-mode

# Custom numbers
cargo run --example demo_nodes_add_two_ints_client -- --a 10 --b 20

# Async mode with custom numbers
cargo run --example demo_nodes_add_two_ints_client -- --a 10 --b 20 --async-mode

# Connect to specific Zenoh router
cargo run --example demo_nodes_add_two_ints_client -- --endpoint tcp/localhost:7447

# See all options
cargo run --example demo_nodes_add_two_ints_client -- --help
```

**Available Options:**

- `-a, --a <A>`: First number to add [default: 2]
- `-b, --b <B>`: Second number to add [default: 3]
- `--async-mode`: Use asynchronous response waiting
- `-m, --mode <MODE>`: Zenoh session mode (peer, client, router) [default: peer]
- `-e, --endpoint <ENDPOINT>`: Zenoh router endpoint to connect to

## Testing

### Pub/Sub Examples

To test the pub/sub examples, run both the talker and listener in separate terminals:

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

### Service Examples

To test the service examples, run the server and client in separate terminals:

Terminal 1 (Server):

```bash
cargo run --example demo_nodes_add_two_ints_server -- --count 1
```

Terminal 2 (Client - sync mode):

```bash
cargo run --example demo_nodes_add_two_ints_client
```

Or Terminal 2 (Client - async mode):

```bash
cargo run --example demo_nodes_add_two_ints_client -- --async-mode
```

The client should print:

```bash
AddTwoInts service client started (mode: sync/async)
Sending request: 2 + 3
Received response: 5
Result: 5
```

The server should print:

```bash
AddTwoInts service server started
Received request: 2 + 3
Sending response: 5
```
