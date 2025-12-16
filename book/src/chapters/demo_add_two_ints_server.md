# Add Two Ints Server

The AddTwoInts server demo provides a service that adds two integers and returns the sum.

## Complete Example

```rust,no_run
{{#include ../../../ros-z/examples/demo_nodes/add_two_ints_server.rs}}
```

## Key Points

- **Line 11**: Creates a node named "add_two_ints_server"
- **Line 14**: Creates a service of type `AddTwoInts` with the service name "add_two_ints"
- **Line 22**: `take_request()` blocks until a request arrives, returning both the key and request data
- **Line 26**: Computes the sum of the two integers
- **Line 29**: Creates the response with the computed sum
- **Line 34**: Sends the response back using the request key
- **Line 39-43**: Optional max_requests feature allows the server to exit after handling N requests

## Service Type

The `AddTwoInts` service uses messages from `example_interfaces`:

```rust,ignore
use ros_z_msgs::example_interfaces::{
    AddTwoInts,
    AddTwoIntsRequest,   // Contains: a: i64, b: i64
    AddTwoIntsResponse,  // Contains: sum: i64
};
```

## Usage

Run the server:

```bash
cargo run --example demo_nodes_add_two_ints_server
```

With options:

```bash
# Handle 5 requests then exit
cargo run --example demo_nodes_add_two_ints_server -- --count 5

# Connect to specific Zenoh router
cargo run --example demo_nodes_add_two_ints_server -- --endpoint tcp/localhost:7447
```

## Testing with Client

Pair this with the [client demo](./demo_add_two_ints_client.md):

```bash
# Terminal 1 - Start server
cargo run --example demo_nodes_add_two_ints_server

# Terminal 2 - Send request
cargo run --example demo_nodes_add_two_ints_client -- --a 10 --b 20
```

Expected output:

```
# Server output:
Incoming request
a: 10 b: 20
Sending response: 30

# Client output:
Sending request: a=10, b=20
Received response: sum=30
```
