# Services

Services provide a request-response communication pattern in ROS 2. A service server processes requests and sends back responses, while service clients send requests and wait for responses.

## Overview

- **Service Server**: Waits for requests and sends responses
- **Service Client**: Sends requests and receives responses
- Services are strongly typed with request and response message types
- Synchronous request-response pattern (unlike pub/sub)

## Basic Pattern

### Server Side

1. Create a context and node
2. Create a service using `node.create_service::<ServiceType>("service_name")`
3. Wait for requests with `service.take_request()`
4. Process the request
5. Send response with `service.send_response()`

### Client Side

1. Create a context and node
2. Create a client using `node.create_client::<ServiceType>("service_name")`
3. Send a request with `client.send_request()`
4. Wait for response with `client.take_response()`

## Example: Combined Service and Client

Here's a simple example showing both server and client:

```rust,no_run
{{#include ../../../ros-z/examples/z_srvcli.rs}}
```

### Key Points

- **Line 33**: Creates a service server that waits for requests
- **Line 38**: `take_request()` blocks until a request arrives
- **Line 44**: Sends the response back to the client
- **Line 51**: Creates a service client
- **Line 58**: Sends a request to the server
- **Line 59**: Blocks waiting for the response

## Dedicated Examples

- [Service Server](./service_server.md) - AddTwoInts server implementation
- [Service Client](./service_client.md) - AddTwoInts client implementation

## Error Handling

Service operations can fail for various reasons:

- Server not available
- Network issues
- Serialization errors

Always check the `Result` returned by service operations:

```rust,no_run,ignore
match client.take_response() {
    Ok(response) => println!("Got response: {}", response.sum),
    Err(e) => eprintln!("Service call failed: {}", e),
}
```

## Running Server and Client

Services require the server to be running before the client makes a request:

```bash
# Terminal 1 - Start server
cargo run --example z_srvcli -- --mode server

# Terminal 2 - Send client request
cargo run --example z_srvcli -- --mode client --a 5 --b 3
```
