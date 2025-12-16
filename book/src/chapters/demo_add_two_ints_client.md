# Add Two Ints Client

The AddTwoInts client demo sends requests to the addition service and receives responses.

## Complete Example

```rust,ignore
{{#include ../../../ros-z/examples/demo_nodes/add_two_ints_client.rs}}
```

## Usage

```bash
# Send a request to add 10 and 20
cargo run --example demo_nodes_add_two_ints_client -- --a 10 --b 20
```

Make sure the [server](./demo_add_two_ints_server.md) is running first!
