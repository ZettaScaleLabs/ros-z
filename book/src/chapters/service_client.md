# Service Client

For a complete service client example, see:

- [Add Two Ints Client](./demo_add_two_ints_client.md) - Full client implementation
- [Services Overview](./services.md) - Complete guide to services

## Basic Pattern

```rust,no_run,ignore
let node = ctx.create_node("my_client").build()?;
let client = node.create_client::<ServiceType>("service_name").build()?;

let request = create_request();
client.send_request(&request)?;
let response = client.take_response()?;
```
