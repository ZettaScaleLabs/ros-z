# Service Server

For a complete service server example, see:

- [Add Two Ints Server](./demo_add_two_ints_server.md) - Full server implementation
- [Services Overview](./services.md) - Complete guide to services

## Basic Pattern

```rust,no_run,ignore
let node = ctx.create_node("my_server").build()?;
let mut service = node.create_service::<ServiceType>("service_name").build()?;

loop {
    let (key, request) = service.take_request()?;
    // Process request...
    let response = create_response(&request);
    service.send_response(&response, &key)?;
}
```
