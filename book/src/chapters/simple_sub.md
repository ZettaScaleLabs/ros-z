# Simple Subscriber Example

This example demonstrates creating a basic subscriber in ros-z.

For a complete working example that includes both publisher and subscriber, see the [z_pubsub example](./quick_start.md#basic-pubsub-example).

## Key Components

A simple subscriber requires:

1. **Context**: Created with `ZContextBuilder`
2. **Node**: Created from the context
3. **Subscriber**: Created with `create_sub::<MessageType>(topic)`
4. **Receiving**: Use `recv()`, `recv_timeout()`, or `async_recv()`

## Example Code

See the complete pub/sub example in the [Quick Start](./quick_start.md) guide.

## Next Steps

- [Demo Listener](./demo_listener.md) - A dedicated subscriber example
- [Publishers and Subscribers](./pubsub.md) - Complete guide to pub/sub
