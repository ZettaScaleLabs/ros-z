# Simple Publisher Example

This example demonstrates creating a basic publisher in ros-z.

For a complete working example that includes both publisher and subscriber, see the [z_pubsub example](./quick_start.md#basic-pubsub-example).

## Key Components

A simple publisher requires:

1. **Context**: Created with `ZContextBuilder`
2. **Node**: Created from the context
3. **Publisher**: Created with `create_pub::<MessageType>(topic)`
4. **Publishing**: Use `publish()` or `async_publish()`

## Example Code

See the complete pub/sub example in the [Quick Start](./quick_start.md) guide, which includes both publisher and subscriber in one file.

## Next Steps

- [Demo Talker](./demo_talker.md) - A dedicated publisher example
- [Publishers and Subscribers](./pubsub.md) - Complete guide to pub/sub
