# Custom Messages

ros-z supports using custom ROS 2 message types through generated Rust code.

## Example

The following example demonstrates using a custom message type:

```rust,no_run
{{#include ../../../ros-z/examples/z_custom_message.rs}}
```

## Message Generation

Custom messages are generated from ROS 2 `.msg` files using the ros-z code generation tools. The generated Rust structs implement the necessary traits for serialization and use with ros-z publishers and subscribers.

## Next Steps

- See the code generation documentation for creating custom messages
- Explore the `ros-z-msgs` crate for available standard ROS 2 messages
