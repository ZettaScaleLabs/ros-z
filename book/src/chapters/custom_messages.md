# Custom Messages

This section demonstrates how to manually implement custom messages in ros-z by defining Rust structs and implementing the required traits.

## Complete Example

The `z_custom_message` example shows a full implementation with both custom messages and services:

```rust,ignore
{{#include ../../../ros-z/examples/z_custom_message.rs}}
```

## What This Example Demonstrates

This example includes:

1. **Custom Message Type** (`RobotStatus`)
   - Implements `MessageTypeInfo` for type identification
   - Implements `WithTypeInfo` for ros-z integration
   - Uses `serde` for serialization
   - Demonstrates pub/sub pattern

2. **Custom Service Types** (`NavigateTo`)
   - Request type (`NavigateToRequest`)
   - Response type (`NavigateToResponse`)
   - Service definition implementing `ServiceTypeInfo` and `ZService`
   - Demonstrates request/response pattern

3. **Four Operating Modes**
   - `status-pub`: Publishes robot status messages
   - `status-sub`: Subscribes to robot status messages
   - `nav-server`: Runs navigation service server
   - `nav-client`: Sends navigation requests

## Running the Example

### Publisher/Subscriber

Terminal 1 (subscriber):

```bash
cargo run --example z_custom_message -- --mode status-sub
```

Terminal 2 (publisher):

```bash
cargo run --example z_custom_message -- --mode status-pub
```

### Service Client/Server

Terminal 1 (server):

```bash
cargo run --example z_custom_message -- --mode nav-server
```

Terminal 2 (client):

```bash
cargo run --example z_custom_message -- --mode nav-client --target_x 10.0 --target_y 20.0 --max_speed 1.5
```

## Key Traits

### MessageTypeInfo

Required for all message types:

```rust,ignore
impl MessageTypeInfo for RobotStatus {
    fn type_name() -> &'static str {
        "custom_msgs::msg::dds_::RobotStatus_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero()  // For ros-z-to-ros-z communication
    }
}
```

### ServiceTypeInfo and ZService

Required for services:

```rust,ignore
impl ServiceTypeInfo for NavigateTo {
    fn service_type_info() -> TypeInfo {
        TypeInfo::new("custom_msgs::srv::dds_::NavigateTo_", TypeHash::zero())
    }
}

impl ZService for NavigateTo {
    type Request = NavigateToRequest;
    type Response = NavigateToResponse;
}
```

## When to Use Manual Messages

**Use manual custom messages when:**

- Prototyping new message types
- Building standalone ros-z applications
- You don't need interoperability with other ROS 2 nodes
- Testing and experimentation

**Use generated messages when:**

- Interoperating with existing ROS 2 systems
- Using standard ROS message types
- You need proper type hashing for ROS 2 compatibility
- Working with existing `.msg` and `.srv` definitions

## Next Steps

- Review the parent [Message Generation](./message_generation.md) chapter for the full architecture
- See [Publishers and Subscribers](./pubsub.md) for using messages with ros-z
- Check [Services](./services.md) for service patterns
- Explore [Feature Flags](./feature_flags.md) for available pre-generated message packages
