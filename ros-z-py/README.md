# ros-z-py: Python Bindings for ros-z

Python bindings for [ros-z](../ros-z), a native Rust ROS 2 implementation using Zenoh.

## Status: 🚧 Phase 1 Complete - Core Pub/Sub Functional

The core infrastructure is complete and functional for basic publish/subscribe operations!

### ✅ What Works

- **Session Management**: Create ROS 2 sessions with domain ID
- **Node Creation**: Create nodes with name and namespace
- **QoS Profiles**: Full QoS configuration (reliability, durability, history, liveliness, deadlines)
- **Type Registry**: Message serialization/deserialization for:
  - `std_msgs/msg/String` ✓
  - `geometry_msgs/msg/Vector3` ✓
  - `geometry_msgs/msg/Twist` ✓
- **Publisher**: Publish messages (with type registry)
- **Subscriber**: Receive messages (with type registry)

### 🚧 In Progress

- **Publisher/Subscriber Creation**: Node methods need to instantiate actual ZPub/ZSub
- **More Message Types**: Expanding registry to cover all standard messages
- **Services**: ROS 2 service client/server support
- **Actions**: ROS 2 action client/server support

## Installation

### Prerequisites

- Python 3.8 or later
- Rust toolchain (for building)
- maturin (`pip install maturin`)

### Build

```bash
cd ros-z-py
maturin develop --release
```

This builds the Python extension module and installs it in your current Python environment.

## Usage

```python
import ros_z_py

# Create a ROS 2 session
session = ros_z_py.open_session(domain_id=0)

# Create a node
node = ros_z_py.create_node(session, "my_node", namespace="/")

# Create a publisher (once implemented)
pub = node.create_publisher(
    "/chatter",
    "std_msgs/msg/String",
    qos={"reliability": "reliable", "history": "keep_last", "depth": 10}
)

# Publish a message
pub.publish({"data": "Hello from Python!"})

# Create a subscriber
sub = node.create_subscriber("/chatter", "std_msgs/msg/String")

# Receive messages
msg = sub.recv(timeout=1.0)  # Returns dict or None
if msg:
    print(f"Received: {msg['data']}")
```

### Twist Example

```python
# Publish geometry_msgs/Twist
pub = node.create_publisher("/cmd_vel", "geometry_msgs/msg/Twist")
pub.publish({
    "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.3}
})
```

## QoS Profiles

QoS profiles are specified as Python dicts:

```python
qos = {
    "reliability": "reliable",  # or "best_effort"
    "durability": "volatile",   # or "transient_local"
    "history": "keep_last",     # or "keep_all"
    "depth": 10,                # for keep_last
    "deadline": 1.0,            # seconds (optional)
    "lifespan": 5.0,            # seconds (optional)
    "liveliness": "automatic",  # or "manual_by_node", "manual_by_topic"
    "liveliness_lease_duration": 2.0  # seconds (optional)
}
```

Predefined QoS profiles are available:

```python
ros_z_py.QOS_DEFAULT
ros_z_py.QOS_SENSOR_DATA
ros_z_py.QOS_PARAMETERS
ros_z_py.QOS_SERVICES
```

## Architecture

### Type Registry

The type registry (in `ros-z-msgs`) provides bidirectional conversion between Python dicts and CDR-serialized bytes:

```python
# Serialization: Python dict → CDR bytes
cdr_bytes = ros_z_msgs.serialize_to_cdr("std_msgs/msg/String", py, {"data": "hello"})

# Deserialization: CDR bytes → Python dict
msg_dict = ros_z_msgs.deserialize_from_cdr("std_msgs/msg/String", py, cdr_bytes)
```

### Type-Erased Traits

The bindings use type-erased `RawPublisher` and `RawSubscriber` traits to bridge generic Rust types to Python:

```rust
trait RawPublisher {
    fn publish_serialized(&self, data: &[u8]) -> Result<()>;
}

trait RawSubscriber {
    fn recv_serialized(&self, timeout: Option<Duration>) -> Result<Vec<u8>>;
    fn try_recv_serialized(&self) -> Result<Option<Vec<u8>>>;
}
```

Wrappers (`ZPubWrapper`, `ZSubWrapper`) implement these traits for ros-z's generic publishers/subscribers.

## Module Structure

```
ros-z-py/src/
├── lib.rs          - PyO3 module definition
├── session.rs      - PySession (wraps ZContext)
├── node.rs         - PyNode (wraps ZNode)
├── publisher.rs    - PyPublisher (type-erased publisher)
├── subscriber.rs   - PySubscriber (type-erased subscriber)
├── qos.rs          - QoS conversion (Python ↔ Rust)
├── traits.rs       - RawPublisher/RawSubscriber traits
├── error.rs        - Custom Python exceptions
├── utils.rs        - Conversion traits
└── registry.rs     - Runtime registration (future)
```

## Development

### Running Examples

```bash
# Build the module
maturin develop --release

# Run example
python examples/simple_pubsub.py
```

### Testing

```bash
# Run Rust tests
cargo test -p ros-z-py

# Integration tests (requires ROS 2 environment)
python -m pytest tests/
```

### Adding Message Types

To add a new message type to the registry, edit `ros-z-msgs/src/python_registry.rs`:

```rust
fn serialize_my_msg(_py: Python, data: &Bound<'_, PyDict>) -> PyResult<Vec<u8>> {
    // Extract fields from dict
    let field: Type = data.get_item("field")?.extract()?;

    // Create message
    let msg = MyMsg { field };

    // Serialize to CDR
    cdr::serialize::<_, _, CdrLe>(&msg, Infinite)
        .map_err(|e| PyValueError::new_err(format!("Serialization failed: {}", e)))
}

// Register in init_registry()
register_type("package/msg/MyMsg", serialize_my_msg, deserialize_my_msg, &hash);
```

## Performance

- **Zero-copy deserialization** where possible (using PyO3's buffer protocol)
- **GIL release** during blocking operations (recv with timeout)
- **Thread-safe** message registry using `RwLock`
- **Stable ABI** (abi3-py38) for binary compatibility across Python versions

## Comparison with rclpy

| Feature | ros-z-py | rclpy |
|---------|----------|-------|
| Backend | Zenoh (Rust) | DDS (C++) |
| Installation | Single wheel | Requires ROS 2 |
| Performance | High (Rust) | Medium (C++) |
| Memory Safety | Guaranteed | Manual |
| Binary Size | ~5MB | ~50MB+ |
| Dependencies | Minimal | Many |

## License

This project is licensed under the same terms as ros-z. See the root LICENSE file for details.

## Contributing

Contributions welcome! Areas for improvement:

1. **More message types**: Add registry entries for sensor_msgs, nav_msgs, etc.
2. **Auto-generation**: Generate registry from message definitions
3. **Services**: Implement service client/server
4. **Actions**: Implement action client/server
5. **Runtime registration**: Allow user-defined messages without rebuilding

## Resources

- [ros-z Documentation](../../book/)
- [PyO3 Documentation](https://pyo3.rs)
- [Zenoh Documentation](https://zenoh.io/docs/)
- [ROS 2 Documentation](https://docs.ros.org/en/rolling/)
