# ros-z-py

Python bindings for [ros-z](https://github.com/ZettaScaleLabs/ros-z), a native Rust ROS 2 implementation using Zenoh.

## Overview

`ros-z-py` provides Python bindings to the ros-z library, allowing you to:

- Create ROS 2 nodes, publishers, and subscribers in Python
- Use auto-generated, type-safe message definitions with [msgspec](https://jcristharif.com/msgspec/)
- Communicate over Zenoh with zero-copy serialization
- Build ROS 2 applications without needing a full ROS 2 installation

## Architecture

ros-z-py consists of two packages:

1. **`ros-z-py`** - The main Python bindings (PyO3-based Rust extension)
   - Session and node management
   - Publisher/subscriber creation
   - QoS configuration
   - Message serialization/deserialization

2. **`ros-z-msgs-py`** - Auto-generated ROS 2 message type definitions
   - Pure Python msgspec.Struct definitions
   - Generated from ROS 2 `.msg` files at build time
   - Includes: `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, etc.

## Installation

```bash
cd ros-z-py

# Create and activate a virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Build the Rust extension
maturin develop --release

# Install both packages in editable mode
pip install -e ../ros-z-msgs/python/    # Message type definitions
pip install -e .                        # Python bindings
```

## Usage

### Basic Publisher/Subscriber

```python
import ros_z_py
from ros_z_py import std_msgs
import time

# Create a session and node
session = ros_z_py.open_session(domain_id=0)
node = ros_z_py.create_node(session, "example_node", "/")

# Create publisher and subscriber
pub = node.create_publisher("/chatter", "std_msgs/msg/String")
sub = node.create_subscriber("/chatter", "std_msgs/msg/String")

# Publish a message (using msgspec.Struct)
msg = std_msgs.String(data="Hello, ros-z!")
pub.publish(msg)

# Receive a message
received = sub.recv(timeout=1.0)
if received:
    print(f"Received: {received.data}")
```

### Complex Messages with Nested Types

```python
from ros_z_py import sensor_msgs, std_msgs, builtin_interfaces

# Create a LaserScan message with nested Header
scan = sensor_msgs.LaserScan(
    header=std_msgs.Header(
        stamp=builtin_interfaces.Time(sec=0, nanosec=0),
        frame_id="laser"
    ),
    angle_min=-1.57,
    angle_max=1.57,
    angle_increment=0.01,
    time_increment=0.0,
    scan_time=0.1,
    range_min=0.1,
    range_max=10.0,
    ranges=[1.0, 2.0, 3.0],
    intensities=[100.0, 150.0, 200.0]
)

pub = node.create_publisher("/scan", "sensor_msgs/msg/LaserScan")
pub.publish(scan)
```

### QoS Configuration

```python
# Use predefined QoS profiles
pub = node.create_publisher(
    "/sensor_data",
    "sensor_msgs/msg/Image",
    qos=ros_z_py.QOS_SENSOR_DATA
)

# Or customize QoS
custom_qos = {
    "reliability": "reliable",
    "durability": "transient_local",
    "history": "keep_last",
    "depth": 10
}
pub = node.create_publisher("/topic", "std_msgs/msg/String", qos=custom_qos)
```

## Message Type System

ros-z-py uses [msgspec](https://jcristharif.com/msgspec/) for message definitions, providing:

- **Type Safety**: Messages are statically typed Python dataclasses
- **Performance**: Fast serialization/deserialization
- **Validation**: Automatic type checking at runtime

### Message Structure

All ROS 2 messages are `msgspec.Struct` subclasses:

```python
from ros_z_py import geometry_msgs

# Create a message
vec = geometry_msgs.Vector3(x=1.0, y=2.0, z=3.0)

# Access fields with type safety
print(vec.x)  # IDE knows this is a float

# Messages are immutable by default (frozen=True)
# vec.x = 5.0  # Raises FrozenInstanceError

# Nested messages work seamlessly
twist = geometry_msgs.Twist(
    linear=geometry_msgs.Vector3(x=1.0, y=0.0, z=0.0),
    angular=geometry_msgs.Vector3(x=0.0, y=0.0, z=0.5)
)
```

## Examples

See the `examples/` directory:

- `talker.py` and `listener.py` - Classical talker/listener example
- `laser_scan.py` - Publish and subscribe to LaserScan messages

Run an example: ([A zenoh router is required](https://zettascalelabs.github.io/ros-z/pr-preview/pr-64/chapters/config.html#running-the-zenoh-router))

```bash
source .venv/bin/activate

# Run as publisher
python examples/laser_scan.py --mode pub

# Run as subscriber (in another terminal)
python examples/laser_scan.py --mode sub
```

## Testing

Run the test suite:

```bash
source .venv/bin/activate

# Run all tests
python tests/test_basic_pubsub.py
python tests/test_complex_messages.py

# Or use pytest
pytest tests/ -v
```

## Development Workflow

### Building Rust Changes

After modifying Rust code:

```bash
cd ros-z-py

# Rebuild and reinstall
cargo build
cp ../target/debug/libros_z_py.so python/ros_z_py/ros_z_py.abi3.so

# Or use maturin
maturin develop
```

### Regenerating Message Types

After modifying message definitions or adding new packages:

```bash
cd ros-z-msgs

# Rebuild with python_registry feature
cargo build --features python_registry

# Reinstall the Python package
cd ../ros-z-py
source .venv/bin/activate
pip install -e ../ros-z-msgs/python/ --force-reinstall
```

### Adding New Message Packages

1. Enable the package feature in `ros-z-msgs/Cargo.toml`
2. Rebuild ros-z-msgs with `cargo build --features python_registry`
3. Import in `ros-z-py/python/ros_z_py/__init__.py`

```bash
ros-z-py/
├── src/                      # Rust source code (PyO3 bindings)
│   ├── lib.rs                # Module entry point
│   ├── session.rs            # Zenoh session management
│   ├── node.rs               # ROS 2 node implementation
│   ├── publisher.rs          # Publisher implementation
│   ├── subscriber.rs         # Subscriber implementation
│   └── ...
├── python/                   # Python package
│   └── ros_z_py/
│       ├── __init__.py       # Package entry point
│       └── ros_z_py.abi3.so  # Compiled Rust extension
├── examples/                 # Example scripts
├── tests/                    # Python tests
├── Cargo.toml                # Rust dependencies
└── pyproject.toml            # Python package metadata

ros-z-msgs/python/            # Separate package for message types
├── ros_z_msgs_py/
│   ├── __init__.py
│   └── types/                # Auto-generated message definitions
│       ├── std_msgs.py
│       ├── geometry_msgs.py
│       └── ...
└── pyproject.toml
```

## Troubleshooting

### Import Error: No module named 'ros_z_msgs_py'

Make sure both packages are installed:

```bash
pip install -e ../ros-z-msgs/python/
pip install -e .
```

### TypeError: Expected msgspec.Struct

Ensure you're passing message objects, not dictionaries:

```python
# Wrong
pub.publish({"data": "hello"})

# Correct
from ros_z_py import std_msgs
pub.publish(std_msgs.String(data="hello"))
```
