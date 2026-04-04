<!-- markdownlint-disable MD046 -->
# Python Quick Start

Get a Python publisher and subscriber running in five minutes.

## Prerequisites

- Python 3.8+
- Rust 1.85+ — install via [rustup](https://rustup.rs)
- `maturin` — `pip install maturin`
- An Eclipse Zenoh router — see [Networking](../user-guide/networking.md)

!!! tip "Using Nix"
    Run `nix develop` in the repo root to get Rust, maturin, and all build tools automatically — skip the manual install steps above.

## 1. Set Up

```bash
cd crates/ros-z-py
python -m venv .venv
source .venv/bin/activate

# Install standard message types
pip install -e ../ros-z-msgs/python/

# Build and install the Python bindings
maturin develop
```

## 2. Write a Publisher

Create `talker.py`:

```python
import time
import ros_z_py
from ros_z_py import std_msgs

ctx = ros_z_py.ZContextBuilder().with_connect_endpoints(["tcp/127.0.0.1:7447"]).build()
node = ctx.create_node("py_talker").build()
pub = node.create_publisher("/chatter", std_msgs.String)

i = 0
while True:
    msg = std_msgs.String(data=f"Hello from Python #{i}")
    pub.publish(msg)
    print(f"Published: {msg.data}")
    i += 1
    time.sleep(1.0)
```

## 3. Write a Subscriber

Create `listener.py`:

```python
import ros_z_py
from ros_z_py import std_msgs

ctx = ros_z_py.ZContextBuilder().with_connect_endpoints(["tcp/127.0.0.1:7447"]).build()
node = ctx.create_node("py_listener").build()
sub = node.create_subscriber("/chatter", std_msgs.String)

print("Listening on /chatter...")
while True:
    msg = sub.recv(timeout=5.0)
    if msg is not None:
        print(f"Received: {msg.data}")
```

## 4. Run

You need a Zenoh router running first:

```bash
# Terminal 1: router
cargo run --example zenoh_router

# Terminal 2: subscriber
source crates/ros-z-py/.venv/bin/activate
python listener.py

# Terminal 3: publisher
source crates/ros-z-py/.venv/bin/activate
python talker.py
```

You should see the subscriber printing messages published by the publisher.

## 5. Try the Built-in Examples

The repo ships ready-to-run Python examples:

```bash
source crates/ros-z-py/.venv/bin/activate

# Pub/sub demo (pass --role talker or --role listener)
python crates/ros-z-py/examples/topic_demo.py --role talker
python crates/ros-z-py/examples/topic_demo.py --role listener

# Service demo
python crates/ros-z-py/examples/service_demo.py --role server
python crates/ros-z-py/examples/service_demo.py --role client

# Action demo
python crates/ros-z-py/examples/action_demo.py --role server
python crates/ros-z-py/examples/action_demo.py --role client
```

## What's Next

- **[Python Bindings](./python.md)** — full API: services, actions, SHM, QoS, context config
- **[Message Generation](../user-guide/message-generation.md)** — generate types from a full ROS 2 install
- **[ROS 2 Interoperability](../user-guide/interop.md)** — connect Python ros-z nodes to live ROS 2 nodes
