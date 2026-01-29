# Running Examples

```admonish note
The examples described here are part of the ros-z repository. To run them, you must first clone the repository:

```bash
git clone https://github.com/ZettaScaleLabs/ros-z.git
cd ros-z
```

<!-- markdownlint-disable-next-line MD040 -->
```

## Start the Zenoh Router

All examples require a Zenoh router to be running first (see [Networking](./networking.md) for why ros-z uses router-based architecture by default).

## From the ros-z Repository

If you're working in the ros-z repository, use the included router example:

```bash
cargo run --example zenoh_router
```

## From Your Own Project

If you're working on your own project, you need to install a Zenoh router. Quick options:

```bash
# Using cargo
cargo install zenoh

# Using Docker
docker run --init --net host eclipse/zenoh:latest

# Using apt (Ubuntu/Debian)
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list
sudo apt update && sudo apt install zenoh
```

Then run:

```bash
zenohd
```

```admonish tip
See the comprehensive [Zenoh Router Installation Guide](./networking.md#running-the-zenoh-router) for all installation methods including pre-built binaries, package managers, and more.
```

---

## Available Examples

Leave the router running in a separate terminal, then run any example in another terminal from the ros-z repository root:

```bash
# Pure Rust example with custom messages (no ros-z-msgs needed)
cargo run --example z_custom_message -- --mode status-pub

# Examples using bundled messages (requires ros-z-msgs)
cargo run --example z_pubsub          # Publisher/Subscriber with std_msgs
cargo run --example twist_pub         # Publishing geometry_msgs
cargo run --example battery_state_sub # Receiving sensor_msgs
cargo run --example z_srvcli          # Service example with example_interfaces
```

```admonish tip
For a detailed walkthrough of creating your own project with ros-z (not using the repository examples), see the [Quick Start](./quick_start.md#option-2-create-your-own-project) guide.
```
