# Networking

**Configure ros-z's Zenoh transport layer for optimal performance in your deployment environment.** ros-z uses router-based architecture by default, matching ROS 2's official `rmw_zenoh_cpp` middleware for production-ready scalability.

## Router-Based Architecture

ros-z uses a centralized Zenoh router for node discovery and communication, providing:

| Benefit | Description |
| --- | --- |
| **Scalability** | Centralized discovery handles large deployments efficiently |
| **Lower Network Overhead** | TCP-based discovery instead of multicast broadcasts |
| **ROS 2 Compatibility** | Matches `rmw_zenoh_cpp` behavior for seamless interoperability |
| **Production Ready** | Battle-tested configuration used in real robotics systems |

## Quick Start

The simplest way to get started is using the built-in router example:

**Terminal 1 - Start the Router:**

```bash
cargo run --example zenoh_router
```

**Terminal 2 - Run Your Application:**

```rust,ignore
use ros_z::context::ZContextBuilder;
use ros_z::Builder;

// Uses default ROS session config (connects to tcp/localhost:7447)
let ctx = ZContextBuilder::default().build()?;
let node = ctx.create_node("my_node").build()?;
```

```admonish success
That's it! The default configuration automatically connects to the router on `tcp/localhost:7447`.
```

## Running the Zenoh Router

### Option 1: Built-in Router Example

ros-z provides a built-in router example that's ROS-compatible out of the box.

```bash
cargo run --example zenoh_router
```

Listens on `tcp/[::]:7447` (all interfaces, port 7447).

### Option 2: Official Zenoh Router

You can also use the official Zenoh router: <https://zenoh.io/docs/getting-started/installation/#installing-the-zenoh-router>.

## Next Steps

Choose the configuration approach that fits your needs:

- **[Configuration Options](./config_options.md)** - Six ways to configure Zenoh (from simple to complex)
- **[Advanced Configuration](./config_advanced.md)** - Generate config files, run routers, configuration reference
- **[Troubleshooting](./troubleshooting.md)** - Solutions to connectivity issues

**Ready to optimize your deployment? Experiment with different configurations and measure performance impact.**
