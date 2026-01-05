# Zenoh Configuration

**Configure ros-z's Zenoh transport layer for optimal performance in your deployment environment.** ros-z uses router-based architecture by default, matching ROS 2's official `rmw_zenoh_cpp` middleware for production-ready scalability.

## Router-Based Architecture

ros-z uses a centralized Zenoh router for node discovery and communication, providing:

| Benefit | Description |
|---------|-------------|
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

## Configuration Options

ros-z provides multiple ways to configure Zenoh, from simple to advanced.

### Option 1: Default Configuration (Recommended)

Use the built-in ROS session config for standard deployments:

```rust,ignore
let ctx = ZContextBuilder::default().build()?;
```

**What it does:**

- Connects to router at `tcp/localhost:7447`
- Uses ROS-compatible timeouts and buffer sizes
- Disables multicast discovery (uses router instead)

### Option 2: Custom Router Endpoint

Connect to a router on a different host or port:

```rust,ignore
let ctx = ZContextBuilder::default()
    .with_router_endpoint("tcp/192.168.1.100:7447")
    .build()?;
```

**Use cases:**

- Distributed systems with remote router
- Custom port configurations
- Multiple isolated networks

### Option 3: Environment Variable Overrides

Override any Zenoh configuration setting using the `ROSZ_CONFIG_OVERRIDE` environment variable without changing code:

```bash
# Override mode and endpoint
export ROSZ_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.1.100:7447"]'

# Run your application
cargo run --example my_app
```

```rust,ignore
// No code changes needed - overrides are applied automatically
let ctx = ZContextBuilder::default().build()?;
```

**Format:**

- Semicolon-separated `key=value` pairs
- Values use JSON5 syntax
- Keys use slash-separated paths (e.g., `connect/endpoints`, `scouting/multicast/enabled`)

**Common examples:**

```bash
# Connect to remote router
export ROSZ_CONFIG_OVERRIDE='connect/endpoints=["tcp/10.0.0.5:7447"]'

# Enable multicast scouting explicitly
export ROSZ_CONFIG_OVERRIDE='scouting/multicast/enabled=true'

# Multiple overrides
export ROSZ_CONFIG_OVERRIDE='mode="client";connect/timeout_ms=5000;scouting/multicast/enabled=false'
```

```admonish tip
Environment variable overrides have the highest priority and will override any programmatic configuration or config file settings.
```

### Option 4: Advanced Configuration Builders

Fine-tune session or router settings programmatically:

```rust,ignore
use ros_z::config::{SessionConfigBuilder, RouterConfigBuilder};

// Customize session config
let session_config = SessionConfigBuilder::new()
    .with_router_endpoint("tcp/192.168.1.100:7447")
    .build()?;

let ctx = ZContextBuilder::default()
    .with_zenoh_config(session_config)
    .build()?;

// Or build a custom router config
let router_config = RouterConfigBuilder::new()
    .with_listen_port(7448)  // Custom port
    .build()?;

zenoh::open(router_config).await?;
```

**Key builder methods:**

| Builder | Methods | Purpose |
|---------|---------|---------|
| `SessionConfigBuilder` | `with_router_endpoint(endpoint)` | Connect to custom router |
| `RouterConfigBuilder` | `with_listen_port(port)` | Set custom router port |

### Option 5: Peer Mode Using Multicast Discovery (No Router Required)

Revert to multicast peer discovery for simple setups:

```rust,ignore
// Use vanilla Zenoh config (peer mode with multicast)
let ctx = ZContextBuilder::default()
    .with_zenoh_config(zenoh::Config::default())
    .build()?;
```

```admonish warning
Multicast scouting discovery is convenient for quick testing but doesn't scale well and won't work with ROS 2 nodes using `rmw_zenoh_cpp` (which expects a zenoh router).
```

### Option 6: Load from Config File

Use JSON5 config files for complex deployments:

```rust,ignore
let ctx = ZContextBuilder::default()
    .with_config_file("/etc/zenoh/session_config.json5")
    .build()?;
```

**When to use:**

- Deploying across multiple machines
- Environment-specific configurations
- Version-controlled infrastructure

## (Advanced) Generating Config Files

ros-z can generate JSON5 config files matching `rmw_zenoh_cpp` defaults. This is opt-in via the `generate-configs` feature flag.

### Basic Generation

```bash
cargo build --features generate-configs
```

**Output location:**

```console
target/debug/build/ros-z-*/out/ros_z_config/
  ├── DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5
  └── DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5
```

### Custom Output Directory

Specify a custom directory using the `ROS_Z_CONFIG_OUTPUT_DIR` environment variable:

**Absolute path:**

```bash
ROS_Z_CONFIG_OUTPUT_DIR=/etc/zenoh cargo build --features generate-configs
```

**Relative path (from package root):**

```bash
ROS_Z_CONFIG_OUTPUT_DIR=./config cargo build --features generate-configs
```

**From workspace root:**

```bash
ROS_Z_CONFIG_OUTPUT_DIR=$PWD/config cargo build -p ros-z --features generate-configs
```

```admonish tip
Generated files include inline comments explaining each setting, making them perfect documentation references.
```

### Using Generated Files

```rust,ignore
let ctx = ZContextBuilder::default()
    .with_config_file("./config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5")
    .build()?;
```

## Running the Zenoh Router

### Option 1: Default Router

ros-z provides a built-in router example that's ROS-compatible out of the box.

```bash
cargo run --example zenoh_router
```

Listens on `tcp/[::]:7447` (all interfaces, port 7447).

### Option 2: Official Zenoh Router

You can also use the official Zenoh router: <https://zenoh.io/docs/getting-started/installation/#installing-the-zenoh-router>.

## Configuration Reference

### Key Settings Explained

| Setting | Router | Session | Purpose |
|---------|--------|---------|---------|
| **Mode** | `router` | `peer` | Router relays messages, peers connect directly |
| **Listen Endpoint** | `tcp/[::]:7447` | - | Router accepts connections |
| **Connect Endpoint** | - | `tcp/localhost:7447` | Session connects to router |
| **Multicast** | Disabled | Disabled | Uses TCP gossip for discovery |
| **Unicast Timeout** | 60s | 60s | Handles slow networks/large deployments |
| **Query Timeout** | 10min | 10min | Long-running service calls |
| **Max Sessions** | 10,000 | - | Supports concurrent node startup |
| **Keep-Alive** | 2s | 2s | Optimized for loopback |

```admonish note
These defaults are tuned for ROS 2 deployments and match `rmw_zenoh_cpp` exactly. Only modify them if you have specific performance requirements.
```

## Troubleshooting

### Nodes Can't Find Each Other

**Symptom:** Publishers and subscribers on different processes don't communicate.

**Solution:** Ensure the Zenoh router is running and both nodes connect to it:

```bash
# Check router is running
cargo run --example zenoh_router

# Verify endpoint matches in your code
let ctx = ZContextBuilder::default()
    .with_router_endpoint("tcp/localhost:7447")  // Must match router
    .build()?;
```

### Router Port Already in Use

**Symptom:** Router fails to start with "Address already in use" error.

**Solution:** Another process is using port 7447. Either:

- Stop the conflicting process
- Use a custom port for the router and update session configs

```rust,ignore
// Custom router port
let router_config = RouterConfigBuilder::new()
    .with_listen_port(7448)
    .build()?;

// Connect sessions to custom port
let ctx = ZContextBuilder::default()
    .with_router_endpoint("tcp/localhost:7448")
    .build()?;
```

### Want Multicast Discovery Back

**Symptom:** Don't want to manage a router for simple local testing.

**Solution:** Use peer mode:

```rust,ignore
let ctx = ZContextBuilder::default()
    .with_zenoh_config(zenoh::Config::default())
    .build()?;
```

```admonish warning
Peer mode won't interoperate with ROS 2 nodes using `rmw_zenoh_cpp` in router mode.
```

## Next Steps

- **[Publishers & Subscribers](./pubsub.md)** - Build communication patterns
- **[Services](./services.md)** - Implement request-response
- **[Troubleshooting](./troubleshooting.md)** - Solve common issues

**Ready to optimize your deployment? Experiment with different configurations and measure performance impact.**
