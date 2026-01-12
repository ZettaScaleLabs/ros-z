# Configuration Options

ros-z provides multiple ways to configure Zenoh, from simple to advanced.

## Option 1: Default Configuration (Recommended)

Use the built-in ROS session config for standard deployments:

```rust,ignore
let ctx = ZContextBuilder::default().build()?;
```

**What it does:**

- Connects to router at `tcp/localhost:7447`
- Uses ROS-compatible timeouts and buffer sizes
- Disables multicast discovery (uses router instead)

## Option 2: Custom Router Endpoint

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

## Option 3: Environment Variable Overrides

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

## Option 4: Advanced Configuration Builders

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

## Option 5: Peer Mode Using Multicast Discovery (No Router Required)

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

## Option 6: Load from Config File

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
