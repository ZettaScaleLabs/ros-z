# Advanced Configuration

## Generating Config Files

ros-z can generate JSON5 config files matching `rmw_zenoh_cpp` defaults. This is opt-in via the `generate-configs` feature flag.

### Basic Generation

```bash
cargo build --features generate-configs
```

**Output location:**

```console
target/debug/build/ros-z-*/out/ros_z_config/
  ├── DEFAULT_ROSZ_ROUTER_CONFIG.json5
  └── DEFAULT_ROSZ_SESSION_CONFIG.json5
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
    .with_config_file("./config/DEFAULT_ROSZ_SESSION_CONFIG.json5")
    .build()?;
```

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
