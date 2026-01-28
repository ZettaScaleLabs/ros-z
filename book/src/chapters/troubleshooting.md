# Troubleshooting

**Quick solutions to common ros-z build and runtime issues.** Click on any question to expand the answer.

```admonish tip
Most issues fall into three categories: build configuration, runtime connectivity, or ROS 2 integration.
```

## Build Issues

````admonish question collapsible=true title="Build fails with 'Cannot find ROS packages' or package discovery errors"
**Root Cause:** ROS 2 environment not sourced or packages not installed.

**Solutions:**

1. **Source ROS 2 environment:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   # or for rolling:
   source /opt/ros/rolling/setup.bash
   ```

2. **Verify environment variables:**
   ```bash
   echo $AMENT_PREFIX_PATH
   echo $CMAKE_PREFIX_PATH
   ```

3. **Check package installation:**
   ```bash
   ros2 pkg prefix example_interfaces
   # If fails, install:
   sudo apt install ros-jazzy-example-interfaces
   ```

4. **Clean and rebuild:**
   ```bash
   cargo clean -p ros-z-msgs
   cargo build -p ros-z-msgs
   ```

**Common Error Messages:**

| Error | Solution |
|-------|----------|
| "Package X not found" | Source ROS 2 environment |
| "Cannot find ament_index" | Install ROS 2 or use bundled msgs |
| "AMENT_PREFIX_PATH not set" | Run `source /opt/ros/jazzy/setup.bash` |
````

````admonish question collapsible=true title="Compiler error: cannot find crate ros_z_msgs"
**Root Cause:** `ros-z-msgs` is not part of default workspace members.

**Solution:**

```bash
# Build ros-z-msgs explicitly
cargo build -p ros-z-msgs

# Then build your example
cargo build --example z_srvcli
```

**Note:** `ros-z-msgs` is excluded from default builds to avoid requiring ROS 2 for core development. Build it explicitly when needed.
````

````admonish question collapsible=true title="Build takes too long to complete"
**Solutions:**

```bash
# Use parallel builds (automatic on most systems)
cargo build -j $(nproc)

# Build only what you need
cargo build -p ros-z-msgs --features std_msgs,geometry_msgs
```
````

````admonish question collapsible=true title="Linker errors during build (especially with rcl-z)"
**Solution:**

```bash
# Clear cache and rebuild
cargo clean
source /opt/ros/jazzy/setup.bash
cargo build -p rcl-z
```

**Warning:** After changing feature flags or updating ROS 2, run `cargo clean -p ros-z-msgs` to force message regeneration.
````

## Runtime Issues

````admonish question collapsible=true title="Publishers and subscribers on different processes don't communicate"
**Root Cause:** Zenoh router not running or nodes not configured correctly.

**Solution:**

1. **Ensure the router is running:**
   ```bash
   cargo run --example zenoh_router
   ```

2. **Verify endpoint matches in your code:**
   ```rust,ignore
   let ctx = ZContextBuilder::default()
       .with_router_endpoint("tcp/localhost:7447")  // Must match router
       .build()?;
   ```
````

````admonish question collapsible=true title="Router fails to start with 'Address already in use' error"
**Root Cause:** Another process is using port 7447.

**Solutions:**

1. **Stop the conflicting process**

2. **Use a custom port:**
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
````

````admonish question collapsible=true title="Don't want to manage a router for simple local testing"
**Solution:** Use peer mode with multicast discovery:

```rust,ignore
let ctx = ZContextBuilder::default()
    .with_zenoh_config(zenoh::Config::default())
    .build()?;
```

**Warning:** Peer mode won't interoperate with ROS 2 nodes using `rmw_zenoh_cpp` in router mode.
````

## Resources

- **[Building Guide](./building.md)** - Correct build procedures
- **[Networking](./networking.md)** - Zenoh router setup
- **[Feature Flags](./feature_flags.md)** - Available features
- **[GitHub Issues](https://github.com/ZettaScaleLabs/ros-z/issues)** - Report bugs

**Most issues are environmental. Verify your setup matches the build scenario requirements before diving deeper.**
