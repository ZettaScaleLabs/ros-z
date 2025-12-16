# Troubleshooting

This guide covers common issues you might encounter when building or using ros-z, along with their solutions.

## Build Issues

### Cannot Find ROS Packages

**Problem:** Build with `external_msgs` fails with "Cannot find ROS packages" or similar error.

**Solutions:**

1. Ensure ROS 2 is sourced:

   ```bash
   source /opt/ros/jazzy/setup.bash
   # or for rolling:
   source /opt/ros/rolling/setup.bash
   ```

2. Check environment variables:

   ```bash
   echo $AMENT_PREFIX_PATH
   echo $CMAKE_PREFIX_PATH
   ```

   These should show paths to your ROS installation.

3. Verify the package exists:

   ```bash
   ros2 pkg prefix example_interfaces
   ```

   If this fails, install the package:

   ```bash
   sudo apt install ros-jazzy-example-interfaces
   ```

4. Clear cargo cache and rebuild:

   ```bash
   cargo clean
   cargo build -p ros-z-msgs --features external_msgs
   ```

### Cannot Find Crate `ros_z_msgs`

**Problem:** Error message like "cannot find crate `ros_z_msgs`" when building examples.

**Solution:**

The `ros-z-msgs` package is not part of the default workspace. Build it explicitly:

```bash
cargo build -p ros-z-msgs
```

For examples using external messages:

```bash
cargo build -p ros-z-msgs --features external_msgs
cargo build --example z_srvcli --features external_msgs
```

### Linker Errors with RCL

**Problem:** Linker errors when building `rcl-z` like "undefined reference to `rcl_init`".

**Solutions:**

1. Ensure ROS 2 is sourced before building:

   ```bash
   source /opt/ros/jazzy/setup.bash
   cargo build -p rcl-z
   ```

2. Check that RCL libraries are installed:

   ```bash
   dpkg -l | grep ros-jazzy-rcl
   ```

3. If using a custom ROS installation, set `CMAKE_PREFIX_PATH`:

   ```bash
   export CMAKE_PREFIX_PATH=/path/to/ros/install:$CMAKE_PREFIX_PATH
   ```

### Build Takes Too Long

**Problem:** Compilation is very slow, especially for message generation.

**Solutions:**

1. Use parallel builds (usually automatic):

   ```bash
   cargo build -j $(nproc)
   ```

2. Build only what you need:

   ```bash
   # Instead of building all messages:
   cargo build -p ros-z-msgs --features bundled_msgs

   # Build only specific packages:
   cargo build -p ros-z-msgs --features std_msgs,geometry_msgs
   ```

3. Use the optimized profile for faster runtime with reasonable compile time:

   ```bash
   cargo build --profile opt
   ```

4. Enable incremental compilation (should be on by default):

   ```bash
   export CARGO_INCREMENTAL=1
   ```

### Nix Build Failures

**Problem:** Build fails inside `nix develop` shell.

**Solutions:**

1. Ensure you're using the correct shell:

   ```bash
   # For pure Rust development:
   nix develop .#pureRust

   # For ROS 2 Jazzy:
   nix develop .#ros-jazzy
   ```

2. Clean and rebuild:

   ```bash
   cargo clean
   nix develop .#ros-jazzy --command cargo build
   ```

3. Update flake inputs:

   ```bash
   nix flake update
   ```

## Runtime Issues

### No Messages Received

**Problem:** Subscriber doesn't receive messages from publisher.

**Diagnostics:**

1. Check if both nodes are running:

   ```bash
   # In one terminal:
   cargo run --example demo_nodes_talker

   # In another:
   cargo run --example demo_nodes_listener
   ```

2. Verify topic names match:

   ```bash
   # Add logging to see what's happening:
   RUST_LOG=debug cargo run --example demo_nodes_listener
   ```

3. Check Zenoh connectivity:

   ```bash
   # Enable Zenoh session tracing:
   RUST_LOG=zenoh::api::session=trace cargo run --example demo_nodes_listener
   ```

**Common causes:**

- **Different topic names**: Ensure publisher and subscriber use the same topic
- **QoS mismatch**: Check that QoS profiles are compatible
- **Network issues**: Verify Zenoh can discover peers (check firewall)
- **Late subscription**: Subscriber started after messages were sent with transient_local

**Solutions:**

1. Use consistent topic names:

   ```bash
   cargo run --example demo_nodes_talker -- --topic /test
   cargo run --example demo_nodes_listener -- --topic /test
   ```

2. Check QoS settings in code:

   ```rust,ignore
   // Ensure compatible QoS
   let qos = QosProfile::default()
       .history(HistoryPolicy::KeepLast(10))
       .reliability(ReliabilityPolicy::Reliable);
   ```

3. Add endpoint for explicit connectivity:

   ```bash
   cargo run --example demo_nodes_talker -- --endpoint tcp/localhost:7447
   cargo run --example demo_nodes_listener -- --endpoint tcp/localhost:7447
   ```

### Service Call Hangs

**Problem:** Service client hangs waiting for response.

**Diagnostics:**

1. Verify server is running:

   ```bash
   cargo run --example demo_nodes_add_two_ints_server
   ```

2. Check service name:

   ```bash
   RUST_LOG=debug cargo run --example demo_nodes_add_two_ints_client
   ```

3. Test with ROS 2 CLI if available:

   ```bash
   ros2 service list
   ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
   ```

**Solutions:**

- Ensure server is running before client sends request
- Verify service names match exactly
- Check that request/response types match
- Add timeout to client calls to avoid infinite blocking

### Message Serialization Errors

**Problem:** Errors like "failed to serialize message" or "failed to deserialize message".

**Common causes:**

- Incompatible message versions between publisher and subscriber
- Corrupted message data
- Wrong message type used

**Solutions:**

1. Ensure all nodes use the same message definitions:

   ```bash
   # Rebuild all with consistent versions:
   cargo clean
   cargo build -p ros-z-msgs
   cargo build --examples
   ```

2. Verify message types match:

   ```rust,ignore
   // Publisher and subscriber must use the same type:
   pub.publish(&msg)?;  // RosString
   sub.recv()?;         // Must expect RosString, not String
   ```

3. Enable detailed error logging:

   ```bash
   RUST_LOG=ros_z=trace cargo run --example demo_nodes_listener
   ```

## ROS Interoperability Issues

### Cannot Communicate with ROS 2 Nodes

**Problem:** ros-z nodes can't communicate with standard ROS 2 nodes.

**Prerequisites:**

Ensure both systems use Zenoh:

- ros-z uses Zenoh natively
- ROS 2 needs `rmw_zenoh` or Zenoh bridge

**Solutions:**

1. If using ROS 2 with DDS (default), you need the Zenoh-DDS bridge:

   ```bash
   # Install zenoh-bridge-dds
   cargo install zenoh-bridge-dds

   # Run the bridge:
   zenoh-bridge-dds
   ```

2. Or use ROS 2 with rmw_zenoh:

   ```bash
   # Install rmw_zenoh
   sudo apt install ros-jazzy-rmw-zenoh-cpp

   # Run ROS 2 node with Zenoh:
   RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run demo_nodes_cpp talker
   ```

3. Verify both can see each other:

   ```bash
   # From ROS 2 side:
   ros2 topic list

   # From ros-z side with debug logging:
   RUST_LOG=debug cargo run --example demo_nodes_listener
   ```

### Wrong Message Format Between ROS 2 and ros-z

**Problem:** Messages received are corrupted or malformed when communicating with ROS 2.

**Solutions:**

1. Ensure message definitions match:

   ```bash
   # Compare message definitions:
   ros2 interface show std_msgs/msg/String
   ```

2. Verify CDR serialization compatibility:
   - ros-z uses CDR serialization by default (compatible with ROS 2)
   - Check that custom messages follow ROS 2 conventions

3. Test with standard messages first:

   ```bash
   # Use std_msgs which are well-defined:
   cargo run --example demo_nodes_talker  # Uses std_msgs::String
   ros2 topic echo /chatter               # Should work
   ```

## Message Generation Issues

### Message Generation Fails

**Problem:** `ros-z-msgs` build fails during message generation.

**Solutions:**

1. Check message definition syntax:

   ```bash
   # Validate message files:
   ros2 interface show geometry_msgs/msg/Twist
   ```

2. Clear generated code and regenerate:

   ```bash
   cargo clean -p ros-z-msgs
   cargo build -p ros-z-msgs
   ```

3. If using external messages, ensure packages are installed:

   ```bash
   ros2 pkg prefix geometry_msgs
   ros2 pkg prefix sensor_msgs
   ```

4. Check for conflicting message definitions:

   ```bash
   # Remove old generated files:
   find target -name "*_msgs" -type d -exec rm -rf {} +
   cargo build -p ros-z-msgs
   ```

### Custom Message Not Found

**Problem:** Custom message type not available after generation.

**Solution:**

1. Ensure feature is enabled:

   ```bash
   cargo build -p ros-z-msgs --features my_custom_msgs
   ```

2. Check `Cargo.toml` includes your custom package:

   ```toml
   [features]
   my_custom_msgs = []
   ```

3. Verify message is in correct location:

   ```text
   ros-z-msgs/
   └── src/
       └── my_custom_msgs/
           └── msg/
               └── MyMessage.msg
   ```

## Zenoh Configuration Issues

### Zenoh Discovery Not Working

**Problem:** Nodes on different machines can't discover each other.

**Solutions:**

1. Use explicit endpoints:

   ```bash
   # On machine 1:
   cargo run --example demo_nodes_talker -- --endpoint tcp/192.168.1.100:7447

   # On machine 2:
   cargo run --example demo_nodes_listener -- --endpoint tcp/192.168.1.100:7447
   ```

2. Check firewall settings:

   ```bash
   # Allow Zenoh default port (7447):
   sudo ufw allow 7447/tcp
   ```

3. Use multicast if available:

   ```bash
   # Enable multicast scouting (default):
   ZENOH_SCOUTING=multicast cargo run --example demo_nodes_talker
   ```

4. Use a Zenoh router for complex networks:

   ```bash
   # Install and run zenoh router:
   cargo install zenohd
   zenohd

   # Connect nodes to router:
   cargo run --example demo_nodes_talker -- --endpoint tcp/router-ip:7447
   ```

### High Latency Between Nodes

**Problem:** Messages take too long to arrive.

**Solutions:**

1. Check network configuration:

   ```bash
   # Test network latency:
   ping other-machine
   ```

2. Use direct connection instead of router:

   ```bash
   cargo run --example demo_nodes_listener -- --endpoint tcp/peer-ip:7447
   ```

3. Optimize QoS settings:

   ```rust,ignore
   let qos = QosProfile::default()
       .reliability(ReliabilityPolicy::BestEffort)  // Faster than Reliable
       .history(HistoryPolicy::KeepLast(1));        // Minimize buffering
   ```

4. Use the z_pingpong example to measure latency:

   ```bash
   # Terminal 1:
   cargo run --example z_pingpong -- --mode pong

   # Terminal 2:
   cargo run --example z_pingpong -- --mode ping
   ```

## Getting Help

If you're still experiencing issues:

1. **Check the logs**: Use `RUST_LOG=debug` or `RUST_LOG=trace` for detailed output
2. **Search issues**: Check [GitHub Issues](https://github.com/ZettaScaleLabs/ros-z/issues)
3. **Ask for help**: Open a new issue with:
   - Your environment (OS, ROS version, Rust version)
   - Complete error messages
   - Steps to reproduce
   - What you've already tried

### Useful Debug Commands

```bash
# Full debug output:
RUST_LOG=trace cargo run --example demo_nodes_talker 2>&1 | tee debug.log

# Check versions:
cargo --version
rustc --version
ros2 --version  # if using ROS 2

# Environment info:
env | grep ROS
env | grep AMENT
env | grep CMAKE

# Cargo tree to check dependencies:
cargo tree -p ros-z-msgs
```

## Next Steps

- Review [Building](./building.md) for correct build procedures
- Check [Feature Flags](./feature_flags.md) for feature requirements
- See [Examples](./examples_overview.md) for working code
- Open an issue on [GitHub](https://github.com/ZettaScaleLabs/ros-z/issues) to report bugs or contribute fixes
