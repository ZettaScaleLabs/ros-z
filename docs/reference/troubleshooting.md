# Troubleshooting

**Quick solutions to common ros-z build and runtime issues.** Click on any question to expand the answer.

!!! tip
    Most issues fall into three categories: build configuration, runtime connectivity, or ROS 2 integration.

## Build Issues

??? question "Build fails with "
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

??? question "Compiler error: cannot find crate ros_z_msgs"
    **Root Cause:** `ros-z-msgs` is not part of default workspace members.

    **Solution:**

    ```bash
    # Build ros-z-msgs explicitly
    cargo build -p ros-z-msgs

    # Then build your example
    cargo build --example z_srvcli
    ```

    **Note:** `ros-z-msgs` is excluded from default builds to avoid requiring ROS 2 for core development. Build it explicitly when needed.

??? question "Build takes too long to complete"
    **Solutions:**

    ```bash
    # Use parallel builds (automatic on most systems)
    cargo build -j $(nproc)

    # Build only what you need
    cargo build -p ros-z-msgs --features std_msgs,geometry_msgs
    ```

??? question "Linker errors during build (especially with rcl-z)"
    **Solution:**

    ```bash
    # Clear cache and rebuild
    cargo clean
    source /opt/ros/jazzy/setup.bash
    cargo build -p rcl-z
    ```

    **Warning:** After changing feature flags or updating ROS 2, run `cargo clean -p ros-z-msgs` to force message regeneration.

## Runtime Issues

??? question "Publishers and subscribers on different processes don"
    **Root Cause:** Zenoh router not running or nodes not configured correctly.

    **Solution:**

    1. **Ensure the router is running:**
       ```bash
       cargo run --example zenoh_router
       ```

    2. **Verify endpoint matches in your code:**
       ```rust
       let ctx = ZContextBuilder::default()
           .with_router_endpoint("tcp/localhost:7447")?  // Must match router
           .build()?;
       ```

??? question "Router fails to start with "
    **Root Cause:** Another process is using port 7447.

    **Solutions:**

    1. **Stop the conflicting process**

    2. **Use a custom port:**

       Start the router on a different port:
       ```bash
       cargo run --example zenoh_router -- --listen "tcp/[::]:7448"
       ```

       Then connect your sessions to that port:
       ```rust
       let ctx = ZContextBuilder::default()
           .with_router_endpoint("tcp/localhost:7448")?
           .build()?;
       ```

??? question "Don"
    **Solution:** Use peer mode with multicast discovery:

    ```rust
    let ctx = ZContextBuilder::default()
        .with_mode("peer")
        .build()?;
    ```

    **Warning:** Peer mode won't interoperate with ROS 2 nodes using `rmw_zenoh_cpp` in router mode.

??? question "Multi-segment topics like /robot/sensors/camera don"
    **Symptom:** Publisher publishes to `/robot/sensors/camera` but subscriber never receives messages.

    **Root Cause:** Old versions of ros-z (before 0.1.0) incorrectly mangled slashes in topic key expressions, breaking multi-segment topic routing.

    **Solution:** Update to ros-z 0.1.0+ which correctly preserves internal slashes in topic key expressions.

    **Verify the fix:**

    ```bash
    # Enable debug logging to see key expressions
    RUST_LOG=ros_z=debug cargo run --example z_pubsub
    ```

    Look for key expressions in the output:

    | Key Expression | Status |
    |---------------|--------|
    | `0/robot/sensors/camera/...` | ✅ Correct (slashes preserved) |
    | `0/robot%sensors%camera/...` | ❌ Wrong (slashes mangled) |

    **Technical Details:**

    - **Topic key expressions** should use `strip_slashes()`: removes leading/trailing slashes, preserves internal slashes
    - **Liveliness tokens** should use `mangle_name()`: replaces all `/` with `%`
    - This matches the behavior of `rmw_zenoh_cpp`

    See [Key Expression Formats](../experimental/keyexpr-formats.md#key-expression-behavior-important) for details.

??? question "ROS 2 nodes don"
    **Root Cause:** Type hash mismatch — ros-z and the ROS 2 node disagree on the message definition's RIHS01 hash.

    **Diagnosis:**

    Enable debug logging to see the key expressions being advertised:

    ```bash
    RUST_LOG=ros_z=debug cargo run --example z_pubsub
    ```

    Look for the hash in the key expression output:

    ```
    [PUB] Key expression: 0/chatter/std_msgs::msg::dds_::String_/RIHS01_<hash>
    ```

    Compare the `RIHS01_<hash>` with what the ROS 2 node advertises (visible with `ZENOH_LOGS=debug` on the ROS 2 side). If the hashes differ, the nodes will not exchange messages.

    **Common causes:**

    | Cause | Fix |
    |-------|-----|
    | Wrong package used (e.g. `action_tutorials_interfaces` vs `example_interfaces`) | Use the exact package the ROS 2 node uses — check with `ros2 interface show` |
    | Message definition mismatch (extra/missing field) | Verify field names match: `ros2 interface show <pkg>/msg/<Msg>` |
    | ros-z-msgs generated from stale `.msg` files | `cargo clean -p ros-z-msgs && cargo build -p ros-z-msgs` |

    **Verify the correct type:**

    ```bash
    ros2 interface show std_msgs/msg/String
    ```

    Then check that `ros-z-codegen/assets/jazzy/std_msgs/msg/String.msg` matches exactly.

## Resources

- **[Building Guide](../getting-started/building.md)** - Correct build procedures
- **[Networking](../user-guide/networking.md)** - Zenoh router setup
- **[Feature Flags](./feature-flags.md)** - Available features
- **[GitHub Issues](https://github.com/ZettaScaleLabs/ros-z/issues)** - Report bugs

**Most issues are environmental. Verify your setup matches the build scenario requirements before diving deeper.**
