# Running Examples

Once you've added ros-z to your project, you can run the included examples to see it in action.

````admonish important
**All examples require a Zenoh router to be running first** (see [Networking](./config.md) for why ros-z uses router-based architecture by default):
```bash
cargo run --example zenoh_router
```
````

Leave this running in a separate terminal, then run any example in another terminal.

## Available Examples

```bash
# Pure Rust example with custom messages (no ros-z-msgs needed)
cargo run --example z_custom_message -- --mode status-pub

# Examples using bundled messages (requires ros-z-msgs)
cargo run --example z_pubsub          # Publisher/Subscriber with std_msgs
cargo run --example twist_pub         # Publishing geometry_msgs
cargo run --example battery_state_sub # Receiving sensor_msgs

# Examples requiring ROS 2 (requires external_msgs feature)
cargo run --example z_srvcli --features external_msgs
```

See the [Networking](./config.md) chapter for router setup details and alternative configurations.
