# Zenoh PingPong Example

This example demonstrates low-level Zenoh communication for performance testing.

## Complete Example

```rust,no_run
{{#include ../../../ros-z/examples/z_pingpong.rs}}
```

## Purpose

The ping-pong example is useful for:

- Performance benchmarking
- Latency measurement
- Understanding the underlying Zenoh layer
- Network configuration testing

## Usage

```bash
# Run as pong (server)
cargo run --example z_pingpong -- --mode pong

# In another terminal, run as ping (client)
cargo run --example z_pingpong -- --mode ping
```

## Performance Testing

This example helps you:

- Measure round-trip latency
- Test different Zenoh configurations
- Verify network connectivity
- Benchmark message throughput

For application development, prefer the higher-level ros-z APIs shown in other examples.
