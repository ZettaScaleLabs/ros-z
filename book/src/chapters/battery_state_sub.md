# Battery State Subscriber Example

This example demonstrates subscribing to battery status messages.

## Complete Example

```rust,ignore
{{#include ../../../ros-z/examples/battery_state_sub.rs}}
```

## Battery State Message

The `BatteryState` message (from `sensor_msgs`) provides comprehensive battery information:

- Voltage, current, and charge
- Battery health and status
- Temperature
- Cell voltages (for multi-cell batteries)

## Usage

```bash
cargo run --example battery_state_sub
```

This is useful for monitoring robot power systems and implementing low-battery behaviors.
