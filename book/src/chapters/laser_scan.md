# Laser Scan Example

This example demonstrates working with laser scan sensor data.

## Complete Example

```rust,no_run
{{#include ../../../ros-z/examples/laser_scan.rs}}
```

## LaserScan Message

The `LaserScan` message (from `sensor_msgs`) represents data from a 2D laser range finder:

- Range measurements at regular angular intervals
- Minimum/maximum ranges and angles
- Scan time and time increment
- Intensity values (if available)

## Common Use Cases

- Obstacle detection
- Mapping and localization (SLAM)
- Safety systems
- Navigation

## Usage

```bash
cargo run --example laser_scan
```

This example is useful for working with LIDAR sensors and range finders.
