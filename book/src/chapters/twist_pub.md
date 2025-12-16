# Twist Publisher Example

This example demonstrates publishing `Twist` messages for robot velocity control.

## Complete Example

```rust,no_run
{{#include ../../../ros-z/examples/twist_pub.rs}}
```

## What is Twist?

The `Twist` message type (from `geometry_msgs`) represents velocity in 3D space:

- **Linear**: x, y, z components of linear velocity
- **Angular**: x, y, z components of angular velocity

It's commonly used for:
- Mobile robot control (typically linear.x and angular.z)
- Drone control
- Manipulator end-effector control

## Usage

```bash
cargo run --example twist_pub
```

This is useful for testing robot motion controllers and simulation environments.
