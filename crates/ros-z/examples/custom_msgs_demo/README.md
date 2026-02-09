# Custom Messages Demo

This example demonstrates how to generate Rust types from user-defined ROS2 message definitions using `ros-z-codegen`.

## Directory Structure

```console
custom_msgs_demo/
├── my_robot_msgs/              # ROS2 message package (your custom .msg files)
│   ├── msg/
│   │   ├── RobotStatus.msg     # Uses geometry_msgs/Point
│   │   └── SensorReading.msg   # Uses builtin_interfaces/Time
│   └── srv/
│       └── NavigateTo.srv      # Service with geometry_msgs/Point
├── src/
│   └── lib.rs                  # Library that includes generated code
├── Cargo.toml
├── build.rs
└── README.md
```

## Building

From this directory:

```bash
ROS_Z_MSG_PATH="./my_robot_msgs" cargo build
```

Or from the workspace root:

```bash
cd ros-z/examples/custom_msgs_demo
ROS_Z_MSG_PATH="./my_robot_msgs" cargo build
```

## How It Works

1. **Message Definitions**: Your custom `.msg`, `.srv`, and `.action` files go in a ROS2-style package directory (e.g., `my_robot_msgs/msg/`, `my_robot_msgs/srv/`)

2. **Environment Variable**: Set `ROS_Z_MSG_PATH` to point to your message package(s). Multiple paths can be separated by colons.

3. **Build Script**: The `build.rs` calls `ros_z_codegen::generate_user_messages()` which:
   - Discovers packages from `ROS_Z_MSG_PATH`
   - Generates Rust types with external references to `ros_z_msgs` for standard types

4. **Generated Code**: Types are generated in `$OUT_DIR/generated.rs` and included via `include!()` macro

## Using Generated Types

```rust
use custom_msgs_demo::ros::my_robot_msgs::{RobotStatus, SensorReading};
use custom_msgs_demo::ros::my_robot_msgs::srv::NavigateTo;
use ros_z_msgs::ros::geometry_msgs::Point;
use ros_z_msgs::ros::builtin_interfaces::Time;

fn main() {
    let status = RobotStatus {
        robot_id: "robot_1".to_string(),
        battery_percentage: 85.5,
        position: Point { x: 1.0, y: 2.0, z: 0.0 },
        is_moving: true,
    };

    let reading = SensorReading {
        timestamp: Time { sec: 1234, nanosec: 0 },
        values: vec![1.0, 2.0, 3.0],
        sensor_id: "lidar_1".to_string(),
    };
}
```

## Key Features

- **External Type References**: User messages that reference standard types (e.g., `geometry_msgs/Point`) automatically use fully qualified paths to `ros_z_msgs`
- **Zero-Copy Support**: `uint8[]` and `byte[]` fields use `ZBuf` for efficient handling
- **Service Support**: `.srv` files generate Request/Response types and service markers
- **Type Hashes**: RIHS01 type hashes are calculated for ROS2 compatibility
