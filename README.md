<div align="center">
    <h1>ROS-Z</h1>
    <p><strong>Making ROS 2 Zenoh-Native</strong></p>
    <sub>Built by the <a href="https://zenoh.io">Zenoh</a> team at <a href="https://www.zettascale.tech">ZettaScale</a></sub>
</div>

## Overview

**ROS-Z** is a Zenoh-native ROS 2 stack that:

- Provides a pure-Rust ROS 2 implementation built directly on Zenoh
- Preserves portability for RCL-C/CPP/Py-based applications
- Delivers optimized performance for Rust users
- Interoperates seamlessly with Zenoh RMW-based ROS 2


<div align="center">
<img src="./assets/architecture.png" alt="architecture">
</div>

## Status

**ROS-Z** is experimental software. It is tested with ROS 2 Jazzy, Humble, and Kilted. We make no guarantees with respect to other official distributions.

## Current Limitations

ros-z does not yet implement the full ROS 2 feature set. Notable gaps for users migrating from rclcpp/rclpy:

- **No parameter server** — `ros2 param` and `rclcpp::Parameter` have no equivalent
- **No lifecycle nodes** — `rclcpp_lifecycle` is not supported
- **No simulation time** — `/clock` topic and `use_sim_time` are not supported
- **No `wait_for_service()`** — clients must use retry logic or a fixed delay before calling
- **Service interop is partial** — pub/sub interop with `rmw_zenoh_cpp` nodes works well; service call interop has known limitations
- **ROS 2 interop requires `rmw_zenoh_cpp`** — the Zenoh router alone is not sufficient; the ROS 2 side must use `RMW_IMPLEMENTATION=rmw_zenoh_cpp`

## Documentation

📚 **[Read the Book](https://zettascalelabs.github.io/ros-z/)** for comprehensive documentation including:

- Installation and build instructions
- Examples and tutorials
- API reference
- Feature flags and configuration
- Contributing guidelines

**Local Development:** `mdbook serve book`

## License

[View license](LICENSE)

## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for details.
