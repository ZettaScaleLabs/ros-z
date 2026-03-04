# Parameters

**ros-z implements the full ROS 2 parameter subsystem for native nodes.** Declare, get, set, and validate typed parameters at runtime — with config file loading, range constraints, and standard parameter services that interoperate with `ros2 param`.

```admonish note
Parameters let you configure node behavior at runtime without recompilation. ros-z parameters are fully compatible with the ROS 2 parameter protocol, so `ros2 param list`, `ros2 param get`, and `ros2 param set` work out of the box against ros-z nodes.
```

## Visual Flow

```mermaid
graph TD
    A[ZNodeBuilder] -->|configure| B[ZNode]
    B -->|owns| C[ParameterStore]
    B -->|owns| D[ParameterService]
    D -->|hosts| E[6 ZServers]
    D -->|publishes| F[/parameter_events]
    E --> G[get_parameters]
    E --> H[set_parameters]
    E --> I[list_parameters]
    E --> J[describe_parameters]
    E --> K[get_parameter_types]
    E --> L[set_parameters_atomically]
```

## Key Features

| Feature | Description |
|---------|-------------|
| **Typed values** | 9 parameter types: bool, integer, double, string, byte/bool/integer/double/string arrays |
| **Range validation** | `FloatingPointRange` and `IntegerRange` constraints on descriptors |
| **Read-only** | Parameters that reject all changes after declaration |
| **YAML loading** | Load initial values from ROS 2 parameter YAML files with `/**` wildcard support |
| **Overrides** | Programmatic overrides applied at declaration time |
| **Validation callbacks** | Accept or reject changes with a reason string |
| **Standard services** | 6 parameter services compatible with `ros2 param` CLI |
| **Parameter events** | `/parameter_events` topic published on every change |

## Quick Start

```rust,ignore
use ros_z::{Builder, parameter::*};

let node = ctx.create_node("my_node").build()?;

// Declare a parameter with a descriptor
let desc = ParameterDescriptor::new("max_speed", ParameterType::Double);
node.declare_parameter("max_speed", ParameterValue::Double(1.0), desc)?;

// Get and set
let value = node.get_parameter("max_speed"); // Some(Double(1.0))
node.set_parameter(Parameter::new("max_speed", ParameterValue::Double(2.5)))?;
```

## Parameter Types

| Type | Rust variant | Wire type ID |
|------|-------------|--------------|
| Not set | `ParameterValue::NotSet` | 0 |
| Bool | `ParameterValue::Bool(bool)` | 1 |
| Integer | `ParameterValue::Integer(i64)` | 2 |
| Double | `ParameterValue::Double(f64)` | 3 |
| String | `ParameterValue::String(String)` | 4 |
| Byte array | `ParameterValue::ByteArray(Vec<u8>)` | 5 |
| Bool array | `ParameterValue::BoolArray(Vec<bool>)` | 6 |
| Integer array | `ParameterValue::IntegerArray(Vec<i64>)` | 7 |
| Double array | `ParameterValue::DoubleArray(Vec<f64>)` | 8 |
| String array | `ParameterValue::StringArray(Vec<String>)` | 9 |

## Declaring Parameters

Parameters must be declared before use. A `ParameterDescriptor` specifies the name, expected type, and optional constraints:

```rust,ignore
use ros_z::parameter::*;

// Basic declaration
let desc = ParameterDescriptor::new("timeout", ParameterType::Double);
node.declare_parameter("timeout", ParameterValue::Double(5.0), desc)?;

// With range constraint
let mut desc = ParameterDescriptor::new("speed", ParameterType::Integer);
desc.integer_range = Some(IntegerRange {
    from_value: 0,
    to_value: 100,
    step: 1,
});
node.declare_parameter("speed", ParameterValue::Integer(50), desc)?;

// Read-only parameter
let mut desc = ParameterDescriptor::new("version", ParameterType::String);
desc.read_only = true;
node.declare_parameter("version", ParameterValue::String("1.0".into()), desc)?;
```

## Getting and Setting

```rust,ignore
// Get returns Option<ParameterValue>
let value = node.get_parameter("timeout"); // Some(Double(5.0))
let missing = node.get_parameter("nonexistent"); // None

// Set returns Result<(), String>
node.set_parameter(Parameter::new("timeout", ParameterValue::Double(10.0)))?;

// Type mismatches are rejected
let err = node.set_parameter(Parameter::new("timeout", ParameterValue::Bool(true)));
assert!(err.is_err());

// Undeclare removes the parameter
node.undeclare_parameter("timeout")?;
```

## Validation Callbacks

Register a callback to accept or reject parameter changes before they take effect:

```rust,ignore
{{#include ../../../crates/ros-z/examples/z_parameters.rs:callback_snippet}}
```

The callback receives all parameters being changed in a single batch. Return `SetParametersResult::success()` to accept or `SetParametersResult::failure("reason")` to reject the entire batch.

## Parameter Services

Each node with parameters enabled exposes 6 standard services:

| Service | Purpose | CLI equivalent |
|---------|---------|---------------|
| `/<node>/get_parameters` | Read parameter values | `ros2 param get` |
| `/<node>/set_parameters` | Update parameter values | `ros2 param set` |
| `/<node>/list_parameters` | List declared parameter names | `ros2 param list` |
| `/<node>/describe_parameters` | Get parameter descriptors | `ros2 param describe` |
| `/<node>/get_parameter_types` | Get type IDs for parameters | — |
| `/<node>/set_parameters_atomically` | All-or-nothing batch update | — |

## Loading from YAML

ros-z supports the standard ROS 2 parameter YAML format:

```yaml
/**:
  ros__parameters:
    global_timeout: 5.0
    debug: true

/my_node:
  ros__parameters:
    sensor_rate: 100
    device_name: "lidar_front"
```

- `/**` applies to all nodes (wildcard)
- `/my_node` applies only to that exact node
- Node-specific values override wildcard values

Load via the builder:

```rust,ignore
let node = ctx.create_node("my_node")
    .with_parameter_file(Path::new("params.yaml"))?
    .build()?;
```

Parameters from the file become overrides — they replace the default value when `declare_parameter` is called.

## Node Builder Options

| Method | Effect |
|--------|--------|
| `.without_parameters()` | Disable parameter services entirely |
| `.with_parameter_overrides(map)` | Set overrides from a `HashMap<String, ParameterValue>` |
| `.with_parameter_file(path)` | Load overrides from a YAML file |

If both a file and programmatic overrides are used, the last call wins.

## /parameter_events

Every successful parameter change publishes a `ParameterEvent` message to `/<node>/parameter_events` with QoS:

- **Reliability**: Reliable
- **Durability**: Transient Local
- **History**: Keep Last (1000)

This matches the ROS 2 default for parameter events. Tools like `ros2 param` and `rqt_reconfigure` subscribe to this topic.

## ROS 2 Comparison

| Operation | rclcpp (C++) | ros-z (Rust) |
|-----------|-------------|--------------|
| Declare | `node->declare_parameter<T>("name", default)` | `node.declare_parameter("name", value, desc)` |
| Get | `node->get_parameter<T>("name")` | `node.get_parameter("name")` → `Option<ParameterValue>` |
| Set | `node->set_parameter(Parameter("name", v))` | `node.set_parameter(Parameter::new("name", v))` → `Result<(), String>` |
| Describe | `node->describe_parameter("name")` | `node.describe_parameter("name")` |
| Callback | `add_on_set_parameters_callback(cb)` | `node.on_set_parameters(cb)` |
| YAML load | `--ros-args --params-file file.yaml` | `.with_parameter_file(path)` |
| Overrides | `--ros-args -p name:=value` | `.with_parameter_overrides(map)` |
| Disable | not possible | `.without_parameters()` |
| Range | `FloatingPointRange` / `IntegerRange` | same types in `ParameterDescriptor` |
| CLI tools | `ros2 param list/get/set/dump` | same (interop via standard services) |

**Key differences:**

- **Error handling**: ros-z returns `Result<(), String>` on set failures; rclcpp throws exceptions
- **Callbacks**: ros-z callbacks receive `&[Parameter]` (slice) and return `SetParametersResult`; rclcpp receives `std::vector<rclcpp::Parameter>`
- **Opt-out**: ros-z can disable parameter services with `.without_parameters()`; rclcpp always enables them
- **No `declare_parameter_if_not_declared`**: check `node.get_parameter("name").is_some()` first

## ROS 2 Interoperability

ros-z parameter services use the same CDR wire format and RIHS01 type hashes as rclcpp. When connected to the same Zenoh router, `ros2 param` commands work against ros-z nodes:

```bash
# List parameters on a ros-z node
ros2 param list /my_node

# Get a parameter value
ros2 param get /my_node max_speed

# Set a parameter value
ros2 param set /my_node max_speed 2.5

# Dump all parameters to YAML
ros2 param dump /my_node
```

```admonish warning
ROS 2 nodes must use `rmw_zenoh_cpp` (`export RMW_IMPLEMENTATION=rmw_zenoh_cpp`) and connect to the same Zenoh router.
```

## Full Example

```rust,ignore
{{#include ../../../crates/ros-z/examples/z_parameters.rs:full_example}}
```

**Running the example:**

```bash
# Start a Zenoh router first
cargo run --example zenoh_router

# Run all parameter demos
cargo run --example z_parameters

# Run a specific demo
cargo run --example z_parameters -- --mode declare
cargo run --example z_parameters -- --mode callback
cargo run --example z_parameters -- --mode yaml
```

## Resources

- **[Feature Flags](./feature_flags.md)** — `rcl_interfaces` feature for parameter service client types
- **[Services](./services.md)** — underlying service mechanism
- **[Quick Start](./quick_start.md)** — getting started with ros-z
