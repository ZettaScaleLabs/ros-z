# Lifecycle Nodes

**Lifecycle nodes give you explicit control over when a node starts communicating, making bringup and shutdown of complex systems safe and predictable.**

```admonish tip
Use lifecycle nodes whenever a node needs external resources (hardware drivers, network connections, configuration files) that must be ready before it starts publishing. They are the standard ROS 2 pattern for production system orchestration.
```

## State Machine

A lifecycle node moves through a defined set of states. Only in the **Active** state does a node send or receive messages.

```mermaid
stateDiagram-v2
    [*] --> Unconfigured: create
    Unconfigured --> Inactive: configure ✓
    Unconfigured --> Finalized: shutdown
    Inactive --> Active: activate ✓
    Inactive --> Unconfigured: cleanup ✓
    Inactive --> Finalized: shutdown
    Active --> Inactive: deactivate ✓
    Active --> Finalized: shutdown
    Finalized --> [*]
    Unconfigured --> Unconfigured: configure ✗ (failure)
    note right of Active: Publishers deliver messages
    note right of Inactive: Publishers drop messages silently
```

### Primary States

| State | ID | Description |
|-------|----|-------------|
| **Unconfigured** | 1 | Initial state. Node exists but has not loaded configuration. |
| **Inactive** | 2 | Configured but not communicating. Publishers drop messages. |
| **Active** | 3 | Fully operational. Publishers deliver messages. |
| **Finalized** | 4 | Terminal state. Node is destroyed. |

### Transitions

| Method | From | To (success) |
|--------|------|--------------|
| `configure()` | Unconfigured | Inactive |
| `activate()` | Inactive | Active |
| `deactivate()` | Active | Inactive |
| `cleanup()` | Inactive | Unconfigured |
| `shutdown()` | Any primary | Finalized |

## Creating a Lifecycle Node

```rust,ignore
use ros_z::lifecycle::{ZLifecycleNode, CallbackReturn, LifecycleState};
use ros_z::prelude::*;

# fn main() -> zenoh::Result<()> {
let ctx = ZContextBuilder::default().build()?;
let mut node = ctx.create_lifecycle_node("my_node").build()?;
# Ok(())
# }
```

### With a Namespace

```rust,ignore
use ros_z::prelude::*;

# fn main() -> zenoh::Result<()> {
let ctx = ZContextBuilder::default().build()?;
let mut node = ctx
    .create_lifecycle_node("my_node")
    .with_namespace("/robot")
    .build()?;
# Ok(())
# }
```

## Lifecycle Callbacks

Set callbacks on the node before triggering any transitions. Each callback receives the previous state and returns a `CallbackReturn`.

```rust,ignore
use ros_z::lifecycle::{ZLifecycleNode, CallbackReturn};
use ros_z::prelude::*;

# fn main() -> zenoh::Result<()> {
# let ctx = ZContextBuilder::default().build()?;
let mut node = ctx.create_lifecycle_node("talker").build()?;

node.on_configure = Box::new(|_prev| {
    // Load parameters, open files, connect to hardware
    println!("configuring");
    CallbackReturn::Success
});

node.on_activate = Box::new(|_prev| {
    // Start timers, enable hardware
    println!("activating");
    CallbackReturn::Success
});

node.on_deactivate = Box::new(|_prev| {
    // Pause timers, disable hardware outputs
    println!("deactivating");
    CallbackReturn::Success
});

node.on_cleanup = Box::new(|_prev| {
    // Release resources acquired in on_configure
    println!("cleaning up");
    CallbackReturn::Success
});

node.on_shutdown = Box::new(|_prev| {
    CallbackReturn::Success
});
# Ok(())
# }
```

### Callback Return Values

| Value | Effect |
|-------|--------|
| `CallbackReturn::Success` | Transition completes normally |
| `CallbackReturn::Failure` | Transition rolls back; state unchanged |
| `CallbackReturn::Error` | `on_error` callback is called |

The default `on_error` returns `Failure`, which drives the node to **Finalized**. Override it to recover to **Unconfigured** instead:

```rust,ignore
# use ros_z::lifecycle::{CallbackReturn};
# use ros_z::prelude::*;
# fn main() -> zenoh::Result<()> {
# let ctx = ZContextBuilder::default().build()?;
# let mut node = ctx.create_lifecycle_node("n").build()?;
// Recover to Unconfigured instead of crashing to Finalized
node.on_error = Box::new(|_prev| CallbackReturn::Success);
# Ok(())
# }
```

## Lifecycle Publishers

A lifecycle publisher wraps a regular publisher and **silently drops** publish calls while the node is not Active. This makes publishing code clean — no manual state checks needed.

```rust,ignore
use ros_z::lifecycle::ZLifecycleNode;
use ros_z::prelude::*;
use ros_z_msgs::ros::std_msgs::String as RosString;

# fn main() -> zenoh::Result<()> {
# let ctx = ZContextBuilder::default().build()?;
# let mut node = ctx.create_lifecycle_node("talker").build()?;
// Create the publisher — it starts deactivated
let pub_ = node.create_publisher::<RosString>("chatter")?;

node.configure()?;
node.activate()?;
// Publisher is now active — messages are delivered
pub_.publish(&RosString { data: "hello".to_string() })?;

node.deactivate()?;
// Publisher is deactivated — publish() returns Ok(()) but drops the message
pub_.publish(&RosString { data: "dropped".to_string() })?;
# Ok(())
# }
```

All publishers created with `create_publisher` are registered as **managed entities**: they activate and deactivate automatically when the node transitions.

```admonish note
Publishers created **after** `activate()` are immediately activated. You can safely create publishers at any point in the node's lifetime.
```

## Triggering Transitions

Transitions can be triggered programmatically or via the lifecycle management services.

### Programmatic

```rust,ignore
# use ros_z::prelude::*;
# fn main() -> zenoh::Result<()> {
# let ctx = ZContextBuilder::default().build()?;
# let mut node = ctx.create_lifecycle_node("n").build()?;
let state = node.configure()?;   // → Inactive
let state = node.activate()?;    // → Active
let state = node.deactivate()?;  // → Inactive
let state = node.cleanup()?;     // → Unconfigured
let state = node.shutdown()?;    // → Finalized
# Ok(())
# }
```

### Via ROS 2 CLI

Every lifecycle node automatically exposes five management services under its node name:

```bash
# Inspect current state
ros2 lifecycle get /my_node

# List available transitions
ros2 lifecycle list /my_node

# Trigger a transition
ros2 lifecycle set /my_node configure
ros2 lifecycle set /my_node activate
```

### Services Exposed

| Service | Description |
|---------|-------------|
| `~/change_state` | Trigger a transition by ID or label |
| `~/get_state` | Query the current state |
| `~/get_available_states` | List all states in the machine |
| `~/get_available_transitions` | List transitions valid from the current state |
| `~/get_transition_graph` | List all transitions in the full graph |

The `~/transition_event` topic publishes a `TransitionEvent` message on every state change, enabling external monitoring.

## Full Example: Managed Talker

```rust,ignore
{{#include ../../../crates/ros-z/examples/lifecycle_talker.rs:full_example}}
```
