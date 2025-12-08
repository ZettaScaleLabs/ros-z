# ROS-Z Action Examples

This directory contains examples demonstrating the ROS 2 Action pattern in ROS-Z. Actions are used for long-running tasks that can provide feedback during execution and can be canceled.

## Examples Overview

### 1. **fibonacci.rs** - Basic Action Pattern

The classic Fibonacci example that computes the Fibonacci sequence up to a given order.

**Demonstrates:**

- Basic action server and client setup
- Publishing feedback during execution
- Handling goal cancellation
- Streaming feedback to the client

**Run server:**

```bash
cargo run --example action_fibonacci server
```

**Run client (in another terminal):**

```bash
cargo run --example action_fibonacci client 10
```

The client will send a goal to compute Fibonacci up to order 10, display feedback as it's received, and print the final result.

---

### 2. **navigation.rs** - Simulated Robot Navigation

Simulates a robot navigating to a target pose (x, y coordinates).

**Demonstrates:**

- Continuous feedback with position updates
- Distance calculation and progress tracking
- Real-world robotics use case

**Run server:**

```bash
cargo run --example action_navigation server
```

**Run client:**

```bash
cargo run --example action_navigation client 5.0 3.0
```

This sends the robot to position (5.0, 3.0), showing progress along the way.

---

### 3. **cancelable_task.rs** - Task Cancellation

Demonstrates a long-running counting task that can be canceled mid-execution.

**Demonstrates:**

- Proper cancellation handling
- Checking `is_cancel_requested()` in the server loop
- Graceful shutdown with partial results

**Run server:**

```bash
cargo run --example action_cancelable_task server
```

**Run client:**

```bash
# Start counting to 20
cargo run --example action_cancelable_task client 20

# In another terminal, send a cancel request:
cargo run --example action_cancelable_task cancel
```

---

### 4. **concurrent_goals.rs** - Multiple Simultaneous Goals

Shows how an action server can handle multiple goals concurrently.

**Demonstrates:**

- Concurrent goal execution with `with_handler()`
- Multiple clients sending goals simultaneously
- Server managing multiple executing goals

**Run server:**

```bash
cargo run --example action_concurrent_goals server
```

**Run multiple clients:**

```bash
# Terminal 1
cargo run --example action_concurrent_goals client task_1 10

# Terminal 2
cargo run --example action_concurrent_goals client task_2 15

# Terminal 3
cargo run --example action_concurrent_goals client task_3 5
```

All goals execute concurrently, each with their own feedback stream.

---

### 5. **manual_control.rs** - Low-Level Action Control

Demonstrates manual control over the action lifecycle without using `with_handler()`.

**Demonstrates:**

- Direct use of `recv_goal()`, `accept()`, `execute()`
- Manual goal state transitions
- Fine-grained control over execution
- Publishing status updates manually

**Run server:**

```bash
cargo run --example action_manual_control server
```

**Run client:**

```bash
cargo run --example action_manual_control client compute 42
```

---

## Key Concepts

### Action Components

1. **Goal**: The request sent by the client (e.g., target position, computation parameters)
2. **Result**: The final outcome returned when the action completes
3. **Feedback**: Periodic updates sent during execution (e.g., progress, current state)

### Action States

- **Accepted**: Goal has been received and accepted by the server
- **Executing**: Goal is actively being processed
- **Succeeded**: Goal completed successfully
- **Canceled**: Goal was canceled before completion
- **Aborted**: Goal failed or was aborted

### Defining an Action

```rust
use ros_z::action::ZAction;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MyGoal {
    pub target: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MyResult {
    pub final_value: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MyFeedback {
    pub current_value: i32,
}

pub struct MyAction;

impl ZAction for MyAction {
    type Goal = MyGoal;
    type Result = MyResult;
    type Feedback = MyFeedback;

    fn name() -> &'static str {
        "my_action"
    }
}
```

### Creating an Action Server

```rust
let server = node
    .create_action_server::<MyAction>("my_action")
    .build()?
    .with_handler(|executing| async move {
        // Process the goal
        let goal = executing.goal;

        // Publish feedback
        executing.publish_feedback(MyFeedback { current_value: 50 }).unwrap();

        // Complete successfully
        executing.succeed(MyResult { final_value: 100 }).unwrap();
    });
```

### Creating an Action Client

```rust
let client = node
    .create_action_client::<MyAction>("my_action")
    .build()?;

// Send a goal
let mut goal_handle = client.send_goal(MyGoal { target: 100 }).await?;

// Get feedback stream
if let Some(mut feedback_stream) = goal_handle.feedback_stream() {
    tokio::spawn(async move {
        while let Some(feedback) = feedback_stream.recv().await {
            println!("Progress: {}", feedback.current_value);
        }
    });
}

// Wait for result
let result = goal_handle.result().await?;
println!("Final: {}", result.final_value);
```

## Testing the Examples

Build all action examples:

```bash
cargo build --examples
```

List all action examples:

```bash
cargo build --examples 2>&1 | grep action_
```

Run with verbose logging:

```bash
RUST_LOG=debug cargo run --example action_fibonacci server
```

## Common Patterns

### Cancellation Handling

```rust
for i in 0..100 {
    if executing.is_cancel_requested() {
        executing.canceled(MyResult { final_value: i }).unwrap();
        return;
    }
    // Do work...
}
```

### Periodic Feedback

```rust
for i in 0..100 {
    executing.publish_feedback(MyFeedback { current_value: i }).unwrap();
    tokio::time::sleep(Duration::from_millis(100)).await;
}
```

### Status Monitoring

```rust
let mut status_watch = client.status_watch(goal_handle.id())?;
while let Ok(_) = status_watch.changed().await {
    let status = *status_watch.borrow();
    println!("Status: {:?}", status);
}
```

## Troubleshooting

### Server not receiving goals

- Ensure the server is running before the client sends goals
- Check that action names match between client and server
- Allow time for Zenoh discovery (~500ms)

### Feedback not received

- Ensure `feedback_stream()` is called before awaiting `result()`
- Check that feedback is published before the goal completes

### Cancellation not working

- Server must check `is_cancel_requested()` periodically
- Client must call `cancel()` or `goal_handle.cancel()` on the goal handle

## Further Reading

- [ROS 2 Actions Design](https://design.ros2.org/articles/actions.html)
- [ROS 2 Action Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- ROS-Z Action API documentation (run `cargo doc --open`)
