# Actions

!!! note "Go users"
    The code examples in this chapter are **Rust**. The Go action API uses separate goal-acceptance and execute callbacks instead of the `with_handler(ExecutingGoal)` pattern shown here. For Go action patterns, see the [Go Bindings](../bindings/go.md) chapter.

**Actions enable long-running tasks with progress feedback and cancellation support, perfect for operations that take seconds or minutes to complete.** Unlike services that return immediately, actions provide streaming feedback while executing complex workflows.

!!! tip
    Use actions for robot navigation, trajectory execution, or any operation where you need progress updates and the ability to cancel mid-execution. Use services for quick request-response operations.

## What is an Action?

<iframe src="../action-lifecycle.html" style="width:100%;height:320px;border:none;border-radius:8px;" title="Action lifecycle animation"></iframe>

**A long-running task with three channels: goal → feedback stream → result. Plus cancellation.**

- **Goal** — what the client wants done (`float64 target_x, target_y`)
- **Feedback** — progress while executing (`float64 percent_complete`)
- **Result** — final outcome when done (`float64 elapsed_seconds`)
- **Cancel** — client can abort at any time; server decides how to honor it

### When to use actions

| Operation | Duration | Use |
|-----------|----------|-----|
| Drive to coordinates | 5–60 s | **Action** |
| Execute trajectory | 1–30 s | **Action** |
| Compute inverse kinematics | < 100 ms | Service |
| Publish odometry | Continuous | Topic |

### The .action format

```text
# Goal
float64 target_x
float64 target_y
---
# Result
float64 elapsed_seconds
string  status_message
---
# Feedback
float64 percent_complete
float64 current_x
float64 current_y
```

Three sections separated by `---`: goal, result, feedback.

### Goal states

```mermaid
stateDiagram-v2
    [*] --> Pending : SendGoal
    Pending --> Accepted : server accepts
    Pending --> Rejected : server rejects
    Accepted --> Executing : execution starts
    Executing --> Succeeded : task complete
    Executing --> Aborted : server error
    Executing --> Canceling : cancel request received
    Canceling --> Canceled : server honors cancel
```

### Cancellation contract

Sending a cancel does **not** immediately stop the action:

1. Client sends cancel request
2. Server receives it — decides how/when to stop
3. Server sends `Result(status=Canceled)` to close the goal
4. Until then, the goal remains in `Executing` state

### ros-z type-state API

ros-z uses Rust's type system to enforce the action protocol at compile time:

```text
RequestedGoal  →  AcceptedGoal  →  ExecutingGoal
     ↓                                   ↓
  (reject)                          publish_feedback()
                                    return Result
```

The compiler prevents calling `publish_feedback` before `accept()`. Invalid state transitions are caught at compile time, not at runtime.

### Key Concepts at a Glance

<div class="flashcard-grid">
  <div class="flashcard">
    <div class="flashcard-inner">
      <div class="flashcard-front">
        <div class="flashcard-tag">Pattern</div>
        <div class="flashcard-term">What are the three action channels?</div>
        <div class="flashcard-hint">Click to flip</div>
      </div>
      <div class="flashcard-back">
        <div>• <strong>Goal</strong>: what the client wants done.</div>
        <div>• <strong>Feedback</strong>: progress updates during execution.</div>
        <div>• <strong>Result</strong>: final outcome when done (or cancelled).</div>
      </div>
    </div>
  </div>
  <div class="flashcard">
    <div class="flashcard-inner">
      <div class="flashcard-front">
        <div class="flashcard-tag">States</div>
        <div class="flashcard-term">What are the four terminal states of a goal?</div>
        <div class="flashcard-hint">Click to flip</div>
      </div>
      <div class="flashcard-back">
        <div>• <strong>Succeeded</strong>: completed normally.</div>
        <div>• <strong>Canceled</strong>: client requested cancel, server honored it.</div>
        <div>• <strong>Aborted</strong>: server encountered an error.</div>
        <div>• <strong>Rejected</strong>: server refused the goal before starting.</div>
      </div>
    </div>
  </div>
  <div class="flashcard">
    <div class="flashcard-inner">
      <div class="flashcard-front">
        <div class="flashcard-tag">Cancellation</div>
        <div class="flashcard-term">Does a cancel request immediately stop the action?</div>
        <div class="flashcard-hint">Click to flip</div>
      </div>
      <div class="flashcard-back">
        No. The server receives the cancel signal and decides how to handle it. It must still send a result to close the goal. Incomplete cancellation leaves the goal in Executing state.
      </div>
    </div>
  </div>
  <div class="flashcard">
    <div class="flashcard-inner">
      <div class="flashcard-front">
        <div class="flashcard-tag">vs Service</div>
        <div class="flashcard-term">Why not use a service for navigation?</div>
        <div class="flashcard-hint">Click to flip</div>
      </div>
      <div class="flashcard-back">
        Services block until done. A 30-second navigation blocks the client for 30 seconds with no updates and no way to cancel. Actions keep the client free and provide progress.
      </div>
    </div>
  </div>
  <div class="flashcard">
    <div class="flashcard-inner">
      <div class="flashcard-front">
        <div class="flashcard-tag">ros-z API</div>
        <div class="flashcard-term">What is the type-state pattern in ros-z actions?</div>
        <div class="flashcard-hint">Click to flip</div>
      </div>
      <div class="flashcard-back">
        <strong>RequestedGoal → AcceptedGoal → ExecutingGoal</strong>
        Each type carries only the methods valid at that stage. The compiler prevents calling <strong>publish_feedback</strong> before accepting the goal.
      </div>
    </div>
  </div>
  <div class="flashcard">
    <div class="flashcard-inner">
      <div class="flashcard-front">
        <div class="flashcard-tag">Feedback</div>
        <div class="flashcard-term">Is feedback required during action execution?</div>
        <div class="flashcard-hint">Click to flip</div>
      </div>
      <div class="flashcard-back">
        No. Feedback is optional — only send it when the client benefits from progress updates. Short actions (under a second) often skip feedback entirely.
      </div>
    </div>
  </div>
</div>

## Action Lifecycle

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> Accepted: Send Goal
    Accepted --> Executing: Start Processing
    Executing --> Executing: Send Feedback
    Executing --> Succeeded: Complete
    Executing --> Canceled: Cancel Request
    Executing --> Aborted: Error Occurs
    Succeeded --> [*]
    Canceled --> [*]
    Aborted --> [*]
```

## Components

| Component | Type | Purpose |
|-----------|------|---------|
| **Goal** | Input | Defines the desired outcome |
| **Feedback** | Stream | Progress updates during execution |
| **Result** | Output | Final outcome when complete |
| **Status** | State | Current execution state |

## Communication Pattern

```mermaid
sequenceDiagram
    participant C as Client
    participant S as Server

    C->>S: Send Goal
    S->>C: Goal Accepted
    loop During Execution
        S->>C: Feedback Update
    end
    alt Success
        S->>C: Result (Success)
    else Canceled
        C->>S: Cancel Request
        S->>C: Result (Canceled)
    else Error
        S->>C: Result (Aborted)
    end
```

### Execution Timeline

```mermaid
sequenceDiagram
    participant C as Action Client
    participant S as Action Server

    C->>S: SendGoal(target_x=3.0, target_y=4.0)
    S-->>C: GoalResponse(accepted=true, goal_id=abc123)

    loop Every 500ms during execution
        S-->>C: Feedback(percent_complete=25%)
        S-->>C: Feedback(percent_complete=50%)
        S-->>C: Feedback(percent_complete=75%)
    end

    alt Normal completion
        S-->>C: Result(status=Succeeded, elapsed=12.3s)
    else Client cancels
        C->>S: CancelGoal(goal_id=abc123)
        S-->>C: CancelResponse(accepted=true)
        S-->>C: Result(status=Canceled)
    else Server error
        S-->>C: Result(status=Aborted, message="obstacle detected")
    end
```

## Use Cases

**Robot Navigation:**

- Goal: Target position and orientation
- Feedback: Current position, distance remaining, obstacles detected
- Result: Final position, success/failure reason

**Gripper Control:**

- Goal: Desired grip force and position
- Feedback: Current force, contact detection
- Result: Grip achieved, object secured

**Long Computations:**

- Goal: Computation parameters
- Feedback: Progress percentage, intermediate results
- Result: Final computed value, execution time

!!! info
    Actions excel when operations take more than a few seconds and users need visibility into progress. For sub-second operations, prefer services for simplicity.

## Minimal Pattern

Before reading the full examples, here is the skeleton every action client and server follows:

**Client:**

```rust
use ros_z::Builder;
use ros_z::context::ZContextBuilder;

let ctx = ZContextBuilder::default().build()?;
let node = ctx.create_node("my_client").build()?;

// Create the action client
let client = node
    .create_action_client::<MyAction>("my_action_name")
    .build()?;

// 1. Send a goal — returns a GoalHandle
let mut goal_handle = client.send_goal(MyAction::Goal { target: 10 }).await?;

// 2. Stream feedback while the action runs
if let Some(mut feedback_rx) = goal_handle.feedback() {
    tokio::spawn(async move {
        while let Some(fb) = feedback_rx.recv().await {
            println!("Progress: {:?}", fb);
        }
    });
}

// 3. Wait for the final result (blocks until terminal state)
let result = goal_handle.result().await?;
println!("Done: {:?}", result);
```

**Server:**

```rust
use ros_z::Builder;
use ros_z::action::server::ExecutingGoal;
use ros_z::context::ZContextBuilder;

let ctx = ZContextBuilder::default().build()?;
let node = ctx.create_node("my_server").build()?;

// Create the action server (keep _server alive for the duration)
let _server = node
    .create_action_server::<MyAction>("my_action_name")
    .build()?
    .with_handler(|executing: ExecutingGoal<MyAction>| async move {
        // Report progress (synchronous — no .await)
        executing.publish_feedback(MyAction::Feedback { progress: 50 }).expect("feedback failed");

        // Check for cancellation
        if executing.is_cancel_requested() {
            executing.canceled(MyAction::Result { value: 0 }).unwrap();
            return;
        }

        executing.succeed(MyAction::Result { value: 42 }).unwrap();
    });
```

The key constraint: **one `GoalHandle` per goal**. `feedback()` and `status_watch()` each return `Some` only on the first call — after that, the handle has moved out the receiver.

## Action Server Example

This example demonstrates an action server that computes Fibonacci sequences. The server accepts goals, publishes periodic feedback with partial results, and supports cancellation.

```rust
/// Fibonacci action server node that computes Fibonacci sequences
///
/// # Arguments
/// * `ctx` - The ros-z context
/// * `timeout` - Optional timeout duration. If None, runs until ctrl+c.
pub async fn run_fibonacci_action_server(ctx: ZContext, timeout: Option<Duration>) -> Result<()> {
    // Create a node named "fibonacci_action_server"
    let node = ctx.create_node("fibonacci_action_server").build()?;

    // Create an action server
    // Note: The server variable must be kept alive for the duration of the function
    // to ensure the action server and its background tasks remain active
    let _server = node
        .create_action_server::<Fibonacci>("fibonacci")
        .build()?
        .with_handler(|executing: ExecutingGoal<Fibonacci>| async move {
            let order = executing.goal.order;
            let mut sequence = vec![0, 1];

            println!("Executing Fibonacci goal with order {}", order);

            let mut canceled = false;
            let mut cancel_sequence = None;

            for i in 2..=order {
                // Check for cancellation
                if executing.is_cancel_requested() {
                    println!("Goal canceled!");
                    canceled = true;
                    cancel_sequence = Some(sequence.clone());
                    break;
                }

                let next = sequence[i as usize - 1] + sequence[i as usize - 2];
                sequence.push(next);

                // Publish feedback
                // Distro-specific feedback field names
                #[cfg(feature = "kilted")]
                let feedback = FibonacciFeedback {
                    sequence: sequence.clone(),
                };
                #[cfg(not(feature = "kilted"))]
                let feedback = FibonacciFeedback {
                    partial_sequence: sequence.clone(),
                };
                executing
                    .publish_feedback(feedback)
                    .expect("Failed to publish feedback");

                tokio::time::sleep(Duration::from_millis(500)).await;
            }

            if canceled {
                executing
                    .canceled(FibonacciResult {
                        sequence: cancel_sequence.unwrap(),
                    })
                    .unwrap();
            } else {
                println!("Goal succeeded!");
                executing.succeed(FibonacciResult { sequence }).unwrap();
            }
        });

    println!("Fibonacci action server started");

    if let Some(timeout) = timeout {
        // For testing: run for the specified timeout
        tokio::time::sleep(timeout).await;
    } else {
        tokio::signal::ctrl_c().await?;
    }

    Ok(())
}
```

**Key points:**

- **Handler Pattern**: Uses `.with_handler()` to define asynchronous goal execution
- **Feedback Publishing**: Sends partial results periodically via `publish_feedback()`
- **Cancellation Support**: Checks `is_cancel_requested()` and handles graceful cancellation
- **Completion**: Uses `.succeed()` or `.canceled()` to send final result

**Running the server:**

```bash
# Start Eclipse Zenoh router first
cargo run --example zenoh_router

# Run the server (runs until Ctrl+C)
cargo run --example demo_nodes_fibonacci_action_server
```

## Action Client Example

This example demonstrates an action client that sends goals and monitors execution progress with feedback updates.

```rust
/// Fibonacci action client node that sends goals to compute Fibonacci sequences
///
/// # Arguments
/// * `ctx` - The ros-z context
/// * `order` - The order of the Fibonacci sequence to compute
pub async fn run_fibonacci_action_client(ctx: ZContext, order: i32) -> Result<Vec<i32>> {
    // Create a node named "fibonacci_action_client"
    let node = ctx.create_node("fibonacci_action_client").build()?;

    // Create an action client
    let client = node
        .create_action_client::<Fibonacci>("fibonacci")
        .build()?;

    // Wait a bit for the server to be discovered
    tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;

    println!(
        "Fibonacci action client started, sending goal with order {}",
        order
    );

    // Send the goal
    let mut goal_handle = client.send_goal(FibonacciGoal { order }).await?;
    println!("Goal sent and accepted!");

    // Set up feedback monitoring
    if let Some(mut feedback_stream) = goal_handle.feedback() {
        tokio::spawn(async move {
            while let Some(fb) = feedback_stream.recv().await {
                // Distro-specific feedback field names
                #[cfg(feature = "kilted")]
                println!("Feedback: {:?}", fb.sequence);
                #[cfg(not(feature = "kilted"))]
                println!("Feedback: {:?}", fb.partial_sequence);
            }
        });
    }

    // Wait for the result with timeout
    println!("Waiting for result (timeout: 10s)...");
    let result = match tokio::time::timeout(
        tokio::time::Duration::from_secs(10),
        goal_handle.result(),
    )
    .await
    {
        Ok(Ok(result)) => {
            println!("Final result: {:?}", result.sequence);
            result
        }
        Ok(Err(e)) => {
            eprintln!("Action failed: {}", e);
            return Err(e);
        }
        Err(_) => {
            eprintln!("Timeout waiting for action result");
            return Err(zenoh::Error::from("Timeout waiting for action result"));
        }
    };

    Ok(result.sequence)
}
```

**Key points:**

- **Goal Sending**: Uses `send_goal()` to submit goals and get a handle
- **Feedback Monitoring**: Spawns async task to receive and display feedback
- **Result Handling**: Waits for completion with timeout and error handling
- **Type Safety**: Strongly-typed goal, feedback, and result messages

**Running the client:**

```bash
# Basic usage - compute Fibonacci(10)
cargo run --example demo_nodes_fibonacci_action_client

# Compute Fibonacci(15)
cargo run --example demo_nodes_fibonacci_action_client -- --order 15

# Connect to specific router
cargo run --example demo_nodes_fibonacci_action_client -- --endpoint tcp/localhost:7447
```

## Complete Action Workflow

**Terminal 1 - Start Zenoh Router:**

```bash
cargo run --example zenoh_router
```

**Terminal 2 - Start Action Server:**

```bash
cargo run --example demo_nodes_fibonacci_action_server
```

**Terminal 3 - Send Goals from Client:**

```bash
cargo run --example demo_nodes_fibonacci_action_client -- --order 10
```

You'll see:

- **Client**: Goal sent, feedback updates with partial sequences, final result
- **Server**: Goal received, executing with feedback, completion status

!!! warning
    Always implement timeout mechanisms for action clients. Long-running actions can fail or hang, and clients need graceful degradation strategies.

## Canceling an Action from the Client

The `GoalHandle` returned by `send_goal()` has a `cancel()` method for requesting cancellation:

```rust
let goal_handle = client.send_goal(goal).await?;

// ... later, if you need to stop the action:
let cancel_response = goal_handle.cancel().await?;
println!("Cancel accepted: {}", cancel_response.goals_canceling.len() > 0);
```

To cancel a specific goal by ID without a handle:

```rust
client.cancel_goal(goal_id).await?;
```

To cancel every in-progress goal at once:

```rust
client.cancel_all_goals().await?;
```

The server checks cancellation with `is_cancel_requested()` and calls `.canceled()` to complete the handshake (see Server section above). There is no guarantee the server will honor the cancellation — it may complete the goal before processing the cancel request.

## GoalHandle API Reference

`send_goal()` returns a `GoalHandle<A>`. All interaction with an in-flight goal goes through this handle:

| Method | Returns | Notes |
|--------|---------|-------|
| `goal_handle.id()` | `GoalId` | Unique identifier for this goal |
| `goal_handle.feedback()` | `Option<UnboundedReceiver<A::Feedback>>` | First call only — receiver is moved out |
| `goal_handle.status_watch()` | `Option<watch::Receiver<GoalStatus>>` | First call only — watcher is moved out |
| `goal_handle.cancel()` | `Result<CancelGoalServiceResponse>` | Requests server cancellation |
| `goal_handle.result()` | `Result<A::Result>` | Waits for terminal state, then fetches result |

**Receiving feedback:**

```rust
// Take the feedback receiver — this works exactly once per GoalHandle
if let Some(mut rx) = goal_handle.feedback() {
    while let Some(fb) = rx.recv().await {
        println!("Feedback: {:?}", fb);
    }
    // Loop ends when the server drops its feedback sender (goal completes)
}
```

**Watching status transitions:**

```rust
if let Some(mut status_rx) = goal_handle.status_watch() {
    while status_rx.changed().await.is_ok() {
        println!("Status: {:?}", *status_rx.borrow());
    }
}
```

**Waiting for the result:**

```rust
// Blocks until the goal reaches a terminal state (Succeeded, Canceled, or Aborted)
// then fetches and returns the final result
let result = goal_handle.result().await?;
```

!!! tip
    Spawn `feedback()` in a separate task while awaiting `result()` in the main task. Both can run concurrently — `result()` does not consume feedback messages.

## Defining a Custom Action Type

The examples above use the pre-built `Fibonacci` action from `ros-z-msgs`. To define your own action type, implement the `ZAction` trait:

```rust
use serde::{Deserialize, Serialize};
use ros_z::action::ZAction;

// Define the three message structs
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CountGoal {
    pub target: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CountFeedback {
    pub current: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CountResult {
    pub message: String,
}

// Tie them together with ZAction
pub struct CountAction;

impl ZAction for CountAction {
    type Goal = CountGoal;
    type Feedback = CountFeedback;
    type Result = CountResult;

    fn name() -> &'static str {
        "my_robot/count_to_n"
    }
}
```

**Key points:**

- `Goal`, `Feedback`, and `Result` can be any struct that implements `Serialize + Deserialize + Clone + Send + Sync + 'static` (blanket implementation via `ZMessage`)
- `name()` sets the Zenoh key prefix for the action's internal services and topics
- The default `send_goal_type_info()`, `get_result_type_info()`, etc. all return `TypeHash::zero()`, which is correct for ros-z-to-ros-z communication. For ROS 2 interop, override these with the correct RIHS01 hashes.

!!! tip
    For ros-z-to-ros-z-only actions, the defaults work without any additional configuration. For ROS 2 interop, use schema-generated types from a `.action` file via `ros-z-codegen` — see [Custom Messages](../user-guide/custom-messages.md).

## Comparison with Other Patterns

| Pattern | Duration | Feedback | Cancellation | Use Case |
|---------|----------|----------|--------------|----------|
| **Pub-Sub** | Continuous | No | N/A | Sensor data streaming |
| **Service** | < 1 second | No | No | Quick queries |
| **Action** | Seconds to minutes | Yes | Yes | Long-running tasks |

## Resources

- **[ROS 2 Actions Documentation](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)** - Official ROS 2 action guide
- **[ros-z Examples](https://github.com/ZettaScaleLabs/ros-z/tree/main/crates/ros-z/examples)** - Working action implementations
- **[Services](./services.md)** - Simpler request-response pattern
- **[Custom Messages](../user-guide/custom-messages.md)** - Defining custom action types with `.action` files

**Action implementation is evolving. Check the ros-z repository for the latest examples and API updates.**
