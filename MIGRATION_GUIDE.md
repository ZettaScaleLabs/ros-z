# Migration Guide: ROS 2 Actions to ros-z

This guide helps you migrate ROS 2 action code from the C++ rclcpp/rclpy APIs to the Rust ros-z API.

## Action Definition

### ROS 2 (C++)
```cpp
#include <rclcpp_action/rclcpp_action.hpp>

class MyAction : public rclcpp_action::ServerGoalHandle<MyAction>
{
public:
    using Goal = interfaces::action::MyAction::Goal;
    using Result = interfaces::action::MyAction::Result;
    using Feedback = interfaces::action::MyAction::Feedback;
};
```

### ros-z (Rust)
```rust
use ros_z::action::ZAction;
use ros_z::msg::ZMessage;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MyGoal { pub target: f64 }
impl ZMessage for MyGoal {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MyResult { pub final_value: f64, pub success: bool }
impl ZMessage for MyResult {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MyFeedback { pub current_value: f64, pub progress: f64 }
impl ZMessage for MyFeedback {}

pub struct MyAction;
impl ZAction for MyAction {
    type Goal = MyGoal;
    type Result = MyResult;
    type Feedback = MyFeedback;

    fn name() -> &'static str { "my_action" }
}
```

## Action Client

### ROS 2 (C++)
```cpp
auto client = rclcpp_action::create_client<MyAction>(node, "my_action");
auto goal = MyAction::Goal();
goal.target = 42.0;
auto options = rclcpp_action::Client<MyAction>::SendGoalOptions();
auto future = client->async_send_goal(goal, options);
```

### ros-z (Rust)
```rust
use ros_z::action::*;

let client = node.create_action_client::<MyAction>("my_action").build()?;
let goal_handle = client.send_goal(MyGoal { target: 42.0 }).await?;
```

## Action Server

### ROS 2 (C++)
```cpp
auto server = rclcpp_action::create_server<MyAction>(
    node,
    "my_action",
    handle_goal,
    handle_cancel,
    handle_accepted
);
```

### ros-z (Rust)
```rust
use ros_z::action::*;

let server = node.create_action_server::<MyAction>("my_action")
    .with_goal_callback(|goal| async move {
        // Process goal
        Ok(AcceptGoal::accept())
    })
    .with_cancel_callback(|goal_handle| async move {
        // Handle cancellation
        goal_handle.set_cancelled(MyResult { ... });
    })
    .build()?;
```

## Goal Handling

### ROS 2 (C++)
```cpp
void handle_accepted(const std::shared_ptr<ServerGoalHandle> goal_handle)
{
    auto result = std::make_shared<MyAction::Result>();
    // Execute goal...
    goal_handle->succeed(result);
}
```

### ros-z (Rust)
```rust
.with_execute_callback(|mut goal_handle| async move {
    // Execute the goal
    let result = MyResult { final_value: 42.0, success: true };
    goal_handle.set_succeeded(result);
})
```

## Feedback Publishing

### ROS 2 (C++)
```cpp
auto feedback = std::make_shared<MyAction::Feedback>();
feedback->current_value = current;
goal_handle->publish_feedback(feedback);
```

### ros-z (Rust)
```rust
goal_handle.publish_feedback(MyFeedback {
    current_value: current,
    progress: current / target,
})?;
```

## Common Patterns

### Monitoring Goal Status

**ROS 2:**
```cpp
auto goal_handle_future = client->async_send_goal(goal);
auto goal_handle = goal_handle_future.get();
while (rclcpp::ok() && !goal_handle->is_done()) {
    // Check status
}
```

**ros-z:**
```rust
let goal_handle = client.send_goal(goal).await?;
let mut status_rx = goal_handle.status_watch().unwrap();
while let Ok(()) = status_rx.changed().await {
    let status = *status_rx.borrow();
    if status.is_terminal() {
        break;
    }
}
```

### Receiving Feedback

**ROS 2:**
```cpp
options.feedback_callback = [](const std::shared_ptr<const Feedback>) {
    // Handle feedback
};
```

**ros-z:**
```rust
let mut feedback_rx = goal_handle.feedback_stream().unwrap();
while let Some(feedback) = feedback_rx.recv().await {
    println!("Progress: {:.1}%", feedback.progress * 100.0);
}
```

### Error Handling

**ROS 2:**
```cpp
try {
    auto result_future = client->async_get_result(goal_handle);
    auto result = result_future.get();
} catch (const std::exception& e) {
    // Handle error
}
```

**ros-z:**
```rust
match goal_handle.result().await {
    Ok(result) => println!("Success: {:?}", result),
    Err(e) => eprintln!("Error: {:?}", e),
}
```