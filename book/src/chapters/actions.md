# Actions

Actions provide a mechanism for long-running tasks in ROS 2, with the ability to:

- Send goals to an action server
- Receive feedback during execution
- Cancel goals in progress
- Get final results

## Overview

Actions are useful for tasks that:
- Take a long time to complete
- Need to provide progress updates
- May need to be cancelled

Common examples include:
- Navigation to a goal
- Gripper control
- Long computations

## Components

An action consists of:

1. **Goal**: The desired outcome
2. **Feedback**: Progress updates during execution
3. **Result**: The final outcome

## Implementation

Action support in ros-z is under active development. Check the examples directory and API documentation for the latest implementation details.

## Example Use Cases

- Robot navigation with progress updates
- Trajectory execution with real-time feedback
- Complex multi-step operations

For concrete examples, see the ros-z examples directory.
