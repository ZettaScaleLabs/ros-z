# ROS-Z Action Examples

This directory contains examples demonstrating ROS 2 action functionality in ros-z.

## Fibonacci Action Example

Computes Fibonacci sequences using actions.

**Server:**
```bash
cargo run --example action_fibonacci -- server
```

**Client:**
```bash
cargo run --example action_fibonacci -- client [order]
```

The client sends a goal to compute Fibonacci(order), receives periodic feedback with partial sequences, and gets the final complete sequence.

## Navigation Action Example

Simulates robot navigation to a target pose.

**Server:**
```bash
cargo run --example action_navigation -- server
```

**Client:**
```bash
cargo run --example action_navigation -- client [target_x] [target_y]
```

The client sends a navigation goal, receives feedback with current position and distance remaining, and gets the final result. The navigation can be cancelled mid-execution.

## Features Demonstrated

- Action client/server creation
- Goal sending and handling
- Feedback streaming
- Result retrieval
- Goal cancellation
- Error handling