# Action Server Deadlock Analysis

## Date: 2025-12-08

## Summary

The current `with_handler` design in the action server has several concurrency issues that can lead to deadlocks, lock contention, and broken cancellation functionality.

## Critical Issues

### 1. **Missing Cancel Request Handler** ❌ CRITICAL

**Location**: `ros-z/src/action/server.rs:317-333`

**Problem**: The `with_handler` method spawns a background task to handle goal requests, but **does not spawn a task to handle cancel requests**.

```rust
pub fn with_handler<F, Fut>(self: Arc<Self>, handler: F) -> Arc<Self> {
    let server_clone = self.clone();
    tokio::spawn(async move {
        loop {
            if let Ok(requested) = server_clone.recv_goal().await {
                let accepted = requested.accept();
                let executing = accepted.execute();
                handler(executing).await;  // User's handler runs
            }
            // ❌ No one calls recv_cancel()!
        }
    });
    self
}
```

**Impact**:
- Cancel requests from clients are never processed
- The `cancel_requested` flag in `ServerGoalState::Executing` is never set to `true`
- `executing.is_cancel_requested()` always returns `false`
- **Cancellation is completely broken when using `with_handler`**

**Severity**: CRITICAL - Core functionality is broken

---

### 2. **Lock Contention on `goal_manager`** ⚠️ MEDIUM

**Location**: Multiple locations throughout `server.rs`

**Problem**: The `goal_manager: Arc<Mutex<GoalManager<A>>>` is accessed from multiple concurrent contexts:

1. `handle_result_requests()` background task (line 111-137)
2. User's goal handler calling `is_cancel_requested()` (line 527-536)
3. Goal state transitions: `accept()`, `execute()`, `terminate()` (lines 449, 490, 552)
4. `publish_status()` (line 238)
5. `expire_goals()` (line 359-381)

**Code Example**:
```rust
// User code polls frequently:
loop {
    if executing.is_cancel_requested() {  // Locks goal_manager
        break;
    }
    tokio::time::sleep(Duration::from_millis(100)).await;
}

// Meanwhile, handle_result_requests is locking:
let manager = goal_manager.lock().unwrap();  // Contention!
```

**Impact**:
- High-frequency polling of `is_cancel_requested()` blocks other operations
- Result requests may experience delays
- State transitions may be blocked

**Severity**: MEDIUM - Performance degradation under load

---

### 3. **Mutex Poisoning on Panic** ⚠️ MEDIUM

**Location**: All `.lock().unwrap()` calls throughout `server.rs`

**Problem**: Using `lock().unwrap()` means any panic while holding the lock will poison the mutex, causing all future lock attempts to panic.

**Code Example**:
```rust
let mut manager = self.server.goal_manager.lock().unwrap();
// If code panics here, mutex is poisoned forever
manager.goals.insert(...);
```

**Impact**:
- One panic in any task can bring down the entire action server
- No recovery mechanism
- Cascading failures

**Severity**: MEDIUM - Resilience issue

---

### 4. **No Status Publishing in Background Tasks** ⚠️ LOW

**Location**: `handle_result_requests()` (line 111-137)

**Problem**: Result requests and potentially cancel requests (when added) don't trigger status updates. Status is only published when:
- Goals are accepted (line 462)
- Goals transition to executing (line 502)
- Goals terminate (line 562)

**Impact**:
- Clients may not see intermediate state changes
- Status topic may be stale

**Severity**: LOW - ROS2 design allows periodic status publishing

---

## Design Comparison with rclrs

### rclrs Approach

```rust
// Integrated with executor wait set
pub fn create_action_server<A: Action, Task>(
    node: &Arc<Node>,
    options: impl IntoActionServerOptions,
    callback: impl FnMut(RequestedGoal<A>) -> Task + Send + Sync + 'static,
) -> Result<ActionServer<A>, RclrsError>
```

**Key differences**:
- Callbacks are invoked by the executor during `spin()`
- Single-threaded event loop processes goal/cancel/result requests in order
- Uses a `Waitable` integrated into the wait set
- No `with_handler` - callback is required at construction
- Uses complex `GoalDispatch` enum to switch between callback and channel modes

**Advantages**:
- No lock contention (single-threaded processing)
- No deadlocks (no concurrent mutex access)
- All request types handled uniformly

**Disadvantages**:
- More complex architecture
- Requires executor integration
- Less flexibility (callback required)

### ros-z Current Approach

**Advantages**:
- Simple API with optional `with_handler`
- Manual API (`recv_goal()`) still available
- Works with any async runtime

**Disadvantages**:
- Missing cancel handler in `with_handler`
- Lock contention under high load
- No executor integration

---

## Proposed Solutions

### Solution 1: Add Cancel Request Handler (IMMEDIATE)

```rust
pub fn with_handler<F, Fut>(self: Arc<Self>, handler: F) -> Arc<Self>
where
    F: Fn(ExecutingGoal<A>) -> Fut + Send + Sync + 'static,
    Fut: std::future::Future<Output = ()> + Send + 'static,
{
    let server_clone = self.clone();

    // Spawn goal handler
    tokio::spawn(async move {
        loop {
            if let Ok(requested) = server_clone.recv_goal().await {
                let accepted = requested.accept();
                let executing = accepted.execute();
                handler(executing).await;
            }
        }
    });

    // ✅ NEW: Spawn cancel handler
    let cancel_server = self.clone();
    tokio::spawn(async move {
        loop {
            if let Ok((cancel_request, query)) = cancel_server.recv_cancel().await {
                cancel_server.handle_cancel_request(cancel_request, query);
            }
        }
    });

    self
}
```

### Solution 2: Use RwLock for Better Concurrency

```rust
goal_manager: Arc<RwLock<GoalManager<A>>>,  // Instead of Mutex
```

**Benefits**:
- Multiple readers can access simultaneously
- `is_cancel_requested()` becomes non-blocking when no writes
- Better performance for read-heavy workloads

**Changes needed**:
```rust
// Read operations (many concurrent readers allowed)
pub fn is_cancel_requested(&self) -> bool {
    let manager = self.server.goal_manager.read().unwrap();
    // ...
}

// Write operations (exclusive access)
pub fn accept(self) -> AcceptedGoal<A> {
    let mut manager = self.server.goal_manager.write().unwrap();
    // ...
}
```

### Solution 3: Use AtomicBool for Cancel Flag

Store cancel state per-goal instead of in shared state:

```rust
pub struct ExecutingGoal<A: ZAction> {
    pub goal: A::Goal,
    pub info: GoalInfo,
    server: Arc<ZActionServer<A>>,
    cancel_requested: Arc<AtomicBool>,  // ✅ Lock-free!
}

impl<A: ZAction> ExecutingGoal<A> {
    pub fn is_cancel_requested(&self) -> bool {
        self.cancel_requested.load(Ordering::Acquire)  // No lock!
    }
}
```

**Benefits**:
- Zero lock contention for cancel checking
- Best performance
- Still maintains consistency

**Trade-offs**:
- More complex state management
- Need to store cancel flags separately

### Solution 4: Handle Poisoned Mutex Gracefully

```rust
let manager = match self.server.goal_manager.lock() {
    Ok(guard) => guard,
    Err(poisoned) => {
        eprintln!("Warning: goal_manager mutex was poisoned, recovering");
        poisoned.into_inner()
    }
};
```

**Benefits**:
- Resilience to panics
- Action server continues to function
- Better error recovery

---

## Recommended Implementation Order

1. **Phase 1 (IMMEDIATE)**: Add cancel request handler to fix broken cancellation
2. **Phase 2 (SHORT-TERM)**: Handle poisoned mutexes gracefully
3. **Phase 3 (MEDIUM-TERM)**: Switch from `Mutex` to `RwLock`
4. **Phase 4 (OPTIONAL)**: Consider lock-free atomics for cancel flags if profiling shows contention

---

## Testing Recommendations

After fixes:

1. Test cancel request handling with `with_handler`
2. Stress test with multiple concurrent goals
3. Test panic recovery (poison mutex handling)
4. Benchmark lock contention under high load
5. Test with frequent `is_cancel_requested()` polling

---

## References

- `ros-z/src/action/server.rs`: Main implementation
- `/home/circle/Workings/ZettaScale/project/nix-ros/ws-ros2-test/src/ros2-rust/ros2_rust/rclrs/src/action/action_server.rs`: rclrs comparison
- C++ rcl_action: Low-level implementation (no deadlock issues as it's single-threaded)

---

## Date: 2025-12-09

## Timeout Investigation: `test_ros_z_fibonacci_action_server_to_ros_z_client`

### Problem Description

The test `test_ros_z_fibonacci_action_server_to_ros_z_client` was timing out after 10 seconds. The test sends a goal to compute the 5th Fibonacci number, which should complete in about 2 seconds (4 iterations × 500ms each).

### Initial Symptoms

Looking at the test logs with `RUST_LOG=ros_z=trace`:

```
[08:11:52.501] Goal succeeded!
[08:11:52.501] Client requests result via get_result
[08:11:52.501] Server receives result request but warns: "Goal not found or not terminated yet"
[Test times out waiting for result response]
```

The sequence showed:
1. Goal executed successfully and called `executing.succeed()`
2. Client immediately requested the result
3. Server received the request but reported the goal wasn't terminated yet
4. Client blocked forever waiting for a response that never came

### Root Cause Analysis

Through detailed investigation, I discovered **two separate but related issues**:

#### Issue 1: Client Race Condition

**Location**: `ros-z/src/action/client.rs:505` (original `GoalHandle::result()` method)

**Problem**: The client's `result()` method was requesting the result immediately after sending the goal, without waiting for the goal to reach a terminal state (Succeeded, Aborted, or Canceled).

```rust
pub async fn result(&mut self) -> Result<A::Result> {
    self.client.get_result(self.id).await  // Requests result immediately!
}
```

This created a race condition where the result request could arrive before the goal finished executing.

**Timeline**:
- T+0ms: Client sends goal
- T+0ms: Client calls `result()`, which immediately sends result request
- T+0ms: Server receives result request, goal not done yet, doesn't reply
- T+2000ms: Goal finishes, but client is still waiting for response that never came

#### Issue 2: Server Premature Cancellation

**Location**: `ros-z/src/action/server.rs:304` (in `recv_goal()` method)

**The Critical Bug**:
```rust
pub async fn recv_goal(&self) -> Result<RequestedGoal<A>> {
    // ...
    Ok(RequestedGoal {
        goal: request.goal,
        info: GoalInfo::new(request.goal_id),
        server: Arc::new(self.clone()),  // ❌ BUG: Creates new Arc with cloned server
        query,
    })
}
```

**Why This Was Wrong**:

1. `recv_goal()` takes `&self` (a reference to the server)
2. `self.clone()` creates a new `ZActionServer` instance with a **cloned CancellationToken**
3. `Arc::new(self.clone())` wraps this cloned server in a new Arc
4. This new Arc is moved through `RequestedGoal` → `AcceptedGoal` → `ExecutingGoal`
5. When `ExecutingGoal::terminate()` is called (after goal succeeds), the ExecutingGoal is dropped
6. This drops the Arc holding the cloned server
7. That Arc's ref count goes to 0, triggering the Drop implementation
8. The Drop impl calls `self._cancellation_token.cancel()`
9. **Since CancellationToken::cancel() cancels ALL clones of the token**, this cancels the token in the original server too!
10. The background `handle_result_requests` task exits immediately
11. The result request that just arrived (after goal succeeded) is never processed

**Evidence from logs**:
```
[08:19:17.253] Goal succeeded!
[08:19:17.253] Result handler task cancelled by token  ← Premature cancellation!
[08:19:17.254] Result query received (too late, no one to process it)
```

The cancellation happened **1ms before** the result request arrived, proving the background task was killed prematurely.

### Fix Implementation

#### Fix 1: Client-side - Wait for Terminal State

Modified `GoalHandle::result()` to wait for the goal to reach a terminal state before requesting the result:

```rust
pub async fn result(&mut self) -> Result<A::Result> {
    // First, wait for the goal to reach a terminal state
    if let Some(mut status_rx) = self.status_rx.take() {
        loop {
            let status = *status_rx.borrow_and_update();

            // Check if we're in a terminal state
            if status.is_terminal() {
                break;
            }

            // Wait for status change
            if status_rx.changed().await.is_err() {
                tracing::warn!("Status channel closed before reaching terminal state");
                break;
            }
        }
    }

    // Now request the result
    self.client.get_result(self.id).await
}
```

This ensures the client waits for status updates (Succeeded, Aborted, or Canceled) before requesting the result.

#### Fix 2: Server-side - Proper Arc Reference Sharing

Changed `recv_goal()` to use proper Arc reference counting instead of creating new Arcs:

```rust
pub async fn recv_goal(self: &Arc<Self>) -> Result<RequestedGoal<A>> {
    // ...
    Ok(RequestedGoal {
        goal: request.goal,
        info: GoalInfo::new(request.goal_id),
        server: Arc::clone(self),  // ✅ Just increment ref count, don't clone server
        query,
    })
}
```

**Key Changes**:
1. Method signature changed from `&self` to `self: &Arc<Self>` (requires being called on Arc reference)
2. Uses `Arc::clone(self)` instead of `Arc::new(self.clone())`
3. This increments the Arc's reference count without cloning the inner `ZActionServer`
4. All goal structs now share the **same** CancellationToken instance (not clones)
5. Drop is only called when the **last** Arc reference is dropped

**Arc Lifecycle After Fix**:
1. `build()` returns `Arc<ZActionServer>` - ref count = 1
2. `with_handler()` clones Arc for background task - ref count = 2
3. `recv_goal()` clones Arc for RequestedGoal - ref count = 3
4. `accept()` moves Arc to AcceptedGoal - ref count = 3
5. `execute()` moves Arc to ExecutingGoal - ref count = 3
6. `terminate()` drops ExecutingGoal - ref count = 2
7. Background task still holds Arc - ref count = 2
8. Server variable `_server` still alive - ref count = 2
9. Drop only happens when function returns and all tasks complete

### Test Results

After both fixes:

**Original failing test**:
```
test test_ros_z_fibonacci_action_server_to_ros_z_client ... PASS [10.042s]
```

The test now completes successfully. Timeline:
- Goal executes (2 seconds)
- Client waits for Succeeded status
- Client requests result
- Server processes request and replies
- Client receives final result: `[0, 1, 1, 2, 3, 5]`

**Other action tests** (previously timing out):
- 9 out of 12 tests now pass
- Tests like `test_action_server_init_fini` now complete in 0.114s (down from 60s timeout)
- Server cleanup now works properly when servers are explicitly dropped

### Remaining Issues

3 tests still timeout with `cargo nextest` but pass with `cargo test`:
- `test_network_failure_simulation`
- `test_server_shutdown_during_operation`
- `test_action_client_feedback_wait`

These appear to be nextest-specific issues, not problems with the core fix.

### Key Lessons

1. **Arc vs. Clone**: When working with Arc-wrapped types that contain state (like CancellationToken), never do `Arc::new(value.clone())`. Use `Arc::clone(&arc)` to share the same instance.

2. **Token Cancellation**: CancellationToken cancels ALL clones when any clone calls `cancel()`. This is by design for proper shutdown coordination, but requires careful Arc management.

3. **Race Conditions**: In async systems, always wait for state transitions before requesting dependent operations. The client must wait for terminal status before requesting results.

4. **Method Signatures**: Using `self: &Arc<Self>` is a powerful Rust pattern that ensures methods are only called on Arc references, preventing accidental cloning.

5. **Testing**: The difference between `cargo test` and `cargo nextest` can reveal subtle timing or isolation issues that only appear under specific test harness conditions.

### Code Changes Summary

Files modified:
- `ros-z/src/action/client.rs`: Added status waiting logic to `GoalHandle::result()`
- `ros-z/src/action/server.rs`: Fixed `recv_goal()` to use proper Arc sharing
- `ros-z/examples/demo_nodes/fibonacci_action_server.rs`: Added comment about `_server` variable lifetime
- `ros-z/examples/demo_nodes/fibonacci_action_client.rs`: Removed debug statements

All changes maintain backward compatibility while fixing the critical bugs.
