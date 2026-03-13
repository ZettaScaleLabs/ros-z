# Go Interop Tests for ros-z-go

This directory contains integration tests that verify interoperability between ros-z-go and ROS 2.

## Test Coverage

### Pub/Sub Tests (`pubsub_test.go`)

- ✅ Go publisher → ROS2 subscriber
- ✅ ROS2 publisher → Go subscriber
- ✅ Go publisher → Go subscriber

### Service Tests (`service_test.go`)

- 🚧 Go service server → ROS2 client (Skeleton implemented, awaiting FFI)
- 🚧 ROS2 service server → Go client (Skeleton implemented, awaiting FFI)
- 🚧 Go service server ↔ Go client (Skeleton implemented, awaiting FFI)
- 🚧 Custom service types (Skeleton implemented, awaiting FFI)

### Action Tests (`action_test.go`)

- 🚧 Go action server → ROS2 client (Skeleton implemented, awaiting FFI)
- 🚧 ROS2 action server → Go client (Skeleton implemented, awaiting FFI)
- 🚧 Go action server ↔ Go client (Skeleton implemented, awaiting FFI)
- 🚧 Goal cancellation (Skeleton implemented, awaiting FFI)
- 🚧 Feedback monitoring (Skeleton implemented, awaiting FFI)
- 🚧 Custom action types (Skeleton implemented, awaiting FFI)

## Prerequisites

1. **ROS 2** - Humble, Jazzy, or Kilted with source files

   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

2. **rmw_zenoh_cpp** - Zenoh RMW implementation

   ```bash
   sudo apt install ros-jazzy-rmw-zenoh-cpp
   ```

3. **Zenoh** - Zenoh router (`zenohd`)

   ```bash
   # Via cargo
   cargo install zenoh

   # Or via Nix (if using flake.nix)
   nix develop
   ```

4. **Generated Messages** - Run code generation first

   ```bash
   make codegen
   ```

5. **Rust Library** - Build FFI library

   ```bash
   make build-rust
   ```

## Running Tests

### Run All Interop Tests

```bash
cd crates/ros-z-go
go test -tags=integration ./interop_tests/... -v
```

### Run Specific Test Category

```bash
# Pub/Sub only
go test -tags=integration ./interop_tests/... -v -run TestGo.*Subscriber

# With ROS2 debug output
ROS_DOMAIN_ID=0 go test -tags=integration ./interop_tests/... -v
```

### Run Single Test

```bash
go test -tags=integration ./interop_tests/... -v -run TestGoPublisherToROS2Subscriber
```

### With Debug Logging

```bash
RUST_LOG=debug go test -tags=integration ./interop_tests/... -v
```

## Test Architecture

### Common Infrastructure (`common_test.go`)

**ZenohRouter:**

- Automatically starts/stops Zenoh router per test
- Uses unique port per test run (PID-based)
- Configures both ros-z-go and ROS2 to use same router

**Helper Functions:**

- `checkROS2Available()` - Verifies ROS2 CLI is available
- `checkZenohAvailable()` - Verifies zenohd is installed
- `waitForProcess()` - Handles timing for process startup

### Test Pattern

Each test follows this pattern:

1. Start Zenoh router
2. Create ros-z-go nodes/publishers/subscribers
3. Create ROS2 nodes/publishers/subscribers (via CLI)
4. Exchange messages/requests
5. Verify communication
6. Cleanup (automatic via defer)

## Current Status

| Test Type | Status | Notes |
|-----------|--------|-------|
| Pub/Sub (Go↔Go) | ✅ Working | Basic test implemented |
| Pub/Sub (Go↔ROS2) | ✅ Working | Tests both directions |
| Service (Go↔Go) | 🚧 Skeleton | Test skeleton ready, awaiting FFI implementation |
| Service (Go↔ROS2) | 🚧 Skeleton | Test skeleton ready, awaiting FFI implementation |
| Action (Go↔Go) | 🚧 Skeleton | Test skeleton ready, awaiting FFI implementation |
| Action (Go↔ROS2) | 🚧 Skeleton | Test skeleton ready, awaiting FFI implementation |

## Known Issues

1. **Timing Sensitivity** - Tests use fixed delays which may need adjustment on slower systems
2. **Port Conflicts** - If tests fail, orphaned zenohd processes may hold ports. Kill with:

   ```bash
   pkill zenohd
   ```

3. **ROS2 Discovery** - Sometimes needs extra time for rmw_zenoh_cpp discovery (increase sleep times if flaky)

## Adding New Tests

To add a new interop test:

1. Create test file in this directory (e.g., `service_test.go`)
2. Add build tag: `// +build integration`
3. Import common utilities: `import . "common_test"`
4. Use `startZenohRouter(t)` for infrastructure
5. Follow the test pattern above
6. Add to this README

## Troubleshooting

### Test fails with "zenohd not found"

Install Zenoh router or ensure it's in PATH.

### Test fails with "ros2 CLI not available"

Source ROS2 setup file before running tests.

### Test timeout / no messages received

- Check Zenoh router logs
- Verify RMW_IMPLEMENTATION is set correctly
- Increase sleep delays in test
- Check for port conflicts

### Generated messages not found

Run `make codegen` to generate message types first.

## Future Work

### High Priority (Blocked on FFI Implementation)

- [ ] Implement service FFI bindings in Rust (ros-z/src/ffi/)
- [ ] Implement action FFI bindings in Rust (ros-z/src/ffi/)
- [ ] Enable service interop tests (skeleton ready in service_test.go)
- [ ] Enable action interop tests (skeleton ready in action_test.go)

### Lower Priority (Enhancements)

- [ ] Add QoS configuration tests
- [ ] Add type hash verification tests
- [ ] Add performance benchmarks
- [ ] Add multi-node discovery tests
- [ ] Add stress tests (many concurrent goals, high-frequency feedback)
- [ ] Add timeout and error handling tests
