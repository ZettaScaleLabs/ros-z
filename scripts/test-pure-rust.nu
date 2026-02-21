#!/usr/bin/env nu

# Pure Rust Test Suite - No ROS dependencies required
# This script tests ros-z in a pure Rust environment using bundled message definitions

use lib/common.nu *

# ============================================================================
# Test Functions
# ============================================================================

def clippy-workspace [] {
    log-step "Clippy (default workspace)"
    run-cmd "cargo clippy --all-targets -- -D warnings"
}

def run-tests [] {
    # Treat warnings as errors
    $env.RUSTFLAGS = "-D warnings"

    log-step "Run tests"
    let result = (do -i { run-cmd "cargo nextest run" | complete })

    # If tests failed, retry with debug logging for better diagnostics
    if $result.exit_code != 0 {
        print "\n⚠️  Some tests failed. Retrying failed tests with debug logging..."
        $env.RUST_LOG = "ros_z=debug,warn"
        run-cmd "cargo nextest run --failed"
    }
}

def check-bundled-msgs [] {
    log-step "Check ros-z-msgs with bundled messages"
    run-cmd "cargo check -p ros-z-msgs"
    run-cmd "cargo check -p ros-z-msgs --features bundled_msgs"
    run-cmd "cargo check -p ros-z-msgs --features common_interfaces"
    run-cmd "cargo build -p ros-z-msgs --no-default-features --features std_msgs"
    run-cmd "cargo build -p ros-z-msgs --no-default-features --features geometry_msgs"
    run-cmd "cargo build -p ros-z-msgs --no-default-features --features sensor_msgs"
    run-cmd "cargo build -p ros-z-msgs --no-default-features --features nav_msgs"
}

def check-console [] {
    log-step "Check ros-z-console"
    run-cmd "cargo check -p ros-z-console"
    run-cmd "cargo clippy -p ros-z-console -- -D warnings"
}

def check-examples [] {
    log-step "Build all examples (cargo build --examples)"
    run-cmd "cargo build --examples"
}

def check-distro-features [] {
    log-step "Check distro feature flags"
    run-cmd "cargo check -p ros-z --no-default-features --features humble"
    run-cmd "cargo check -p ros-z --no-default-features --features jazzy"
    run-cmd "cargo check -p ros-z --no-default-features --features rolling"
    run-cmd "cargo check -p ros-z --no-default-features --features iron"
}

def test-shm [] {
    log-step "Test SHM functionality"

    # Run tests without debug logging first (faster)
    # Library unit tests (ShmConfig, ShmProviderBuilder)
    let lib_result = (do -i { run-cmd "cargo test --package ros-z --lib shm" | complete })
    # Integration-style unit tests (pub/sub with SHM)
    let test_result = (do -i { run-cmd "cargo test --package ros-z --test shm" | complete })
    # Integration tests (validate shm_pointcloud2 example)
    let example_result = (do -i { run-cmd "cargo test --package ros-z-tests --test shm_example" | complete })

    # If any failed, retry with debug logging
    if $lib_result.exit_code != 0 or $test_result.exit_code != 0 or $example_result.exit_code != 0 {
        print "\n⚠️  SHM tests failed. Retrying with debug logging..."
        $env.RUST_LOG = "ros_z=debug,warn"
        run-cmd "cargo test --package ros-z --lib shm -- --nocapture"
        run-cmd "cargo test --package ros-z --test shm -- --nocapture"
        run-cmd "cargo test --package ros-z-tests --test shm_example -- --nocapture"
    }
}

# ============================================================================
# Test Suite Configuration
# ============================================================================

def get-test-map [] {
    {
        clippy-workspace: { clippy-workspace }
        run-tests: { run-tests }
        check-bundled-msgs: { check-bundled-msgs }
        check-console: { check-console }
        check-examples: { check-examples }
        check-distro-features: { check-distro-features }
        test-shm: { test-shm }
    }
}

def get-test-pipeline [] {
    [
        "clippy-workspace"
        "run-tests"
        "check-bundled-msgs"
        "check-console"
        "check-examples"
        "check-distro-features"
        "test-shm"
    ]
}

# ============================================================================
# Main Entry Point
# ============================================================================

# Run pure Rust test suite (no ROS dependencies)
#
# Examples:
#   ./test-pure-rust.nu                      # Run all tests
#   ./test-pure-rust.nu clippy-workspace     # Run specific test
#   ./test-pure-rust.nu --list               # List available test functions
def main [
    --list                # List available test functions
    ...tests: string      # Specific test functions to run (optional)
] {
    if $list {
        print "Available test functions:"
        get-test-pipeline | each { |name| print $"  - ($name)" }
        return
    }

    let test_map = get-test-map
    let pipeline = get-test-pipeline

    let tests_to_run = if ($tests | is-empty) { $pipeline } else { $tests }

    # Validate test names
    for test_name in $tests_to_run {
        if $test_name not-in $pipeline {
            error make {
                msg: $"Test function '($test_name)' not found"
                label: {
                    text: "Use './test-pure-rust.nu --list' to see available tests"
                    span: (metadata $test_name).span
                }
            }
        }
    }

    log-header "Pure Rust Test Suite (No ROS Required)"

    run-test-pipeline $tests_to_run { |test_name|
        do ($test_map | get $test_name)
    }

    print "\n================================================"
    log-success "All pure Rust tests passed!"
    print "================================================"
}
