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
    run-cmd "cargo nextest run"
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

def test-fallback-mode [] {
    log-step "Test fallback mode (bundled assets)"

    print "  📦 Verifying ros-z-msgs can build without ROS 2 installation"

    # Ensure ROS environment variables are not set
    hide-env --ignore-errors AMENT_PREFIX_PATH
    hide-env --ignore-errors CMAKE_PREFIX_PATH
    hide-env --ignore-errors ROS_DISTRO

    # Verify environment is clean
    let ament = try { $env.AMENT_PREFIX_PATH } catch { "" }
    if $ament != "" {
        error make {
            msg: "AMENT_PREFIX_PATH is still set - ROS environment not clean"
            label: { text: $"Value: ($ament)" }
        }
    }

    print "  ✓ ROS environment variables cleared"

    # Clean and rebuild ros-z-msgs to force asset discovery
    run-cmd "cargo clean -p ros-z-msgs"

    # Build and capture output
    let build_output = (
        cargo build -p ros-z-msgs --features bundled_msgs
            --message-format=json-render-diagnostics
        | complete
    )

    if $build_output.exit_code != 0 {
        print "\n  ❌ Build failed!"
        print $build_output.stderr
        error make {
            msg: "ros-z-msgs build failed without ROS installation"
            label: { text: "Check that bundled assets are present in ros-z-codegen/assets/" }
        }
    }

    let stderr = $build_output.stderr

    # Verify bundled assets were used
    if ($stderr | str contains "Using local bundled assets") {
        print "  ✓ Bundled assets detected in build output"
    } else {
        print "\n  ❌ Expected 'Using local bundled assets' in build output"
        print $stderr
        error make {
            msg: "Bundled assets not detected - fallback mode failed"
            label: { text: "Check ros-z-msgs/build.rs discover_ros_packages()" }
        }
    }

    # Verify system packages were NOT used
    if ($stderr | str contains "Found") and ($stderr | str contains "packages from ROS 2 installation") {
        print "\n  ❌ System ROS packages were detected!"
        print $stderr
        error make {
            msg: "System ROS packages leaked into build"
            label: { text: "Environment not clean - check AMENT_PREFIX_PATH" }
        }
    }

    print "  ✓ System ROS packages were NOT used (correct)"
    print "  ✓ Fallback mode verified - standalone build works"
}

# ============================================================================
# Test Suite Configuration
# ============================================================================

def get-test-map [] {
    {
        test-fallback-mode: { test-fallback-mode }
        clippy-workspace: { clippy-workspace }
        run-tests: { run-tests }
        check-bundled-msgs: { check-bundled-msgs }
        check-console: { check-console }
    }
}

def get-test-pipeline [] {
    [
        "test-fallback-mode"  # Run first to verify bundled assets
        "clippy-workspace"
        "run-tests"
        "check-bundled-msgs"
        "check-console"
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
