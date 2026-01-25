#!/usr/bin/env nu

# ROS-Specific Test Suite
# This script tests components that require ROS 2 environment:
# - rcl-z: C bindings to ROS 2 libraries
# - interop tests: Communication with native ROS 2 nodes via rmw_zenoh_cpp

# Configuration
const DISTROS = ["humble", "jazzy"]

# Get environment variables
def get-distro [] {
    $env.DISTRO? | default "jazzy"
}

def is-ci [] {
    ($env.CI? | default "false") == "true"
}

# Logging functions
def log-step [message: string] {
    if (is-ci) {
        print $"\n→ ($message)"
    } else {
        print $"\n(ansi blue)→ ($message)(ansi reset)"
    }
}

def log-success [message: string] {
    if (is-ci) {
        print $"✅ ($message)"
    } else {
        print $"(ansi green)✅ ($message)(ansi reset)"
    }
}

def log-header [distro: string] {
    let header = $"ROS 2 ($distro | str upcase) - rcl-z & Interop Tests"
    let separator = (1..50 | each { "=" } | str join "")
    print $"\n($separator)"
    print $header
    print $"($separator)\n"
}

# Command execution wrapper
def run-cmd [cmd: string] {
    let distro = get-distro

    print $"($cmd)\n"
    if (is-ci) {
        # In CI, environment is already set up
        nu -c $cmd
    } else {
        # Locally, wrap with nix develop
        nix develop $".#ros-($distro)" -c nu -c $cmd
    }
}

# ============================================================================
# Test Functions - ROS-Dependent Components Only
# ============================================================================

def clippy-rclz [] {
    log-step "Clippy rcl-z"

    let distro = get-distro
    let cmd = if $distro == "humble" {
        "cargo clippy -p rcl-z --lib --features humble_compat -- -D warnings"
    } else {
        "cargo clippy -p rcl-z --all-targets -- -D warnings"
    }

    run-cmd $cmd
}

def build-rclz [] {
    log-step "Build rcl-z"

    let distro = get-distro
    let cmd = if $distro == "humble" {
        "cargo build -p rcl-z --features humble_compat"
    } else {
        "cargo build -p rcl-z"
    }

    run-cmd $cmd
}

def run-ros-interop [] {
    log-step "Run interop tests (requires rmw_zenoh_cpp)"

    # Check if ros2 is available
    if (which ros2 | is-empty) {
        print "  ⚠  Skipping: ros2 CLI not available"
        return
    }

    $env.RMW_IMPLEMENTATION = "rmw_zenoh_cpp"

    let distro = get-distro
    let cmd = if $distro == "humble" {
        "cargo nextest run -p ros-z-tests --no-default-features --features ros-interop,humble"
    } else {
        "cargo nextest run -p ros-z-tests --features ros-interop"
    }

    run-cmd $cmd
}

# ============================================================================
# Test Suite Configuration
# ============================================================================

def get-test-pipeline [] {
    [
        "clippy-rclz"
        "build-rclz"
        "run-ros-interop"
    ]
}

# List all available test functions
def "main list" [] {
    print "Available test functions:"
    get-test-pipeline | each { |name| print $"  - ($name)" }
}

# ============================================================================
# Execution Modes
# ============================================================================

# Run all tests in sequence
def run-all-tests [] {
    let distro = get-distro
    log-header $distro

    get-test-pipeline | each { |test_name|
        match $test_name {
            "clippy-rclz" => { clippy-rclz }
            "build-rclz" => { build-rclz }
            "run-ros-interop" => { run-ros-interop }
        }
    }

    print "\n================================================"
    log-success $"All ROS 2 ($distro | str upcase) tests passed!"
    print "================================================"
}

# Run specific test functions
def run-specific-tests [
    ...test_names: string  # Names of test functions to run
] {
    let available_tests = get-test-pipeline

    for test_name in $test_names {
        if $test_name not-in $available_tests {
            error make {
                msg: $"Test function '($test_name)' not found"
                label: {
                    text: "Use 'test-ros.nu list' to see available tests"
                    span: (metadata $test_name).span
                }
            }
        }

        match $test_name {
            "clippy-rclz" => { clippy-rclz }
            "build-rclz" => { build-rclz }
            "run-ros-interop" => { run-ros-interop }
        }
    }
}

# ============================================================================
# Main Entry Point
# ============================================================================

# Run ROS-specific test suite (rcl-z and interop tests)
#
# Examples:
#   ./test-ros.nu                    # Run all tests with default distro (jazzy)
#   ./test-ros.nu humble             # Run all tests for humble
#   ./test-ros.nu jazzy clippy-rclz  # Run specific test
#   ./test-ros.nu list               # List available test functions
def main [
    distro?: string = "jazzy"  # ROS distro to test (humble, jazzy)
    ...tests: string           # Specific test functions to run (optional)
] {
    # Validate distro
    if $distro not-in $DISTROS and $distro != "list" {
        error make {
            msg: $"Unknown distro: ($distro)"
            label: {
                text: $"Must be one of: ($DISTROS | str join ', ')"
            }
        }
    }

    # Set distro environment variable
    $env.DISTRO = $distro

    # Determine execution mode
    if ($tests | is-empty) {
        run-all-tests
    } else {
        run-specific-tests ...$tests
    }
}
