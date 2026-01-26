#!/usr/bin/env nu

# Pure Rust Test Suite - No ROS dependencies required
# This script tests ros-z in a pure Rust environment using bundled message definitions

# Check if running in CI environment
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

def log-header [] {
    let header = "Pure Rust Test Suite (No ROS Required)"
    let separator = (1..50 | each { "=" } | str join "")
    print $"\n($separator)"
    print $header
    print $"($separator)\n"
}

# Command execution wrapper
def run-cmd [cmd: string] {
    print $"($cmd)\n"
    if (is-ci) {
        # In CI, environment is already set up
        nu -c $cmd
    } else {
        # Locally, wrap with nix develop
        nix develop '.#pureRust' -c nu -c $cmd
    }
}

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

# ============================================================================
# Test Suite Configuration
# ============================================================================

def get-test-pipeline [] {
    [
        "clippy-workspace"
        "run-tests"
        "check-bundled-msgs"
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
    log-header

    get-test-pipeline | each { |test_name|
        match $test_name {
            "clippy-workspace" => { clippy-workspace }
            "run-tests" => { run-tests }
            "check-bundled-msgs" => { check-bundled-msgs }
        }
    }

    print "\n================================================"
    log-success "All pure Rust tests passed!"
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
                    text: "Use 'test-pure-rust.nu list' to see available tests"
                    span: (metadata $test_name).span
                }
            }
        }

        match $test_name {
            "clippy-workspace" => { clippy-workspace }
            "run-tests" => { run-tests }
            "check-bundled-msgs" => { check-bundled-msgs }
        }
    }
}

# ============================================================================
# Main Entry Point
# ============================================================================

# Run pure Rust test suite (no ROS dependencies)
#
# Examples:
#   ./test-pure-rust.nu                      # Run all tests
#   ./test-pure-rust.nu clippy-workspace     # Run specific test
#   ./test-pure-rust.nu list                 # List available test functions
def main [
    ...tests: string  # Specific test functions to run (optional)
] {
    if ($tests | is-empty) {
        run-all-tests
    } else if ($tests | first) == "list" {
        main list
    } else {
        run-specific-tests ...$tests
    }
}
