#!/usr/bin/env nu

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
    let header = $"ROS 2 ($distro | str upcase) - Comprehensive Test Suite"
    let separator = (1..50 | each { "=" } | str join "")
    print $"\n($separator)"
    print $header
    print $"($separator)\n"
}

# Command execution wrapper
def run-cmd [cmd: string] {
    let distro = get-distro
    
    if (is-ci) {
        # In CI, environment is already set up
        nu -c $cmd
    } else {
        # Locally, wrap with nix develop
        nix develop $".#ros-($distro)" -c nu -c $cmd
    }
}

# ============================================================================
# Test Functions - Single Source of Truth
# ============================================================================

# --- Setup ---

def setup-nix-env [] {
    log-step "Setting up Nix environment"
    
    if (is-ci) {
        # In CI, nix-dev-env.sh is already sourced by the workflow
        print "  Using pre-configured CI environment"
    } else {
        print $"  Using nix develop for (get-distro)"
    }
}

# --- Clippy Functions ---

def clippy-workspace [] {
    log-step "Clippy (workspace without rcl-z, without examples)"
    
    let distro = get-distro
    let cmd = if $distro == "humble" {
        "cargo clippy --workspace --exclude rcl-z --lib --bins --tests --no-default-features --features humble -- -D warnings"
    } else {
        "cargo clippy --workspace --exclude rcl-z --lib --bins --tests -- -D warnings"
    }
    
    run-cmd $cmd
}

def clippy-bundled-examples [] {
    log-step "Clippy bundled examples"
    
    let distro = get-distro
    let cmd = if $distro == "humble" {
        "cargo clippy -p ros-z --examples --no-default-features --features humble -- -D warnings"
    } else {
        "cargo clippy -p ros-z --examples -- -D warnings"
    }
    
    run-cmd $cmd
}

def clippy-external-examples [] {
    log-step "Clippy external_msgs examples"
    
    let distro = get-distro
    let cmd = if $distro == "humble" {
        "cargo clippy -p ros-z --examples --no-default-features --features external_msgs,humble -- -D warnings"
    } else {
        "cargo clippy -p ros-z --examples --features external_msgs -- -D warnings"
    }
    
    run-cmd $cmd
}

def clippy-rclz [] {
    log-step "Clippy rcl-z with distro-specific features"
    
    let distro = get-distro
    let cmd = if $distro == "humble" {
        "cargo clippy -p rcl-z --lib --features humble_compat -- -D warnings"
    } else {
        "cargo clippy -p rcl-z --all-targets -- -D warnings"
    }
    
    run-cmd $cmd
}

def cleanup-after-clippy [] {
    let distro = get-distro
    
    if ($distro == "humble" and (is-ci)) {
        log-step "Cleaning up to save disk space"
        cargo clean --release
        cargo clean --doc
        rm -rf target/debug/incremental
        df -h
    }
}

# --- Check Functions ---

def check-rclz [] {
    log-step "Check rcl-z"
    
    let distro = get-distro
    let cmd = if $distro == "humble" {
        "cargo check -p rcl-z --features humble_compat"
    } else {
        "cargo check -p rcl-z"
    }
    
    run-cmd $cmd
}

def check-ros-z-msgs-all [] {
    log-step "Check ros-z-msgs with all messages"
    
    let distro = get-distro
    let cmd = if $distro == "humble" {
        "cargo check -p ros-z-msgs --no-default-features --features all_msgs,humble"
    } else {
        "cargo check -p ros-z-msgs --features all_msgs"
    }
    
    run-cmd $cmd
}

def check-protobuf [] {
    let distro = get-distro
    
    if $distro == "humble" {
        log-step "Skipping protobuf check for Humble (disk space optimization)"
        return
    }
    
    log-step "Check with protobuf feature"
    run-cmd "cargo check -p ros-z -p ros-z-msgs --features ros-z/protobuf,ros-z-msgs/protobuf"
}

# --- Build Functions ---

def build-examples [] {
    log-step "Build examples"
    
    let distro = get-distro
    let cmd = if $distro == "humble" {
        "cargo build --examples --no-default-features --features external_msgs,humble"
    } else {
        "cargo build --examples --features external_msgs"
    }
    
    run-cmd $cmd
}

def cleanup-after-build [] {
    if (is-ci) {
        log-step "Aggressive cleanup after build"
        
        # Nushell's safer file operations
        try {
            glob target/debug/deps/*
            | where type != dir
            | where name !~ '\.so$'
            | each { |file| rm -f $file.name }
        }
        
        rm -rf target/debug/.fingerprint
        rm -rf target/debug/incremental
        rm -rf target/debug/build
        df -h
    }
}

def build-protobuf-demo [] {
    let distro = get-distro
    
    if $distro == "humble" {
        log-step "Skipping protobuf demo for Humble"
        return
    }
    
    log-step "Build protobuf demo"
    run-cmd "cargo build -p protobuf_demo"
}

# --- Test Functions ---

def run-unit-tests [] {
    log-step "Run unit tests"
    
    let distro = get-distro
    let cmd = if $distro == "humble" {
        "cargo nextest run --workspace --exclude rcl-z --no-default-features --features humble"
    } else {
        "cargo nextest run --workspace"
    }
    
    run-cmd $cmd
}

def run-interop-tests [] {
    log-step "Run interop tests (requires rmw_zenoh_cpp)"
    
    # Check if ros2 is available
    if (which ros2 | is-empty) {
        print "  ⚠  Skipping: ros2 CLI not available"
        return
    }
    
    $env.RMW_IMPLEMENTATION = "rmw_zenoh_cpp"
    
    let distro = get-distro
    let cmd = if $distro == "humble" {
        "cargo nextest run -p ros-z-tests --no-default-features --features interop-tests,humble"
    } else {
        "cargo nextest run -p ros-z-tests --features interop-tests"
    }
    
    run-cmd $cmd
}

# ============================================================================
# Test Suite Configuration
# ============================================================================

# Define test pipeline as list of names
def get-test-pipeline [] {
    [
        "clippy-workspace"
        "clippy-bundled-examples"
        "clippy-external-examples"
        "clippy-rclz"
        "cleanup-after-clippy"
        "check-rclz"
        "check-ros-z-msgs-all"
        "check-protobuf"
        "build-examples"
        "cleanup-after-build"
        "build-protobuf-demo"
        "run-unit-tests"
        "run-interop-tests"
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

    setup-nix-env

    # Run all tests in the pipeline
    get-test-pipeline | each { |test_name|
        match $test_name {
            "clippy-workspace" => { clippy-workspace }
            "clippy-bundled-examples" => { clippy-bundled-examples }
            "clippy-external-examples" => { clippy-external-examples }
            "clippy-rclz" => { clippy-rclz }
            "cleanup-after-clippy" => { cleanup-after-clippy }
            "check-rclz" => { check-rclz }
            "check-ros-z-msgs-all" => { check-ros-z-msgs-all }
            "check-protobuf" => { check-protobuf }
            "build-examples" => { build-examples }
            "cleanup-after-build" => { cleanup-after-build }
            "build-protobuf-demo" => { build-protobuf-demo }
            "run-unit-tests" => { run-unit-tests }
            "run-interop-tests" => { run-interop-tests }
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
    setup-nix-env

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
            "clippy-workspace" => { clippy-workspace }
            "clippy-bundled-examples" => { clippy-bundled-examples }
            "clippy-external-examples" => { clippy-external-examples }
            "clippy-rclz" => { clippy-rclz }
            "cleanup-after-clippy" => { cleanup-after-clippy }
            "check-rclz" => { check-rclz }
            "check-ros-z-msgs-all" => { check-ros-z-msgs-all }
            "check-protobuf" => { check-protobuf }
            "build-examples" => { build-examples }
            "cleanup-after-build" => { cleanup-after-build }
            "build-protobuf-demo" => { build-protobuf-demo }
            "run-unit-tests" => { run-unit-tests }
            "run-interop-tests" => { run-interop-tests }
        }
    }
}

# ============================================================================
# Main Entry Point
# ============================================================================

# Run ROS test suite
#
# Examples:
#   ./test-ros.nu                    # Run all tests with default distro (jazzy)
#   ./test-ros.nu humble             # Run all tests for humble
#   ./test-ros.nu humble clippy-workspace check-rclz  # Run specific tests
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