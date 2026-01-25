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

def in-nix-shell [] {
    ($env.IN_NIX_SHELL? | default "") != ""
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
    let header = "Python (ros-z-py) - ROS 2 " + ($distro | str upcase) + " Test Suite"
    let separator = (1..50 | each { "=" } | str join "")
    print $"\n($separator)"
    print $header
    print $"($separator)\n"
}

# Command execution wrapper
def run-cmd [cmd: string] {
    let distro = get-distro

    print $"($cmd)\n"
    if (is-ci) or (in-nix-shell) {
        # In CI or already in nix shell, environment is already set up
        bash -c $cmd
    } else {
        # Locally, wrap with nix develop
        nix develop $".#ros-($distro)" -c bash -c $cmd
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
    } else if (in-nix-shell) {
        print $"  Already in nix develop shell for (get-distro)"
    } else {
        print $"  Using nix develop for (get-distro)"
    }
}

# --- Linting Functions ---

def lint-ruff [] {
    log-step "Ruff linting (ros-z-py)"

    run-cmd "cd ros-z-py; source .venv/bin/activate && ruff check tests/ examples/ --output-format=github"
}

def format-ruff [] {
    log-step "Ruff format check (ros-z-py)"

    run-cmd "cd ros-z-py; source .venv/bin/activate && ruff format --check tests/ examples/"
}

def format-ruff-fix [] {
    log-step "Ruff format fix (ros-z-py)"

    run-cmd "cd ros-z-py; source .venv/bin/activate && ruff format tests/ examples/"
}

# --- Type Checking Functions ---

def typecheck-mypy [] {
    log-step "MyPy type checking (ros-z-py)"

    run-cmd "cd ros-z-py; source .venv/bin/activate && mypy tests/ examples/ --ignore-missing-imports"
}

# --- Build Functions ---

def build-package [] {
    log-step "Build Python package (ros-z-py)"

    run-cmd "cd ros-z-py; source .venv/bin/activate && maturin build --release"
}

def clippy [] {
    log-step "Clippy (ros-z-py)"

    run-cmd "cargo clippy -p ros-z-py --all-targets -- -D warnings"
}

def setup-venv [] {
    log-step "Set up Python virtual environment"

    # Create venv if it doesn't exist
    if not ("ros-z-py/.venv" | path exists) {
        run-cmd "cd ros-z-py; python -m venv .venv"
        print "  Created new virtual environment"
    } else {
        print "  ✓ Virtual environment exists"
    }

    # Install ros-z-msgs-py (pure Python message definitions)
    run-cmd "cd ros-z-py; source .venv/bin/activate && pip install -e ../ros-z-msgs/python/"
    print "  ✓ Installed ros-z-msgs-py (message types)"

    # Install ros-z-py in editable mode using maturin
    run-cmd "cd ros-z-py; source .venv/bin/activate && RUSTFLAGS='-D warnings' maturin develop"
    print "  ✓ Installed ros-z-py (Rust bindings)"
}

# --- Test Functions ---

def run-pytest [] {
    log-step "Run pytest (ros-z-py)"

    run-cmd "cd ros-z-py; source .venv/bin/activate && python -m pytest tests/ -v"
}

def run-pytest-coverage [] {
    log-step "Run pytest with coverage (ros-z-py)"

    run-cmd "cd ros-z-py; source .venv/bin/activate && python -m pytest tests/ --cov=ros_z_py --cov-report=term-missing --cov-report=html --cov-fail-under=80"
}

def run-examples [] {
    log-step "Run Python examples (ros-z-py)"

    # Check if examples directory exists
    if not ("ros-z-py/examples" | path exists) {
        print "  ⚠  Skipping: no examples directory found"
        return
    }

    # # FIXME: workaround the timeout exit code
    # run-cmd "cd ros-z-py; source .venv/bin/activate && timeout 2 python examples/talker.py"
}

def run-python-interop [] {
    log-step "Run Python interop tests (ros-z-tests)"

    # These tests verify Rust<->Python pub/sub and service communication
    # Requires the Python venv to be set up first
    run-cmd "source ros-z-py/.venv/bin/activate && cargo test --features python-interop -p ros-z-tests --test python_interop -- --nocapture"
}

# --- Cleanup Functions ---

def cleanup-python [] {
    if (is-ci) {
        log-step "Cleaning up Python artifacts"

        try {
            rm -rf ros-z-py/.pytest_cache
            rm -rf ros-z-py/.mypy_cache
            rm -rf ros-z-py/.ruff_cache
            rm -rf ros-z-py/__pycache__
            rm -rf ros-z-py/python/**/__pycache__
            rm -rf ros-z-py/htmlcov
            rm -rf ros-z-py/.coverage
            rm -rf ros-z-py/dist
            rm -rf ros-z-py/build
            rm -rf ros-z-py/*.egg-info
        }

        df -h
    }
}

# ============================================================================
# Test Suite Configuration
# ============================================================================

# Define test pipeline as list of names
def get-test-pipeline [] {
    [
        "setup-venv"
        "clippy"
        "lint-ruff"
        "format-ruff"
        "build-package"
        "typecheck-mypy"
        "run-pytest"
        "run-python-interop"
        "run-examples"
        "cleanup-python"
    ]
}

# List all available test functions
def "main list" [] {
    print "Available test functions:"
    get-test-pipeline | each { |name| print $"  - ($name)" } | ignore
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
            "clippy" => { clippy }
            "lint-ruff" => { lint-ruff }
            "format-ruff" => { format-ruff }
            "format-ruff-fix" => { format-ruff-fix }
            "typecheck-mypy" => { typecheck-mypy }
            "build-package" => { build-package }
            "setup-venv" => { setup-venv }
            "run-pytest" => { run-pytest }
            "run-pytest-coverage" => { run-pytest-coverage }
            "run-python-interop" => { run-python-interop }
            "run-examples" => { run-examples }
            "cleanup-python" => { cleanup-python }
        }
    } | ignore

    print "\n================================================"
    log-success $"All Python ros-z-py tests passed for ($distro | str upcase)!"
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
                    text: "Use 'test-python.nu list' to see available tests"
                    span: (metadata $test_name).span
                }
            }
        }

        match $test_name {
            "clippy" => { clippy }
            "lint-ruff" => { lint-ruff }
            "format-ruff" => { format-ruff }
            "format-ruff-fix" => { format-ruff-fix }
            "typecheck-mypy" => { typecheck-mypy }
            "build-package" => { build-package }
            "setup-venv" => { setup-venv }
            "run-pytest" => { run-pytest }
            "run-pytest-coverage" => { run-pytest-coverage }
            "run-python-interop" => { run-python-interop }
            "run-examples" => { run-examples }
            "cleanup-python" => { cleanup-python }
        }
    }
}

# ============================================================================
# Main Entry Point
# ============================================================================

# Run Python test suite
#
# Examples:
#   ./test-python.nu                    # Run all tests with default distro (jazzy)
#   ./test-python.nu humble             # Run all tests for humble
#   ./test-python.nu jazzy lint-ruff typecheck-mypy  # Run specific tests
#   ./test-python.nu list               # List available test functions
def main [
    ...args: string
] {
    # Parse arguments
    let distro = if ($args | is-empty) {
        "jazzy"
    } else if ($args | first) in $DISTROS {
        $args | first
    } else if ($args | first) == "list" {
        "list"
    } else {
        "jazzy"
    }

    let tests = if ($args | is-empty) {
        []
    } else if ($args | first) in $DISTROS {
        $args | skip 1
    } else if ($args | first) == "list" {
        ["list"]
    } else {
        $args
    }

    # Handle list
    if "list" in $tests {
        main list
        return
    }

    # Validate distro
    if $distro not-in $DISTROS {
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
