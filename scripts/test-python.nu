#!/usr/bin/env nu

# Python (ros-z-py) Test Suite
# Tests Python bindings for ros-z

use lib/common.nu *

# ============================================================================
# Test Functions
# ============================================================================

# --- Setup ---

def setup-nix-env [] {
    log-step "Setting up Nix environment"

    if (is-ci) {
        print "  Using pre-configured CI environment"
    } else if (in-nix-shell) {
        print $"  Already in nix develop shell for (get-distro)"
    } else {
        print $"  Using nix develop for (get-distro)"
    }
}

def setup-venv [] {
    log-step "Set up Python virtual environment"

    # Create venv if it doesn't exist
    if not ("crates/ros-z-py/.venv" | path exists) {
        run-cmd "cd crates/ros-z-py; python -m venv .venv" --shell bash --distro (get-distro)
        print "  Created new virtual environment"
    } else {
        print "  Virtual environment exists"
    }

    # Build ros-z-msgs with python_registry feature to generate Python types
    run-cmd "cargo build -p ros-z-msgs --features python_registry" --shell bash --distro (get-distro)
    print "  Generated Python message types"

    # Install ros-z-msgs-py (pure Python message definitions)
    run-cmd "cd crates/ros-z-py; source .venv/bin/activate && pip install -e ../ros-z-msgs/python/" --shell bash --distro (get-distro)
    print "  Installed ros-z-msgs-py (message types)"

    # Install ros-z-py in editable mode using maturin
    run-cmd "cd crates/ros-z-py; source .venv/bin/activate && RUSTFLAGS='-D warnings' maturin develop" --shell bash --distro (get-distro)
    print "  Installed ros-z-py (Rust bindings)"
}

# --- Linting Functions ---

def lint-ruff [] {
    log-step "Ruff linting (ros-z-py)"
    run-cmd "cd crates/ros-z-py; source .venv/bin/activate && ruff check tests/ examples/ --output-format=github" --shell bash --distro (get-distro)
}

def format-ruff [] {
    log-step "Ruff format check (ros-z-py)"
    run-cmd "cd crates/ros-z-py; source .venv/bin/activate && ruff format --check tests/ examples/" --shell bash --distro (get-distro)
}

def format-ruff-fix [] {
    log-step "Ruff format fix (ros-z-py)"
    run-cmd "cd crates/ros-z-py; source .venv/bin/activate && ruff format tests/ examples/" --shell bash --distro (get-distro)
}

# --- Type Checking Functions ---

def typecheck-mypy [] {
    log-step "MyPy type checking (ros-z-py)"
    run-cmd "cd crates/ros-z-py; source .venv/bin/activate && mypy tests/ examples/ --ignore-missing-imports" --shell bash --distro (get-distro)
}

# --- Build Functions ---

def build-package [] {
    log-step "Build Python package (ros-z-py)"
    run-cmd "cd crates/ros-z-py; source .venv/bin/activate && maturin build" --shell bash --distro (get-distro)
}

def clippy [] {
    log-step "Clippy (ros-z-py)"
    run-cmd "cargo clippy -p ros-z-py --all-targets -- -D warnings" --shell bash --distro (get-distro)
}

# --- Test Functions ---

def run-pytest [] {
    log-step "Run pytest (ros-z-py)"
    run-cmd "cd crates/ros-z-py; source .venv/bin/activate && python -m pytest tests/ -v" --shell bash --distro (get-distro)
}

def run-pytest-coverage [] {
    log-step "Run pytest with coverage (ros-z-py)"
    run-cmd "cd crates/ros-z-py; source .venv/bin/activate && python -m pytest tests/ --cov=ros_z_py --cov-report=term-missing --cov-report=html --cov-fail-under=80" --shell bash --distro (get-distro)
}

def run-examples [] {
    log-step "Run Python examples (ros-z-py)"

    if not ("crates/ros-z-py/examples" | path exists) {
        print "  Skipping: no examples directory found"
        return
    }

    # FIXME: workaround the timeout exit code
    # run-cmd "cd crates/ros-z-py; source .venv/bin/activate && timeout 2 python examples/talker.py" --shell bash --distro (get-distro)
}

def run-python-interop [] {
    log-step "Run Python interop tests (ros-z-tests)"
    run-cmd "source crates/ros-z-py/.venv/bin/activate && cargo test --features python-interop -p ros-z-tests --test python_interop -- --nocapture" --shell bash --distro (get-distro)
}

# --- Cleanup Functions ---

def cleanup-python [] {
    if (is-ci) {
        log-step "Cleaning up Python artifacts"

        try {
            rm -rf crates/ros-z-py/.pytest_cache
            rm -rf crates/ros-z-py/.mypy_cache
            rm -rf crates/ros-z-py/.ruff_cache
            rm -rf crates/ros-z-py/__pycache__
            rm -rf crates/ros-z-py/python/**/__pycache__
            rm -rf crates/ros-z-py/htmlcov
            rm -rf crates/ros-z-py/.coverage
            rm -rf crates/ros-z-py/dist
            rm -rf crates/ros-z-py/build
            rm -rf crates/ros-z-py/*.egg-info
        }

        df -h
    }
}

# ============================================================================
# Test Suite Configuration
# ============================================================================

def get-test-map [] {
    {
        setup-venv: { setup-venv }
        clippy: { clippy }
        lint-ruff: { lint-ruff }
        format-ruff: { format-ruff }
        format-ruff-fix: { format-ruff-fix }
        build-package: { build-package }
        typecheck-mypy: { typecheck-mypy }
        run-pytest: { run-pytest }
        run-pytest-coverage: { run-pytest-coverage }
        run-python-interop: { run-python-interop }
        run-examples: { run-examples }
        cleanup-python: { cleanup-python }
    }
}

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

# ============================================================================
# Main Entry Point
# ============================================================================

# Run Python test suite
#
# Examples:
#   ./test-python.nu                           # Run all tests with default distro (jazzy)
#   ./test-python.nu --distro humble           # Run all tests for humble
#   ./test-python.nu --distro jazzy lint-ruff  # Run specific tests
#   ./test-python.nu --list                    # List available test functions
def main [
    --list                       # List available test functions
    --distro: string = "jazzy"   # ROS distro to test (humble, jazzy)
    ...tests: string             # Specific test functions to run (optional)
] {
    if $list {
        print "Available test functions:"
        get-test-pipeline | each { |name| print $"  - ($name)" }
        return
    }

    validate-distro $distro
    $env.DISTRO = $distro

    let test_map = get-test-map
    let pipeline = get-test-pipeline

    let tests_to_run = if ($tests | is-empty) { $pipeline } else { $tests }

    # Validate test names
    for test_name in $tests_to_run {
        if $test_name not-in $pipeline and $test_name not-in ["format-ruff-fix", "run-pytest-coverage"] {
            error make {
                msg: $"Test function '($test_name)' not found"
                label: {
                    text: "Use './test-python.nu --list' to see available tests"
                    span: (metadata $test_name).span
                }
            }
        }
    }

    log-header "Python (ros-z-py) Test Suite" $distro
    setup-nix-env

    run-test-pipeline $tests_to_run { |test_name|
        do ($test_map | get $test_name)
    }

    print "\n================================================"
    log-success $"All Python ros-z-py tests passed for ($distro | str upcase)!"
    print "================================================"
}
