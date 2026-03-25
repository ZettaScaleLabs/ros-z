#!/usr/bin/env nu

# Build ros-z-py and ros-z-msgs-py distribution wheels locally.
#
# Mirrors the steps in .github/workflows/release.yml so you can verify the
# wheel build and test pip-install before pushing a release tag.
#
# Output:
#   crates/ros-z-py/dist/      ← ros-z-py wheels (one per distro)
#   crates/ros-z-msgs/python/dist/  ← ros-z-msgs-py wheel + sdist
#
# Examples:
#   ./scripts/build-python-wheels.nu                     # jazzy + humble
#   ./scripts/build-python-wheels.nu --distros [jazzy]   # jazzy only
#   ./scripts/build-python-wheels.nu --install           # build + pip install into .venv

use lib/common.nu *

def build-msgs-wheel [] {
    log-step "Build ros-z-msgs-py wheel"
    run-cmd "cd crates/ros-z-msgs/python && pip install build && python -m build" --shell bash
}

def build-ros-z-py-wheel [distro: string] {
    log-step $"Build ros-z-py wheel for ($distro)"
    run-cmd $"
        cd crates/ros-z-py
        maturin build --release --strip \
            --features 'pyo3/extension-module,($distro)' \
            --no-default-features \
            --out dist
    " --shell bash --distro $distro
}

def rename-wheel [distro: string] {
    log-step $"Rename wheel to include distro suffix: ($distro)"
    let whl = (ls crates/ros-z-py/dist/*.whl | where name !~ $"-($distro)\.whl$" | get name)
    for w in $whl {
        let dest = ($w | str replace --regex '\.whl$' $"-($distro).whl")
        mv $w $dest
        print $"  ($w | path basename) → ($dest | path basename)"
    }
}

def install-into-venv [] {
    log-step "Install wheels into crates/ros-z-py/.venv"

    if not ("crates/ros-z-py/.venv" | path exists) {
        run-cmd "python -m venv crates/ros-z-py/.venv" --shell bash
    }

    let msgs_wheel = (ls crates/ros-z-msgs/python/dist/*.whl | first | get name)
    run-cmd $"
        source crates/ros-z-py/.venv/bin/activate
        pip install --force-reinstall ($msgs_wheel)
    " --shell bash

    let py_wheel = (ls crates/ros-z-py/dist/*.whl | first | get name)
    run-cmd $"
        source crates/ros-z-py/.venv/bin/activate
        pip install --force-reinstall ($py_wheel)
    " --shell bash

    print ""
    log-success "Installed. Activate with:"
    print $"  source crates/ros-z-py/.venv/bin/activate"
}

def print-install-hint [] {
    print "\n── Built wheels ──────────────────────────────────────────"
    ls crates/ros-z-msgs/python/dist/*.whl | get name | each { |w| print $"  ($w)" }
    ls crates/ros-z-py/dist/*.whl | get name | each { |w| print $"  ($w)" }
    print ""
    print "To pip-install from the local files, run:"
    let msgs_whl = (ls crates/ros-z-msgs/python/dist/*.whl | first | get name)
    let py_whl   = (ls crates/ros-z-py/dist/*.whl | first | get name)
    print $"  pip install ($msgs_whl)"
    print $"  pip install ($py_whl)"
    print "──────────────────────────────────────────────────────────"
}

# Build ros-z-py and ros-z-msgs-py wheels locally (mirrors the release CI workflow).
#
# Examples:
#   ./scripts/build-python-wheels.nu
#   ./scripts/build-python-wheels.nu --distros [jazzy]
#   ./scripts/build-python-wheels.nu --install
def main [
    --distros: list<string> = ["jazzy", "humble"]  # Distros to build wheels for
    --install                                       # Install built wheels into crates/ros-z-py/.venv
] {
    for d in $distros { validate-distro $d }

    log-header "Building Python wheels" ($distros | str join ", ")

    # Clean previous build artefacts
    rm -rf crates/ros-z-py/dist crates/ros-z-msgs/python/dist

    build-msgs-wheel

    for distro in $distros {
        build-ros-z-py-wheel $distro
        rename-wheel $distro
    }

    if $install {
        install-into-venv
    } else {
        print-install-hint
    }

    log-success "Done"
}
