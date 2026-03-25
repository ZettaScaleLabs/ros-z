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
    run-cmd "cd crates/ros-z-msgs/python && python -m build" --shell bash
}

def build-ros-z-py-wheel [distro: string] {
    log-step $"Build ros-z-py wheel for ($distro)"
    run-cmd $"cd crates/ros-z-py && maturin build --release --strip --features 'pyo3/extension-module,($distro)' --no-default-features --out dist" --shell bash --distro $distro
}

def rename-wheel [distro: string] {
    log-step $"Rename wheel to include distro suffix: ($distro)"
    # Only rename wheels that have no distro suffix yet (i.e. freshly built by maturin)
    let whl = (ls crates/ros-z-py/dist/*.whl
        | where { |f| not ($DISTROS | any { |d| ($f.name | str contains $"-($d)") }) }
        | get name)
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
        pip install --force-reinstall --no-deps ($py_wheel)
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
#   ./scripts/build-python-wheels.nu                  # jazzy + humble
#   ./scripts/build-python-wheels.nu jazzy            # jazzy only
#   ./scripts/build-python-wheels.nu jazzy humble     # explicit list
#   ./scripts/build-python-wheels.nu --install jazzy  # build + pip install
def main [
    --install           # Install built wheels into crates/ros-z-py/.venv
    ...distros: string  # Distros to build (default: jazzy humble)
] {
    let distros = if ($distros | is-empty) { ["jazzy", "humble"] } else { $distros }
    for d in $distros { validate-distro $d }

    log-header "Building Python wheels" ($distros | str join ", ")

    # Clean previous build artefacts
    rm -rf crates/ros-z-py/dist crates/ros-z-msgs/python/dist

    build-msgs-wheel

    # Build each distro wheel, install before renaming (rename makes filename invalid for pip)
    let primary = ($distros | first)
    for distro in $distros {
        build-ros-z-py-wheel $distro
        # Install only the first (primary) distro into venv
        if $install and ($distro == $primary) {
            install-into-venv
        }
        # Rename immediately so next distro build doesn't conflict
        rename-wheel $distro
    }

    if not $install {
        print-install-hint
    }

    log-success "Done"
}
