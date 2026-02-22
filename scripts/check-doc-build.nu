#!/usr/bin/env nu

# Doc-build checker for the feedback-loop agent (Phase A).
#
# Runs the 6 build/test checks that verify documented commands actually work.
# Outputs a machine-readable result table to stdout and exits non-zero if any
# check fails.
#
# Usage (from repo root):
#   nu scripts/check-doc-build.nu
#   nu scripts/check-doc-build.nu --cargo /nix/store/.../bin/cargo
#   nu scripts/check-doc-build.nu --json    # JSON output for agents

use lib/common.nu *

const CHECKS = [
    { name: "build zenoh_router",   cmd: "cargo build --example zenoh_router" }
    { name: "build z_pubsub",       cmd: "cargo build --example z_pubsub" }
    { name: "build z_srvcli",       cmd: "cargo build --example z_srvcli" }
    { name: "build z_custom_message", cmd: "cargo build --example z_custom_message" }
    { name: "test ros-z-cdr",       cmd: "cargo test -p ros-z-cdr" }
    { name: "test ros-z-codegen",   cmd: "cargo test -p ros-z-codegen" }
]

# Run doc-build Phase A checks
#
# Examples:
#   nu scripts/check-doc-build.nu
#   nu scripts/check-doc-build.nu --cargo /nix/store/.../bin/cargo
#   nu scripts/check-doc-build.nu --json
def main [
    --cargo: string = "cargo"   # Path to cargo binary
    --json                      # Output JSON instead of table
] {
    print --stderr "\n==================================================\nDoc-Build Phase A Checks\n==================================================\n"

    mut results = []

    for check in $CHECKS {
        let cmd = $"($cargo) ($check.cmd | str replace 'cargo ' '')"

        print --stderr $"→ ($check.name)"
        print --stderr $"  ($cmd)"

        let outcome = do -i {
            (^$cargo ...(($check.cmd | str replace 'cargo ' '') | split row ' ')
                | complete)
        }

        let passed = $outcome.exit_code == 0
        let status = if $passed { "PASS" } else { "FAIL" }
        let icon   = if $passed { "✅" } else { "❌" }

        print --stderr $"  ($icon) ($status)\n"

        if not $passed {
            let tail = ($outcome.stderr | lines | last 10 | str join "\n")
            print --stderr $"  Error output:\n($tail)\n"
        }

        $results = ($results | append {
            check:  $check.name
            cmd:    $cmd
            status: $status
            passed: $passed
            stderr: (if $passed { "" } else { $outcome.stderr | lines | last 10 | str join "\n" })
        })
    }

    # Summary table
    let passed_count = ($results | where passed == true | length)
    let total = ($results | length)
    let all_passed = $passed_count == $total

    if $json {
        # Clean JSON to stdout; all progress was on stderr
        print ({
            passed:  $passed_count
            total:   $total
            ok:      $all_passed
            checks:  ($results | select check status stderr)
        } | to json)
    } else {
        print "\n─────────────────────────────────────────"
        print $"Phase A Results: ($passed_count)/($total) passed"
        print "─────────────────────────────────────────"
        $results | select check status | print
        print ""

        if $all_passed {
            log-success "All Phase A checks passed"
        } else {
            let failed = ($results | where passed == false | get check)
            print $"❌ Failed checks: ($failed | str join ', ')"
        }
    }

    if not $all_passed {
        exit 1
    }
}
