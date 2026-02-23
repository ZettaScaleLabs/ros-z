#!/usr/bin/env nu

# Demo video generator for ros-z-console TUI
#
# Usage: nu scripts/demos/generate.nu
#
# Builds binaries, starts Zenoh router + publisher for realistic data,
# runs VHS on each tape, then cleans up background processes.

const WORKTREE = "/home/circle/Workings/ZettaScale/zenoh-ws/project/nix-ros/src/worktree-ros-z/console"
const ROUTER = "tcp/127.0.0.1:7447"

def main [] {
    cd $WORKTREE

    # --- Preflight checks ---
    for tool in ["vhs", "zenohd", "ttyd"] {
        if (which $tool | is-empty) {
            error make { msg: $"Required tool not found: ($tool). Run `nix develop` to enter the dev shell." }
        }
    }

    # Kill any orphaned processes from previous runs (VHS leaks ttyd + chromium)
    do -i { ^pkill zenohd }
    do -i { ^pkill ttyd }
    do -i { ^pkill -9 -f chromium }
    sleep 2sec

    # --- Build ---
    print "Building ros-z-console..."
    cargo build -p ros-z-console

    print "Building z_pubsub example..."
    cargo build --example z_pubsub

    mkdir scripts/demos/results
    mkdir _tmp

    # --- Start background processes ---
    print "Starting zenohd..."
    let zenohd_pid = (
        ^bash -c $"zenohd > _tmp/zenohd-demo.log 2>&1 & echo $!"
        | str trim
        | into int
    )
    sleep 1sec

    print "Starting z_pubsub example..."
    let pubsub_pid = (
        ^bash -c $"./target/debug/examples/z_pubsub > _tmp/pubsub-demo.log 2>&1 & echo $!"
        | str trim
        | into int
    )
    sleep 2sec

    print $"zenohd PID: ($zenohd_pid)  z_pubsub PID: ($pubsub_pid)"

    # --- Run tapes ---
    let all_tapes = (ls scripts/demos/tapes/*.tape | get name | sort)

    # 00-warmup.tape runs first to prime chromium/ttyd (cold-start latency absorber)
    # Retry until chromium is ready (can take several attempts on cold start)
    print "\nWarming up VHS (chromium cold start)..."
    mut warmed = false
    for attempt in 1..10 {
        let ok = (try { ^vhs ($all_tapes | first); true } catch { false })
        if $ok {
            print $"  Ready after ($attempt) attempts"
            $warmed = true
            break
        }
        print $"  Attempt ($attempt) failed, retrying in 3s..."
        sleep 3sec
    }
    if not $warmed {
        print "  Warning: warmup never succeeded, proceeding anyway"
    }

    # Real tapes (skip 00-warmup)
    let tapes = ($all_tapes | skip 1)

    let results = ($tapes | each {|tape|
        print $"\nRunning tape: ($tape)"
        mut success = false
        for attempt in 1..10 {
            let ok = (try { ^vhs $tape; true } catch { false })
            if $ok {
                $success = true
                break
            }
            print $"  Attempt ($attempt) failed, retrying in 3s..."
            sleep 3sec
        }
        if $success {
            print $"✅ ($tape)"
            # Clean up leaked VHS processes between tapes to prevent RAM exhaustion
            do -i { ^pkill -9 ttyd }
            do -i { ^pkill -9 -f chromium }
            sleep 5sec
        } else {
            print $"❌ ($tape)"
        }
        $success
    })
    let passed = ($results | where $it | length)
    let failed = ($results | where { not $in } | length)

    # --- Cleanup ---
    print "\nCleaning up..."
    do -i { ^kill $pubsub_pid }
    do -i { ^kill $zenohd_pid }
    do -i { ^pkill ttyd }
    do -i { ^pkill -f chromium }
    sleep 500ms

    # --- Summary ---
    print $"\n=== Demo generation complete ==="
    print $"Passed: ($passed) / ($passed + $failed)"
    print $"Output: scripts/demos/results/"
    ls scripts/demos/results/ | select name size | print
}
