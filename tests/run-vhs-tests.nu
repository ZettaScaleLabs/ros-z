#!/usr/bin/env nu

# VHS-based TUI correctness tests for ros-z-console
#
# Follows the pattern from ~/nixos (zjbar) and ~/Repos/rust-related/ptt-hot.
#
# Usage:
#   nu tests/run-vhs-tests.nu
#
# Prerequisites (run once):
#   nix develop   # enter dev shell to get vhs, zenohd, ttyd in PATH
#
# Background processes started automatically:
#   zenohd          - Zenoh router
#   z_pubsub talker - Publishes std_msgs/String on /chatter at 1 Hz
#   z_pubsub listener - Subscribes to /chatter (makes the topic bidirectional)
#   z_srvcli server - Exposes example_interfaces/AddTwoInts service
#
# Assertions (following zjbar/ptt-hot approach):
#   - Screenshot existence  = app ran that code path without crashing
#   - Size differences      = state actually changed (filtered vs unfiltered, etc.)
#   - All screenshots exist = test passed

const WORKTREE = "/home/circle/Workings/ZettaScale/zenoh-ws/project/nix-ros/src/worktree-ros-z/console"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def run_tape [tape_path: string] {
    let before = (date now)
    let result = (try { ^vhs $tape_path | complete } catch { {exit_code: 1, stderr: "vhs threw"} })
    let after = (date now)
    {
        tape:        ($tape_path | path basename)
        success:     ($result.exit_code == 0)
        duration_ms: (($after - $before) / 1ms)
        exit_code:   $result.exit_code
        stderr:      ($result.stderr? | default "")
    }
}

def shot [path: string] {
    let exists = ($path | path exists)
    let size   = (if $exists { ls $path | get size | first | into int } else { 0 })
    { path: $path, exists: $exists, size: $size }
}

def sizes_differ [a: string, b: string] {
    let sa = (shot $a)
    let sb = (shot $b)
    ($sa.exists and $sb.exists and $sa.size != $sb.size)
}

def make_result [name: string, checks: record, error: string] {
    let all_pass = ($checks | values | all {|v| $v })
    {
        name:   $name
        status: (if $all_pass { "passed" } else { "failed" })
        checks: $checks
        error:  (if $all_pass { null } else { $error })
    }
}

# ---------------------------------------------------------------------------
# Per-tape analysis functions
# ---------------------------------------------------------------------------

def analyze_topics_panel [] {
    let initial  = (shot "tests/results/01-topics-initial.png")
    let nav_down = (shot "tests/results/01-topics-nav-down.png")
    let nav_up   = (shot "tests/results/01-topics-nav-up.png")

    let checks = {
        initial_captured:  $initial.exists
        nav_down_captured: $nav_down.exists
        nav_up_captured:   $nav_up.exists
        # /chatter topic discovered: initial screenshot should be a real TUI frame (>50 KB)
        initial_has_content: ($initial.size > 50000)
        # initial shows TUI (not direnv loading): should be at least 100 KB with color theme
        initial_is_tui: ($initial.size > 100000)
    }
    make_result "topics_panel" $checks "Topics panel did not load or /chatter not discovered"
}

def analyze_panel_navigation [] {
    let topics   = (shot "tests/results/02-topics-panel.png")
    let services = (shot "tests/results/02-services-panel.png")
    let nodes    = (shot "tests/results/02-nodes-panel.png")
    let measure  = (shot "tests/results/02-measure-panel.png")
    let tab_back = (shot "tests/results/02-tab-back-to-topics.png")

    let checks = {
        topics_captured:   $topics.exists
        services_captured: $services.exists
        nodes_captured:    $nodes.exists
        measure_captured:  $measure.exists
        tab_back_captured: $tab_back.exists
        # Each panel renders differently (panel header + content changes)
        topics_vs_services: (sizes_differ "tests/results/02-topics-panel.png" "tests/results/02-services-panel.png")
        services_vs_nodes:  (sizes_differ "tests/results/02-services-panel.png" "tests/results/02-nodes-panel.png")
        nodes_vs_measure:   (sizes_differ "tests/results/02-nodes-panel.png" "tests/results/02-measure-panel.png")
    }
    make_result "panel_navigation" $checks "Panel switching (1/2/3/4/Tab) did not change display"
}

def analyze_topic_detail [] {
    let list_pane    = (shot "tests/results/03-list-pane.png")
    let detail_pane  = (shot "tests/results/03-detail-pane.png")
    let back_to_list = (shot "tests/results/03-back-to-list.png")
    let via_enter    = (shot "tests/results/03-detail-via-enter.png")

    let checks = {
        list_captured:      $list_pane.exists
        detail_captured:    $detail_pane.exists
        back_captured:      $back_to_list.exists
        enter_captured:     $via_enter.exists
        # l moves focus to detail pane: content should differ from list pane
        l_key_changes_pane: (sizes_differ "tests/results/03-list-pane.png" "tests/results/03-detail-pane.png")
        # h returns: back-to-list should look like list-pane
        h_key_returns:      (sizes_differ "tests/results/03-detail-pane.png" "tests/results/03-back-to-list.png")
        # Enter reaches detail same as l
        enter_shows_detail: (sizes_differ "tests/results/03-list-pane.png" "tests/results/03-detail-via-enter.png")
    }
    make_result "topic_detail" $checks "l/h keys did not switch between list and detail panes"
}

def analyze_filter [] {
    let before   = (shot "tests/results/04-filter-before.png")
    let active   = (shot "tests/results/04-filter-active.png")
    let matched  = (shot "tests/results/04-filter-match.png")
    let cleared  = (shot "tests/results/04-filter-cleared.png")
    let exited   = (shot "tests/results/04-filter-exited.png")

    let checks = {
        before_captured:  $before.exists
        active_captured:  $active.exists
        matched_captured: $matched.exists
        cleared_captured: $cleared.exists
        exited_captured:  $exited.exists
        # / activates filter mode (status bar changes)
        slash_changes_display: (sizes_differ "tests/results/04-filter-before.png" "tests/results/04-filter-active.png")
        # Typing "chatter" narrows the list (fewer rows = different size)
        typing_filters_list: (sizes_differ "tests/results/04-filter-before.png" "tests/results/04-filter-match.png")
        # Ctrl+U clears text (list expands again)
        ctrl_u_clears: (sizes_differ "tests/results/04-filter-match.png" "tests/results/04-filter-cleared.png")
    }
    make_result "filter_mode" $checks "Filter mode (/) did not narrow the topic list"
}

def analyze_help_overlay [] {
    let off       = (shot "tests/results/05-help-off.png")
    let on        = (shot "tests/results/05-help-on.png")
    let dismissed = (shot "tests/results/05-help-dismissed.png")
    let escaped   = (shot "tests/results/05-help-escape.png")

    let checks = {
        off_captured:       $off.exists
        on_captured:        $on.exists
        dismissed_captured: $dismissed.exists
        escaped_captured:   $escaped.exists
        # ? shows overlay (different render = different size)
        question_shows_overlay: (sizes_differ "tests/results/05-help-off.png" "tests/results/05-help-on.png")
        # second ? dismisses overlay
        second_question_dismisses: (sizes_differ "tests/results/05-help-on.png" "tests/results/05-help-dismissed.png")
        # Escape also dismisses
        escape_dismisses: (sizes_differ "tests/results/05-help-on.png" "tests/results/05-help-escape.png")
    }
    make_result "help_overlay" $checks "? key did not show/dismiss help overlay"
}

def analyze_rate_check [] {
    let before    = (shot "tests/results/06-rate-before.png")
    let measuring = (shot "tests/results/06-rate-measuring.png")
    let done      = (shot "tests/results/06-rate-done.png")

    let checks = {
        before_captured:    $before.exists
        measuring_captured: $measuring.exists
        done_captured:      $done.exists
        # After measurement completes, rate value appears in topic list (list content changes)
        # Note: measuring vs before sizes are similar because status text change has minimal PNG impact
        rate_shown_after: (sizes_differ "tests/results/06-rate-before.png" "tests/results/06-rate-done.png")
    }
    make_result "rate_check" $checks "r key rate measurement did not update /chatter rate display"
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main [] {
    cd $WORKTREE

    # --- Preflight ---
    for tool in ["vhs", "zenohd", "ttyd"] {
        if (which $tool | is-empty) {
            error make { msg: $"Required tool not found: ($tool). Run `nix develop` first." }
        }
    }

    # Kill orphaned processes from previous runs
    do -i { ^pkill zenohd }
    do -i { ^pkill -9 ttyd }
    do -i { ^pkill -9 -f chromium }
    sleep 2sec

    # --- Build ---
    print "Building ros-z-console..."
    cargo build -p ros-z-console

    print "Building examples (z_pubsub, z_srvcli)..."
    cargo build --example z_pubsub
    cargo build --example z_srvcli

    mkdir tests/results
    mkdir _tmp

    # --- Start background processes ---
    print "Starting zenohd..."
    let zenohd_pid = (
        ^bash -c $"zenohd > _tmp/zenohd-test.log 2>&1 & echo $!"
        | str trim | into int
    )
    sleep 1sec

    print "Starting z_pubsub talker (/chatter publisher)..."
    let talker_pid = (
        ^bash -c $"./target/debug/examples/z_pubsub --role talker > _tmp/talker-test.log 2>&1 & echo $!"
        | str trim | into int
    )

    print "Starting z_pubsub listener (/chatter subscriber)..."
    let listener_pid = (
        ^bash -c $"./target/debug/examples/z_pubsub --role listener > _tmp/listener-test.log 2>&1 & echo $!"
        | str trim | into int
    )

    print "Starting z_srvcli server (AddTwoInts service)..."
    let srvcli_pid = (
        ^bash -c $"./target/debug/examples/z_srvcli > _tmp/srvcli-test.log 2>&1 & echo $!"
        | str trim | into int
    )
    sleep 3sec

    print $"PIDs: zenohd=($zenohd_pid) talker=($talker_pid) listener=($listener_pid) srvcli=($srvcli_pid)"

    # --- Warmup ---
    let all_tapes = (ls tests/tapes/*.tape | get name | sort)

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

    # --- Run tapes ---
    let tapes = ($all_tapes | skip 1)

    let tape_results = ($tapes | each {|tape|
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
            print $"  ✅ ($tape | path basename)"
            do -i { ^pkill -9 ttyd }
            do -i { ^pkill -9 -f chromium }
            sleep 5sec
        } else {
            print $"  ❌ ($tape | path basename)"
        }
        { tape: ($tape | path basename), success: $success }
    })

    # --- Analyze ---
    print "\nAnalyzing screenshots..."
    let tests = [
        (analyze_topics_panel)
        (analyze_panel_navigation)
        (analyze_topic_detail)
        (analyze_filter)
        (analyze_help_overlay)
        (analyze_rate_check)
    ]

    for t in $tests {
        let icon = (if $t.status == "passed" { "✅" } else { "❌" })
        print $"  ($icon) ($t.name)"
        if $t.status == "failed" {
            print $"     error: ($t.error)"
            let failing = ($t.checks | transpose key val | where { not $in.val })
            for f in $failing {
                print $"     - ($f.key): false"
            }
        }
    }

    # --- Cleanup ---
    print "\nCleaning up..."
    do -i { ^kill $talker_pid }
    do -i { ^kill $listener_pid }
    do -i { ^kill $srvcli_pid }
    do -i { ^kill $zenohd_pid }
    do -i { ^pkill ttyd }
    do -i { ^pkill -f chromium }
    sleep 500ms

    # --- Report ---
    let passed = ($tests | where status == "passed" | length)
    let failed = ($tests | where status == "failed" | length)

    let report = {
        timestamp:    (date now | format date '%Y-%m-%dT%H:%M:%S')
        total:        ($tests | length)
        passed:       $passed
        failed:       $failed
        tape_results: $tape_results
        tests:        $tests
    }

    let ts = (date now | format date '%Y%m%d-%H%M%S')
    $report | to json | save $"tests/results/test-run-($ts).json"
    $report | to json | save --force "tests/results/latest.json"

    print $"\n=== TUI Test Results: ($passed)/($passed + $failed) passed ==="
    print $"Report: tests/results/latest.json"
    ls tests/results/*.png | select name size | print

    if $failed > 0 {
        exit 1
    }
}
