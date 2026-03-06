//! Smoke tests for the `z_cache` example.
//!
//! Each test runs the example with `--count N` so it exits cleanly after N
//! iterations rather than looping forever. This lets us use `Command::output()`
//! and assert on stdout without killing the process.
//!
//! Run with:
//! ```bash
//! cargo test --test z_cache_example -- --ignored
//! ```

use std::process::Command;

/// Build the z_cache example and return its binary path.
fn build_z_cache() -> std::path::PathBuf {
    // CARGO_MANIFEST_DIR is crates/ros-z-tests — go up two levels to workspace root.
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_else(|_| ".".into());
    let workspace_root = std::path::Path::new(&manifest_dir)
        .parent()
        .and_then(|p| p.parent())
        .expect("could not find workspace root")
        .to_path_buf();

    let output = Command::new("cargo")
        .args([
            "build",
            "--package",
            "ros-z",
            "--example",
            "z_cache",
            "--features",
            "jazzy",
        ])
        .current_dir(&workspace_root)
        .output()
        .expect("failed to invoke cargo build");

    assert!(
        output.status.success(),
        "cargo build --example z_cache failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );

    workspace_root.join("target/debug/examples/z_cache")
}

#[test]
#[ignore = "integration smoke test — builds and runs z_cache; invoke with --ignored"]
fn z_cache_cache_role_zenoh_stamp() {
    let binary = build_z_cache();

    // --count 1: subscribe, wait 300 ms for connections, run one query cycle, exit.
    let output = Command::new(&binary)
        .args(["--role", "cache", "--count", "1"])
        .output()
        .expect("failed to run z_cache");

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "z_cache --role cache exited with {:?}\nstderr:\n{stderr}",
        output.status.code()
    );
    assert!(
        stdout.contains("[cache/zenoh] subscribed to"),
        "missing startup banner\nstdout:\n{stdout}"
    );
    assert!(
        stdout.contains("[cache/zenoh] window="),
        "missing query output\nstdout:\n{stdout}"
    );
}

#[test]
#[ignore = "integration smoke test — builds and runs z_cache; invoke with --ignored"]
fn z_cache_cache_role_app_stamp() {
    let binary = build_z_cache();

    let output = Command::new(&binary)
        .args(["--role", "cache", "--stamp", "app", "--count", "1"])
        .output()
        .expect("failed to run z_cache");

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "z_cache --role cache --stamp app exited with {:?}\nstderr:\n{stderr}",
        output.status.code()
    );
    assert!(
        stdout.contains("[cache/app] subscribed to"),
        "missing startup banner\nstdout:\n{stdout}"
    );
    assert!(
        stdout.contains("[cache/app] len="),
        "missing query output\nstdout:\n{stdout}"
    );
    assert!(
        stdout.contains("[cache/app] nearest to t=5s:"),
        "missing get_nearest output\nstdout:\n{stdout}"
    );
}

#[test]
#[ignore = "integration smoke test — builds and runs z_cache; invoke with --ignored"]
fn z_cache_talker_role() {
    let binary = build_z_cache();

    // --count 3: publish 3 messages then exit.
    let output = Command::new(&binary)
        .args(["--role", "talker", "--count", "3"])
        .output()
        .expect("failed to run z_cache");

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "z_cache --role talker exited with {:?}\nstderr:\n{stderr}",
        output.status.code()
    );
    assert!(
        stdout.contains("[talker] publishing on"),
        "missing startup banner\nstdout:\n{stdout}"
    );
    assert!(
        stdout.contains("[talker] sent: msg-0"),
        "missing first published message\nstdout:\n{stdout}"
    );
    assert!(
        stdout.contains("[talker] sent: msg-2"),
        "expected 3 messages (msg-0..msg-2)\nstdout:\n{stdout}"
    );
}
