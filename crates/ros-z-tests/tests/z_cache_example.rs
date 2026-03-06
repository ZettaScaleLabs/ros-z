//! Smoke tests for the `z_cache` example.
//!
//! The example is included directly as a module via `#[path]` so the async
//! helper functions can be called in-process from `#[tokio::test]` without
//! spawning a subprocess.

#![cfg(feature = "ros-msgs")]

mod common;

#[allow(dead_code, unused_imports)]
#[path = "../../ros-z/examples/z_cache.rs"]
mod z_cache_example;

use common::*;
use ros_z::{Builder, context::ZContextBuilder};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn make_ctx(endpoint: &str) -> ros_z::context::ZContext {
    ZContextBuilder::default()
        .with_mode(String::from("client"))
        .with_connect_endpoints([endpoint])
        .build()
        .expect("context")
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// Cache role with ZenohStamp: subscribe, wait for connections, run one query
/// cycle, exit cleanly.
#[tokio::test(flavor = "multi_thread")]
async fn z_cache_cache_role_zenoh_stamp() {
    let router = TestRouter::new();
    let ctx = make_ctx(&router.endpoint());
    z_cache_example::run_cache_zenoh(ctx, "/smoke/cache_zenoh".into(), 20, 500, 1)
        .await
        .expect("run_cache_zenoh returned Err");
}

/// Cache role with ExtractorStamp: subscribe, run one query cycle, exit cleanly.
#[tokio::test(flavor = "multi_thread")]
async fn z_cache_cache_role_app_stamp() {
    let router = TestRouter::new();
    let ctx = make_ctx(&router.endpoint());
    z_cache_example::run_cache_app(ctx, "/smoke/cache_app".into(), 20, 1)
        .await
        .expect("run_cache_app returned Err");
}

/// Talker role: publish 3 messages then exit cleanly.
#[tokio::test(flavor = "multi_thread")]
async fn z_cache_talker_role() {
    let router = TestRouter::new();
    let ctx = make_ctx(&router.endpoint());
    z_cache_example::run_talker(ctx, "/smoke/talker".into(), 3)
        .await
        .expect("run_talker returned Err");
}
