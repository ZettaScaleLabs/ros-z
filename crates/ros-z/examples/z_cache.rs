//! # ZCache demo
//!
//! Demonstrates both stamp strategies for [`ZCache`](ros_z::cache::ZCache):
//!
//! - **zenoh** (default) — Zenoh transport timestamp, zero-config
//! - **app** — user-supplied extractor reading the message payload as seconds
//!
//! ## Run two terminals
//!
//! Terminal 1 — publisher:
//! ```text
//! cargo run --example z_cache -- --role talker
//! ```
//!
//! Terminal 2 — cache consumer (zenoh stamp, default):
//! ```text
//! cargo run --example z_cache -- --role cache
//! ```
//!
//! Or with application-level stamp extraction:
//! ```text
//! cargo run --example z_cache -- --role cache --stamp app
//! ```

use std::time::{Duration, SystemTime};

use clap::{Parser, ValueEnum};
use ros_z::{Builder, Result, context::ZContextBuilder};
use ros_z_msgs::std_msgs::String as RosString;

#[derive(Debug, Clone, Copy, ValueEnum)]
enum Role {
    /// Continuously publish messages on the topic
    Talker,
    /// Create a cache and periodically query it
    Cache,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum StampMode {
    /// Index by Zenoh transport timestamp (default, zero-config)
    Zenoh,
    /// Index by parsing message payload as seconds since UNIX_EPOCH
    App,
}

#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, value_enum, default_value = "cache")]
    role: Role,

    #[arg(short, long, default_value = "/cache_demo")]
    topic: String,

    #[arg(short, long, default_value = "peer")]
    mode: String,

    #[arg(short, long)]
    endpoint: Option<String>,

    /// Stamp strategy to use when role=cache
    #[arg(long, value_enum, default_value = "zenoh")]
    stamp: StampMode,

    /// Cache capacity (max messages retained)
    #[arg(short, long, default_value = "20")]
    capacity: usize,

    /// Query window in milliseconds (only used with --stamp zenoh)
    #[arg(short, long, default_value = "500")]
    window_ms: u64,

    /// Exit after this many query/publish iterations (0 = run forever)
    #[arg(long, default_value = "0")]
    count: usize,
}

#[tokio::main]
async fn main() -> Result<()> {
    zenoh::init_log_from_env_or("error");
    let args = Args::parse();

    let ctx = match args.endpoint {
        Some(ref e) => ZContextBuilder::default()
            .with_mode(args.mode.clone())
            .with_connect_endpoints([e.as_str()])
            .build()?,
        None => ZContextBuilder::default()
            .with_mode(args.mode.clone())
            .build()?,
    };

    match args.role {
        Role::Talker => run_talker(ctx, args.topic, args.count).await,
        Role::Cache => match args.stamp {
            StampMode::Zenoh => {
                run_cache_zenoh(ctx, args.topic, args.capacity, args.window_ms, args.count).await
            }
            StampMode::App => run_cache_app(ctx, args.topic, args.capacity, args.count).await,
        },
    }
}

// ---------------------------------------------------------------------------
// Talker: publishes a new message every 100 ms carrying an incrementing counter
// ---------------------------------------------------------------------------

async fn run_talker(ctx: ros_z::context::ZContext, topic: String, count: usize) -> Result<()> {
    let node = ctx.create_node("cache_talker").build()?;
    let publisher = node.create_pub::<RosString>(&topic).build()?;

    println!("[talker] publishing on '{}' every 100 ms", topic);

    let mut seq: u64 = 0;
    loop {
        let msg = RosString {
            data: format!("msg-{seq}"),
        };
        publisher.async_publish(&msg).await?;
        println!("[talker] sent: {}", msg.data);
        tokio::time::sleep(Duration::from_millis(100)).await;
        seq += 1;
        if count > 0 && seq as usize >= count {
            break;
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Cache consumer — ZenohStamp (default, zero-config)
//
// Queries a sliding [now-window_ms, now] window every 500 ms.
// ---------------------------------------------------------------------------

async fn run_cache_zenoh(
    ctx: ros_z::context::ZContext,
    topic: String,
    capacity: usize,
    window_ms: u64,
    count: usize,
) -> Result<()> {
    let node = ctx.create_node("cache_consumer").build()?;

    // Build cache with default Zenoh transport timestamp.
    // No extractor required — works for any message type.
    let cache = node.create_cache::<RosString>(&topic, capacity).build()?;

    println!(
        "[cache/zenoh] subscribed to '{}', capacity={}, window={}ms",
        topic, capacity, window_ms
    );

    // Give the subscription time to connect before querying
    tokio::time::sleep(Duration::from_millis(300)).await;

    let window = Duration::from_millis(window_ms);
    let mut i = 0usize;
    loop {
        let now = SystemTime::now();
        let t_start = now - window;

        let msgs = cache.get_interval(t_start, now);
        let newest = cache.get_before(now);

        println!(
            "[cache/zenoh] window=[now-{}ms, now]: {} messages | newest: {}",
            window_ms,
            msgs.len(),
            newest.as_ref().map(|m| m.data.as_str()).unwrap_or("(none)"),
        );

        i += 1;
        if count > 0 && i >= count {
            break;
        }
        tokio::time::sleep(Duration::from_millis(500)).await;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Cache consumer — ExtractorStamp (application-level timestamp)
//
// Treats the message payload as "<seconds>" and uses that as the stamp.
// Demonstrates sensor fusion: align by logical time, not transport time.
// ---------------------------------------------------------------------------

async fn run_cache_app(
    ctx: ros_z::context::ZContext,
    topic: String,
    capacity: usize,
    count: usize,
) -> Result<()> {
    let node = ctx.create_node("cache_consumer_app").build()?;

    // Build cache with application-level extractor.
    // The closure reads the message payload and returns the logical timestamp.
    // For real sensor data this would read header.stamp.
    let cache = node
        .create_cache::<RosString>(&topic, capacity)
        .with_stamp(|msg: &RosString| {
            // Parse payload as seconds since UNIX_EPOCH (demo convention).
            // In real code: SystemTime::UNIX_EPOCH + Duration::new(msg.header.stamp.sec as u64, msg.header.stamp.nanosec)
            let secs: u64 = msg
                .data
                .split('-')
                .next_back()
                .and_then(|s| s.parse().ok())
                .unwrap_or(0);
            SystemTime::UNIX_EPOCH + Duration::from_secs(secs)
        })
        .build()?;

    println!(
        "[cache/app] subscribed to '{}', capacity={} (app-level stamp)",
        topic, capacity,
    );

    tokio::time::sleep(Duration::from_millis(300)).await;

    let mut i = 0usize;
    loop {
        println!(
            "[cache/app] len={} | oldest={:?} | newest={:?}",
            cache.len(),
            cache.oldest_stamp(),
            cache.newest_stamp(),
        );

        // get_nearest: find the message whose logical timestamp is closest to
        // 5 seconds after UNIX_EPOCH (arbitrary alignment target for the demo).
        let target = SystemTime::UNIX_EPOCH + Duration::from_secs(5);
        let nearest = cache.get_nearest(target);
        println!(
            "[cache/app] nearest to t=5s: {}",
            nearest
                .as_ref()
                .map(|m| m.data.as_str())
                .unwrap_or("(none)"),
        );

        i += 1;
        if count > 0 && i >= count {
            break;
        }
        tokio::time::sleep(Duration::from_millis(500)).await;
    }
    Ok(())
}
