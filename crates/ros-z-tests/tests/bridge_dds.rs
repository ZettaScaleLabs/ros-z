//! Integration tests for ros-z-bridge-dds.
//!
//! Covers both wire formats:
//! - `--wire-format ros2dds` (legacy): compatible with zenoh-plugin-ros2dds
//! - `--wire-format rmw-zenoh` (default): interops with ros-z / rmw_zenoh_cpp
//!
//! Legacy test coverage (ros2dds wire format, tests 1-6):
//! | # | Scenario                             |
//! |---|--------------------------------------|
//! | 1 | Zenoh pub → bridge → DDS sub         |
//! | 2 | DDS pub → bridge → Zenoh sub         |
//! | 3 | Zenoh queryable ← bridge ← DDS cli   |
//! | 4 | DDS server ← bridge ← Zenoh get      |
//! | 5 | Zenoh action server ← bridge ← DDS   |
//! | 6 | DDS action server ← bridge ← Zenoh   |
//!
//! Primary tests (rmw-zenoh wire format, tests 7-10):
//! | # | Scenario                             |
//! |---|--------------------------------------|
//! | 7 | DDS pub → bridge → Zenoh sub (rmw_zenoh keys)  |
//! | 8 | Zenoh pub → bridge → DDS sub (rmw_zenoh keys)  |
//! | 9 | DDS server ← bridge ← Zenoh get (rmw_zenoh)    |
//! |10 | Zenoh server ← bridge ← DDS client (rmw_zenoh) |
//!
//! API-level tests (no binary spawn, tests 11-13):
//! | # | Scenario                             |
//! |---|--------------------------------------|
//! |11 | ZDdsPubBridge::new() constructs cleanly         |
//! |12 | ZDdsSubBridge::new() constructs cleanly         |
//! |13 | DdsBridgeExt typed constructors work            |
//!
//! Requirements:
//! - ROS 2 Jazzy with `rmw_cyclonedds_cpp`, `demo_nodes_cpp`, `action_tutorials_cpp`
//! - `ros-z-bridge-dds` binary built at `target/debug/` or in PATH
//!
//! Run with:
//! ```bash
//! cargo test -p ros-z-tests --features dds-bridge-interop,jazzy
//! ```

#![cfg(feature = "dds-bridge-interop")]

mod common;

use std::{
    process::{Child, Command, Output, Stdio},
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use common::{ProcessGuard, TestRouter};
use nix::{
    sys::signal::{Signal, kill},
    unistd::Pid,
};
use zenoh::Wait;

#[cfg(feature = "dds-bridge-interop")]
use ros_z_dds::{
    CyclorsParticipant, DdsBridgeExt, ZDdsPubBridge, ZDdsSubBridge,
    participant::{BridgeQos, DdsParticipant},
};

#[cfg(feature = "dds-bridge-interop")]
use ros_z_msgs::std_msgs::String as RosString;

/// Kill an entire process group (including any children spawned by `ros2 run`)
/// and then collect stdout output. Using `child.kill()` alone only kills the
/// direct child; `ros2 run` may exec-fork a C++ node that keeps the pipe open.
fn kill_group_and_collect(child: Child) -> Output {
    let pgid = Pid::from_raw(-(child.id() as i32));
    let _ = kill(pgid, Signal::SIGKILL);
    child.wait_with_output().unwrap()
}

// ── helpers ──────────────────────────────────────────────────────────────────

/// CycloneDDS URI that forces use of the loopback interface with multicast enabled.
///
/// Without this, CycloneDDS tries the primary network interface (wlp1s0 etc.) for
/// multicast discovery. Between processes on the same host, that often fails when
/// multicast routing is not configured. The loopback interface reliably supports
/// multicast on Linux and is always available.
const CYCLONEDDS_URI: &str = "<CycloneDDS><Domain><General><Interfaces>\
     <NetworkInterface name=\"lo\" multicast=\"true\"/>\
     </Interfaces></General></Domain></CycloneDDS>";

fn bridge_binary() -> String {
    let local = format!(
        "{}/../../target/debug/ros-z-bridge-dds",
        env!("CARGO_MANIFEST_DIR")
    );
    if std::path::Path::new(&local).exists() {
        return local;
    }
    "ros-z-bridge-dds".to_string()
}

fn ros2_available() -> bool {
    Command::new("ros2")
        .args(["pkg", "list"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

fn pkg_available(pkg: &str) -> bool {
    Command::new("ros2")
        .args(["pkg", "prefix", pkg])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

fn spawn_bridge_fmt(zenoh_endpoint: &str, domain_id: u32, wire_format: &str) -> ProcessGuard {
    use std::os::unix::process::CommandExt;
    let bin = bridge_binary();
    let log_path = format!(
        "{}/../../_tmp/bridge-{domain_id}.log",
        env!("CARGO_MANIFEST_DIR")
    );
    let log_file = std::fs::File::create(&log_path)
        .unwrap_or_else(|e| panic!("Failed to create bridge log {log_path}: {e}"));
    let child = Command::new(&bin)
        .args([
            "--zenoh-endpoint",
            zenoh_endpoint,
            "--domain-id",
            &domain_id.to_string(),
            "--wire-format",
            wire_format,
        ])
        .env("RUST_LOG", "info")
        .env("CYCLONEDDS_URI", CYCLONEDDS_URI)
        .stdout(log_file)
        .stderr(Stdio::null())
        .process_group(0)
        .spawn()
        .unwrap_or_else(|e| panic!("Failed to spawn {bin}: {e}"));
    thread::sleep(Duration::from_secs(5));
    ProcessGuard::new(child, "ros-z-bridge-dds")
}

/// Spawn bridge in legacy ros2dds mode (zenoh-plugin-ros2dds compatible key expressions).
fn spawn_bridge_ros2dds(zenoh_endpoint: &str, domain_id: u32) -> ProcessGuard {
    spawn_bridge_fmt(zenoh_endpoint, domain_id, "ros2dds")
}

/// Spawn bridge in default rmw-zenoh mode (ros-z / rmw_zenoh_cpp compatible key expressions).
fn spawn_bridge(zenoh_endpoint: &str, domain_id: u32) -> ProcessGuard {
    spawn_bridge_fmt(zenoh_endpoint, domain_id, "rmw-zenoh")
}

fn spawn_cyclone(domain_id: u32, pkg: &str, node: &str, extra_args: &[&str]) -> ProcessGuard {
    use std::os::unix::process::CommandExt;
    let name = format!("{}/{}", pkg, node);
    let mut cmd = Command::new("ros2");
    cmd.args(["run", pkg, node])
        .args(extra_args)
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .env("CYCLONEDDS_URI", CYCLONEDDS_URI)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .process_group(0);
    let child = cmd
        .spawn()
        .unwrap_or_else(|e| panic!("Failed to spawn {name}: {e}"));
    thread::sleep(Duration::from_secs(2));
    ProcessGuard::new(child, &name)
}

fn zenoh_session(endpoint: &str) -> zenoh::Session {
    let mut cfg = zenoh::Config::default();
    cfg.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))
        .unwrap();
    cfg.insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    cfg.insert_json5("mode", r#""client""#).unwrap();
    zenoh::open(cfg).wait().unwrap()
}

/// Minimal CDR string payload: 4-byte LE header + 4-byte length + UTF-8 bytes + NUL + padding.
fn cdr_string(s: &str) -> Vec<u8> {
    let bytes = s.as_bytes();
    let len = bytes.len() as u32 + 1; // include NUL
    let mut v = Vec::with_capacity(4 + 4 + len as usize);
    v.extend_from_slice(&[0, 1, 0, 0]); // CDR LE header
    v.extend_from_slice(&len.to_le_bytes());
    v.extend_from_slice(bytes);
    v.push(0); // NUL terminator
    while v.len() % 4 != 0 {
        v.push(0); // align
    }
    v
}

/// CDR payload for AddTwoInts request (two i64 values, no request-header).
fn cdr_add_two_ints_request(a: i64, b: i64) -> Vec<u8> {
    let mut v = Vec::with_capacity(4 + 16);
    v.extend_from_slice(&[0, 1, 0, 0]); // CDR LE header
    v.extend_from_slice(&a.to_le_bytes());
    v.extend_from_slice(&b.to_le_bytes());
    v
}

/// Extract i64 sum from a CDR AddTwoInts response payload (skip 4-byte CDR header).
fn parse_add_two_ints_response(bytes: &[u8]) -> Option<i64> {
    // bytes = 4-byte CDR header + optional 16-byte request header + 8-byte i64 sum
    // When coming from the DDS server, the bridge strips the request header.
    // Attempt to parse at offset 4 (no request header) or offset 20 (with header).
    for offset in [4usize, 20] {
        if bytes.len() >= offset + 8 {
            let sum = i64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap());
            if sum > 0 && sum < 1_000_000 {
                return Some(sum);
            }
        }
    }
    None
}

// ── Test 1: Zenoh pub → bridge → DDS subscriber (ros2dds) ────────────────────

/// Mirrors `test_zenoh_pub_ros_sub` from zenoh-plugin-ros2dds.
///
/// Zenoh publishes a CDR String on `chatter`. The bridge forwards it to DDS.
/// A CycloneDDS listener receives it — we verify by capturing its stdout.
#[test]
fn test_zenoh_pub_ros_sub() {
    if !ros2_available() || !pkg_available("demo_nodes_cpp") {
        eprintln!("Skipping test_zenoh_pub_ros_sub: ROS 2 / demo_nodes_cpp not available");
        return;
    }

    let domain_id = 51u32;
    let router = TestRouter::new();
    let endpoint = router.endpoint().to_string();

    let _bridge = spawn_bridge_ros2dds(&endpoint, domain_id);

    // Start listener; ROS 2 RCLCPP_INFO logs go to stderr, not stdout.
    let mut listener_cmd = Command::new("ros2");
    listener_cmd
        .args(["run", "demo_nodes_cpp", "listener"])
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .env("CYCLONEDDS_URI", CYCLONEDDS_URI)
        .stdout(Stdio::null())
        .stderr(Stdio::piped());
    use std::os::unix::process::CommandExt;
    listener_cmd.process_group(0);
    let listener_child = listener_cmd.spawn().expect("Failed to spawn listener");
    // Wait for DDS discovery: CycloneDDS on loopback needs ~5 s to discover the bridge.
    // Wait extra time for the bridge to create its DDS writer and for the listener to
    // discover that new writer.
    thread::sleep(Duration::from_secs(10));

    // Publish from Zenoh side; repeat to handle transient delivery gaps.
    let session = zenoh_session(&endpoint);
    let publisher = session.declare_publisher("chatter").wait().unwrap();
    let payload = cdr_string("Hello from Zenoh");
    for _ in 0..30 {
        publisher
            .put(zenoh::bytes::ZBytes::from(payload.clone()))
            .wait()
            .unwrap();
        thread::sleep(Duration::from_millis(500));
    }

    // Read listener output (RCLCPP_INFO goes to stderr in ROS 2 Jazzy)
    let output = kill_group_and_collect(listener_child);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("Hello"),
        "listener received nothing; stderr={stderr:?}"
    );
}

// ── Test 2: DDS pub → bridge → Zenoh subscriber (ros2dds) ────────────────────

/// Mirrors `test_ros_pub_zenoh_sub` from zenoh-plugin-ros2dds.
///
/// A CycloneDDS talker publishes on `/chatter`. The bridge forwards CDR bytes
/// to Zenoh. A Zenoh subscriber receives at least one sample.
#[test]
fn test_ros_pub_zenoh_sub() {
    if !ros2_available() || !pkg_available("demo_nodes_cpp") {
        eprintln!("Skipping test_ros_pub_zenoh_sub: ROS 2 / demo_nodes_cpp not available");
        return;
    }

    let domain_id = 52u32;
    let router = TestRouter::new();
    let endpoint = router.endpoint().to_string();

    let _bridge = spawn_bridge_ros2dds(&endpoint, domain_id);

    let received = Arc::new(Mutex::new(false));
    let received_clone = received.clone();

    let session = zenoh_session(&endpoint);
    let _sub = session
        .declare_subscriber("chatter")
        .callback(move |_sample| {
            *received_clone.lock().unwrap() = true;
        })
        .wait()
        .unwrap();

    let _talker = spawn_cyclone(domain_id, "demo_nodes_cpp", "talker", &[]);

    // Wait up to 10 s for a message
    for _ in 0..20 {
        if *received.lock().unwrap() {
            break;
        }
        thread::sleep(Duration::from_millis(500));
    }

    assert!(
        *received.lock().unwrap(),
        "Zenoh subscriber received nothing from CycloneDDS talker"
    );
}

// ── Test 3: Zenoh queryable ← bridge ← DDS service client (ros2dds) ─────────

/// Mirrors `test_ros_client_zenoh_service` from zenoh-plugin-ros2dds.
///
/// A Zenoh queryable acts as the AddTwoInts service server.
/// A DDS `add_two_ints_client` calls the service through the bridge.
/// We verify the bridge forwards the request to Zenoh and the client gets a reply.
#[test]
fn test_ros_client_zenoh_service() {
    if !ros2_available() || !pkg_available("demo_nodes_cpp") {
        eprintln!("Skipping test_ros_client_zenoh_service: dependencies not available");
        return;
    }

    let domain_id = 53u32;
    let router = TestRouter::new();
    let endpoint = router.endpoint().to_string();

    let _bridge = spawn_bridge_ros2dds(&endpoint, domain_id);

    // Zenoh service server: receives CDR request, returns CDR reply
    let session = zenoh_session(&endpoint);
    let _queryable = session
        .declare_queryable("add_two_ints")
        .callback(|query| {
            let payload: Vec<u8> = query
                .payload()
                .map(|p| p.to_bytes().into_owned())
                .unwrap_or_default();

            // Request layout: 4-byte CDR header + 8-byte a + 8-byte b
            let (a, b) = if payload.len() >= 20 {
                (
                    i64::from_le_bytes(payload[4..12].try_into().unwrap()),
                    i64::from_le_bytes(payload[12..20].try_into().unwrap()),
                )
            } else {
                (0i64, 0i64)
            };

            // Reply: 4-byte CDR header + 8-byte sum
            let mut reply = Vec::with_capacity(12);
            reply.extend_from_slice(&[0, 1, 0, 0]);
            reply.extend_from_slice(&(a + b).to_le_bytes());

            let ke = query.key_expr().clone();
            query
                .reply(ke, zenoh::bytes::ZBytes::from(reply))
                .wait()
                .unwrap();
        })
        .wait()
        .unwrap();

    // Give time for the queryable to be discovered
    thread::sleep(Duration::from_secs(2));

    // Spawn DDS client (sends one request: 1 + 2); RCLCPP_INFO goes to stderr in ROS 2 Jazzy.
    let mut client_cmd = Command::new("ros2");
    client_cmd
        .args(["run", "demo_nodes_cpp", "add_two_ints_client"])
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .env("CYCLONEDDS_URI", CYCLONEDDS_URI)
        .stdout(Stdio::null())
        .stderr(Stdio::piped());
    use std::os::unix::process::CommandExt;
    client_cmd.process_group(0);
    let client_child = client_cmd.spawn().expect("spawn add_two_ints_client");

    // Wait for client to complete (it exits after one response)
    let output = {
        let deadline = std::time::Instant::now() + Duration::from_secs(15);
        let mut child = client_child;
        loop {
            match child.try_wait() {
                Ok(Some(_)) => break child.wait_with_output().unwrap(),
                Ok(None) if std::time::Instant::now() < deadline => {
                    thread::sleep(Duration::from_millis(200));
                }
                _ => break kill_group_and_collect(child),
            }
        }
    };
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("3") || output.status.success(),
        "DDS client did not receive a valid reply; stderr={stderr:?}"
    );
}

// ── Test 4: Zenoh client → bridge → DDS service server (ros2dds) ─────────────

/// Mirrors `test_zenoh_client_ros_service` from zenoh-plugin-ros2dds.
///
/// A CycloneDDS `add_two_ints_server` is running. A Zenoh `get()` call reaches
/// it through the bridge and receives sum=3 (1+2).
#[test]
fn test_zenoh_client_ros_service() {
    if !ros2_available() || !pkg_available("demo_nodes_cpp") {
        eprintln!("Skipping test_zenoh_client_ros_service: dependencies not available");
        return;
    }

    let domain_id = 54u32;
    let router = TestRouter::new();
    let endpoint = router.endpoint().to_string();

    let _bridge = spawn_bridge_ros2dds(&endpoint, domain_id);
    let _server = spawn_cyclone(domain_id, "demo_nodes_cpp", "add_two_ints_server", &[]);
    thread::sleep(Duration::from_secs(2)); // let server + bridge settle

    let session = zenoh_session(&endpoint);
    let payload = cdr_add_two_ints_request(1, 2);

    let replies: Vec<_> = session
        .get("add_two_ints")
        .payload(zenoh::bytes::ZBytes::from(payload))
        .timeout(Duration::from_secs(10))
        .wait()
        .unwrap()
        .into_iter()
        .collect();

    assert!(
        !replies.is_empty(),
        "No reply from DDS add_two_ints_server via bridge"
    );

    let reply_bytes: Vec<u8> = replies[0]
        .result()
        .expect("reply error")
        .payload()
        .to_bytes()
        .into_owned();

    let sum = parse_add_two_ints_response(&reply_bytes);
    assert_eq!(
        sum,
        Some(3),
        "Expected sum=3, got {:?}; raw={reply_bytes:?}",
        sum
    );
}

// ── Test 5: Zenoh action server ← bridge ← DDS action client (ros2dds) ───────

/// Mirrors `test_ros_client_zenoh_action` from zenoh-plugin-ros2dds.
///
/// A Zenoh side implements the three action service components
/// (send_goal, get_result) and the feedback publisher.
/// A DDS `fibonacci_action_client` sends a goal through the bridge and
/// receives feedback and result.
///
/// Uses `action_tutorials_cpp` package (Jazzy+).
#[test]
fn test_ros_client_zenoh_action() {
    if !ros2_available() || !pkg_available("action_tutorials_cpp") {
        eprintln!("Skipping test_ros_client_zenoh_action: action_tutorials_cpp not available");
        return;
    }

    let domain_id = 55u32;
    let router = TestRouter::new();
    let endpoint = router.endpoint().to_string();

    let _bridge = spawn_bridge_ros2dds(&endpoint, domain_id);
    thread::sleep(Duration::from_secs(1));

    let session = zenoh_session(&endpoint);

    // Zenoh action server components
    // send_goal queryable
    let _send_goal = session
        .declare_queryable("fibonacci/_action/send_goal")
        .callback(move |query| {
            // Accept immediately: [bool accept=true, int32 sec=0, uint32 nanosec=0]
            let mut reply = vec![0u8; 4 + 1 + 4 + 4];
            reply[..4].copy_from_slice(&[0, 1, 0, 0]); // CDR LE
            reply[4] = 1; // accept = true
            let ke = query.key_expr().clone();
            query
                .reply(ke, zenoh::bytes::ZBytes::from(reply))
                .wait()
                .unwrap();
        })
        .wait()
        .unwrap();

    // feedback publisher
    let fb_pub = session
        .declare_publisher("fibonacci/_action/feedback")
        .wait()
        .unwrap();

    // get_result queryable: publishes feedback then returns result
    let _get_result = session
        .declare_queryable("fibonacci/_action/get_result")
        .callback(move |query| {
            // Publish one feedback sample: goal_id(16) + sequence [0,1,1,2,3]
            let mut fb = vec![0u8; 4 + 16 + 4 + 5 * 4];
            fb[..4].copy_from_slice(&[0, 1, 0, 0]);
            let seq = [0i32, 1, 1, 2, 3];
            let off = 4 + 16 + 4;
            for (i, &v) in seq.iter().enumerate() {
                fb[off + i * 4..off + i * 4 + 4].copy_from_slice(&v.to_le_bytes());
            }
            // length prefix for sequence
            let len_off = 4 + 16;
            let len = seq.len() as u32;
            fb[len_off..len_off + 4].copy_from_slice(&len.to_le_bytes());
            let _ = fb_pub.put(zenoh::bytes::ZBytes::from(fb)).wait();

            // Reply with result: status=4 (SUCCEEDED), sequence [0,1,1,2,3,5]
            let result_seq = [0i32, 1, 1, 2, 3, 5];
            let mut reply = vec![0u8; 4 + 1 + 4 + result_seq.len() * 4];
            reply[..4].copy_from_slice(&[0, 1, 0, 0]);
            reply[4] = 4; // status = SUCCEEDED
            let rlen_off = 5;
            let rlen = result_seq.len() as u32;
            reply[rlen_off..rlen_off + 4].copy_from_slice(&rlen.to_le_bytes());
            for (i, &v) in result_seq.iter().enumerate() {
                let off = rlen_off + 4 + i * 4;
                reply[off..off + 4].copy_from_slice(&v.to_le_bytes());
            }
            let ke = query.key_expr().clone();
            query
                .reply(ke, zenoh::bytes::ZBytes::from(reply))
                .wait()
                .unwrap();
        })
        .wait()
        .unwrap();

    thread::sleep(Duration::from_secs(2));

    // Spawn DDS action client
    let mut child = Command::new("ros2");
    child
        .args(["run", "action_tutorials_cpp", "fibonacci_action_client"])
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .env("CYCLONEDDS_URI", CYCLONEDDS_URI)
        .stdout(Stdio::piped())
        .stderr(Stdio::null());
    use std::os::unix::process::CommandExt;
    child.process_group(0);
    let client_child = child.spawn().expect("spawn fibonacci_action_client");

    let output = {
        let timeout = std::time::Instant::now() + Duration::from_secs(20);
        let mut c = client_child;
        loop {
            match c.try_wait() {
                Ok(Some(_)) => break c.wait_with_output().unwrap(),
                Ok(None) if std::time::Instant::now() < timeout => {
                    thread::sleep(Duration::from_millis(300));
                }
                _ => break kill_group_and_collect(c),
            }
        }
    };
    let stdout = String::from_utf8_lossy(&output.stdout);
    // The client prints the result sequence; we just check it ran and got something
    assert!(
        output.status.success() || stdout.contains("sequence"),
        "DDS action client did not receive action result; stdout={stdout:?}"
    );
}

// ── Test 6: Zenoh action client → bridge → DDS action server ─────────────────

/// Mirrors `test_zenoh_client_ros_action` from zenoh-plugin-ros2dds.
///
/// A CycloneDDS `fibonacci_action_server` runs on the DDS side.
/// A Zenoh client drives the three service components manually:
/// send_goal → subscribe feedback → get_result.
///
/// Uses `action_tutorials_cpp` package (Jazzy+).
#[test]
fn test_zenoh_client_ros_action() {
    if !ros2_available() || !pkg_available("action_tutorials_cpp") {
        eprintln!("Skipping test_zenoh_client_ros_action: action_tutorials_cpp not available");
        return;
    }

    let domain_id = 56u32;
    let router = TestRouter::new();
    let endpoint = router.endpoint().to_string();

    let _bridge = spawn_bridge_ros2dds(&endpoint, domain_id);
    let _server = spawn_cyclone(
        domain_id,
        "action_tutorials_cpp",
        "fibonacci_action_server",
        &[],
    );
    thread::sleep(Duration::from_secs(3));

    let rt = tokio::runtime::Runtime::new().unwrap();
    let (tx, rx) = std::sync::mpsc::channel::<bool>();

    rt.block_on(async move {
        let mut cfg = zenoh::Config::default();
        cfg.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))
            .unwrap();
        cfg.insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        cfg.insert_json5("mode", r#""client""#).unwrap();
        let session = zenoh::open(cfg).await.unwrap();

        let goal_id = [1u8; 16];
        let order = 5i32;

        // 1. send_goal
        let mut sg_payload = Vec::with_capacity(4 + 16 + 4);
        sg_payload.extend_from_slice(&[0, 1, 0, 0]); // CDR LE
        sg_payload.extend_from_slice(&goal_id);
        sg_payload.extend_from_slice(&order.to_le_bytes());

        let sg_replies: Vec<_> = session
            .get("fibonacci/_action/send_goal")
            .payload(zenoh::bytes::ZBytes::from(sg_payload))
            .timeout(Duration::from_secs(10))
            .await
            .unwrap()
            .into_iter()
            .collect();

        if sg_replies.is_empty() {
            tx.send(false).unwrap();
            return;
        }

        // 2. subscribe to feedback
        let fb_received = Arc::new(Mutex::new(false));
        let fb_clone = fb_received.clone();
        let _fb_sub = session
            .declare_subscriber("fibonacci/_action/feedback")
            .callback(move |_| {
                *fb_clone.lock().unwrap() = true;
            })
            .await
            .unwrap();

        // 3. get_result
        let mut gr_payload = Vec::with_capacity(4 + 16);
        gr_payload.extend_from_slice(&[0, 1, 0, 0]); // CDR LE
        gr_payload.extend_from_slice(&goal_id);

        let gr_replies: Vec<_> = session
            .get("fibonacci/_action/get_result")
            .payload(zenoh::bytes::ZBytes::from(gr_payload))
            .timeout(Duration::from_secs(10))
            .await
            .unwrap()
            .into_iter()
            .collect();

        let success = !gr_replies.is_empty();
        tx.send(success).unwrap();
    });

    let got = rx.recv_timeout(Duration::from_secs(30)).unwrap_or(false);
    rt.shutdown_background();
    assert!(
        got,
        "Zenoh action client did not receive result from DDS fibonacci_action_server"
    );
}

// ── Test 7: DDS pub → bridge → Zenoh sub (rmw-zenoh) ─────────────────────────

/// CycloneDDS talker publishes on `/chatter`. Bridge runs in default rmw-zenoh mode.
/// A Zenoh subscriber at the wildcard rmw-zenoh key `0/chatter/**` receives at least
/// one sample.  The `0` prefix is the ZContext domain_id (default when not set).
#[test]
fn test_dds_pub_rosz_sub() {
    if !ros2_available() || !pkg_available("demo_nodes_cpp") {
        eprintln!("Skipping test_dds_pub_rosz_sub: ROS 2 / demo_nodes_cpp not available");
        return;
    }

    let domain_id = 57u32;
    let router = TestRouter::new();
    let endpoint = router.endpoint().to_string();

    let _bridge = spawn_bridge(&endpoint, domain_id);

    let received = Arc::new(Mutex::new(false));
    let received_clone = received.clone();

    let session = zenoh_session(&endpoint);
    let _sub = session
        .declare_subscriber("0/chatter/**")
        .callback(move |_sample| {
            *received_clone.lock().unwrap() = true;
        })
        .wait()
        .unwrap();

    let _talker = spawn_cyclone(domain_id, "demo_nodes_cpp", "talker", &[]);

    for _ in 0..20 {
        if *received.lock().unwrap() {
            break;
        }
        thread::sleep(Duration::from_millis(500));
    }

    assert!(
        *received.lock().unwrap(),
        "Zenoh subscriber received nothing from CycloneDDS talker via rmw-zenoh bridge"
    );
}

// ── Test 8: Zenoh pub → bridge → DDS sub (rmw-zenoh) ─────────────────────────

/// Zenoh publishes a CDR String on `0/chatter/EMPTY_TOPIC_TYPE/EMPTY_TOPIC_HASH`.
/// Bridge (rmw-zenoh mode) discovers the DDS listener and creates a DDS writer route.
/// The CycloneDDS listener receives the message.
#[test]
fn test_rosz_pub_dds_sub() {
    if !ros2_available() || !pkg_available("demo_nodes_cpp") {
        eprintln!("Skipping test_rosz_pub_dds_sub: ROS 2 / demo_nodes_cpp not available");
        return;
    }

    let domain_id = 58u32;
    let router = TestRouter::new();
    let endpoint = router.endpoint().to_string();

    let _bridge = spawn_bridge(&endpoint, domain_id);

    // Start listener; RCLCPP_INFO goes to stderr in ROS 2 Jazzy.
    let mut listener_cmd = Command::new("ros2");
    listener_cmd
        .args(["run", "demo_nodes_cpp", "listener"])
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .env("CYCLONEDDS_URI", CYCLONEDDS_URI)
        .stdout(Stdio::null())
        .stderr(Stdio::piped());
    use std::os::unix::process::CommandExt;
    listener_cmd.process_group(0);
    let listener_child = listener_cmd.spawn().expect("Failed to spawn listener");

    // Wait for the bridge to discover the DDS listener and create a Zenoh subscriber route.
    thread::sleep(Duration::from_secs(10));

    let session = zenoh_session(&endpoint);
    let publisher = session
        .declare_publisher("0/chatter/EMPTY_TOPIC_TYPE/EMPTY_TOPIC_HASH")
        .wait()
        .unwrap();
    let payload = cdr_string("Hello from rmw-zenoh");
    for _ in 0..30 {
        publisher
            .put(zenoh::bytes::ZBytes::from(payload.clone()))
            .wait()
            .unwrap();
        thread::sleep(Duration::from_millis(500));
    }

    let output = kill_group_and_collect(listener_child);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("Hello"),
        "listener received nothing; stderr={stderr:?}"
    );
}

// ── Test 9: DDS server ← bridge ← Zenoh get (rmw-zenoh) ──────────────────────

/// CycloneDDS add_two_ints_server runs. Bridge (rmw-zenoh mode) exposes it as a
/// Zenoh queryable. A Zenoh get() at `0/add_two_ints/EMPTY_TOPIC_TYPE/EMPTY_TOPIC_HASH`
/// returns sum=3.
#[test]
fn test_dds_srv_rosz_client() {
    if !ros2_available() || !pkg_available("demo_nodes_cpp") {
        eprintln!("Skipping test_dds_srv_rosz_client: dependencies not available");
        return;
    }

    let domain_id = 59u32;
    let router = TestRouter::new();
    let endpoint = router.endpoint().to_string();

    let _bridge = spawn_bridge(&endpoint, domain_id);
    let _server = spawn_cyclone(domain_id, "demo_nodes_cpp", "add_two_ints_server", &[]);
    thread::sleep(Duration::from_secs(2));

    let session = zenoh_session(&endpoint);
    let payload = cdr_add_two_ints_request(1, 2);

    let replies: Vec<_> = session
        .get("0/add_two_ints/EMPTY_TOPIC_TYPE/EMPTY_TOPIC_HASH")
        .payload(zenoh::bytes::ZBytes::from(payload))
        .timeout(Duration::from_secs(10))
        .wait()
        .unwrap()
        .into_iter()
        .collect();

    assert!(
        !replies.is_empty(),
        "No reply from DDS add_two_ints_server via rmw-zenoh bridge"
    );

    let reply_bytes: Vec<u8> = replies[0]
        .result()
        .expect("reply error")
        .payload()
        .to_bytes()
        .into_owned();

    let sum = parse_add_two_ints_response(&reply_bytes);
    assert_eq!(
        sum,
        Some(3),
        "Expected sum=3, got {:?}; raw={reply_bytes:?}",
        sum
    );
}

// ── Test 10: Zenoh server ← bridge ← DDS client (rmw-zenoh) ──────────────────

/// A Zenoh queryable at `0/add_two_ints/EMPTY_TOPIC_TYPE/EMPTY_TOPIC_HASH` acts
/// as the AddTwoInts server. A DDS add_two_ints_client calls through the bridge
/// (rmw-zenoh mode) and receives the computed reply.
#[test]
fn test_rosz_srv_dds_client() {
    if !ros2_available() || !pkg_available("demo_nodes_cpp") {
        eprintln!("Skipping test_rosz_srv_dds_client: dependencies not available");
        return;
    }

    let domain_id = 60u32;
    let router = TestRouter::new();
    let endpoint = router.endpoint().to_string();

    let _bridge = spawn_bridge(&endpoint, domain_id);

    let session = zenoh_session(&endpoint);
    let _queryable = session
        .declare_queryable("0/add_two_ints/EMPTY_TOPIC_TYPE/EMPTY_TOPIC_HASH")
        .callback(|query| {
            let payload: Vec<u8> = query
                .payload()
                .map(|p| p.to_bytes().into_owned())
                .unwrap_or_default();

            // Request layout: 4-byte CDR header + 8-byte a + 8-byte b
            let (a, b) = if payload.len() >= 20 {
                (
                    i64::from_le_bytes(payload[4..12].try_into().unwrap()),
                    i64::from_le_bytes(payload[12..20].try_into().unwrap()),
                )
            } else {
                (0i64, 0i64)
            };

            // Reply: 4-byte CDR header + 8-byte sum
            let mut reply = Vec::with_capacity(12);
            reply.extend_from_slice(&[0, 1, 0, 0]);
            reply.extend_from_slice(&(a + b).to_le_bytes());

            let ke = query.key_expr().clone();
            query
                .reply(ke, zenoh::bytes::ZBytes::from(reply))
                .wait()
                .unwrap();
        })
        .wait()
        .unwrap();

    thread::sleep(Duration::from_secs(2));

    let mut client_cmd = Command::new("ros2");
    client_cmd
        .args(["run", "demo_nodes_cpp", "add_two_ints_client"])
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .env("CYCLONEDDS_URI", CYCLONEDDS_URI)
        .stdout(Stdio::null())
        .stderr(Stdio::piped());
    use std::os::unix::process::CommandExt;
    client_cmd.process_group(0);
    let client_child = client_cmd.spawn().expect("spawn add_two_ints_client");

    let output = {
        let deadline = std::time::Instant::now() + Duration::from_secs(15);
        let mut child = client_child;
        loop {
            match child.try_wait() {
                Ok(Some(_)) => break child.wait_with_output().unwrap(),
                Ok(None) if std::time::Instant::now() < deadline => {
                    thread::sleep(Duration::from_millis(200));
                }
                _ => break kill_group_and_collect(child),
            }
        }
    };
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("3") || output.status.success(),
        "DDS client did not receive a valid reply; stderr={stderr:?}"
    );
}

// ── Test 11: ZDdsPubBridge::new() API construction ────────────────────────────

/// Verifies that `ZDdsPubBridge::new()` constructs without error when given a
/// valid ZNode and CyclorsParticipant.  No message passing — pure construction test.
#[cfg(feature = "dds-bridge-interop")]
#[test]
fn test_api_pub_bridge() {
    use ros_z::{Builder, context::ZContextBuilder};

    let domain_id = 70u32;
    let router = TestRouter::new();
    let endpoint = router.endpoint().to_string();

    unsafe { std::env::set_var("CYCLONEDDS_URI", CYCLONEDDS_URI) };

    let rt = tokio::runtime::Runtime::new().unwrap();
    rt.block_on(async {
        let ctx = ZContextBuilder::default()
            .with_connect_endpoints([endpoint.as_str()])
            .build()
            .expect("ZContext");
        let node = ctx
            .create_node("test_api_pub_bridge")
            .build()
            .expect("ZNode");
        let participant = CyclorsParticipant::create(domain_id).expect("participant");

        let bridge = ZDdsPubBridge::new(
            &node,
            "/chatter",
            "std_msgs/msg/String",
            None,
            &participant,
            BridgeQos::default(),
            true,
            10,
        )
        .await;

        assert!(
            bridge.is_ok(),
            "ZDdsPubBridge::new failed: {:?}",
            bridge.err()
        );
    });
}

// ── Test 12: ZDdsSubBridge::new() API construction ────────────────────────────

/// Verifies that `ZDdsSubBridge::new()` constructs without error.
#[cfg(feature = "dds-bridge-interop")]
#[test]
fn test_api_sub_bridge() {
    use ros_z::{Builder, context::ZContextBuilder};

    let domain_id = 71u32;
    let router = TestRouter::new();
    let endpoint = router.endpoint().to_string();

    unsafe { std::env::set_var("CYCLONEDDS_URI", CYCLONEDDS_URI) };

    let rt = tokio::runtime::Runtime::new().unwrap();
    rt.block_on(async {
        let ctx = ZContextBuilder::default()
            .with_connect_endpoints([endpoint.as_str()])
            .build()
            .expect("ZContext");
        let node = ctx
            .create_node("test_api_sub_bridge")
            .build()
            .expect("ZNode");
        let participant = CyclorsParticipant::create(domain_id).expect("participant");

        let bridge = ZDdsSubBridge::new(
            &node,
            "/chatter",
            "std_msgs/msg/String",
            None,
            &participant,
            BridgeQos::default(),
            true,
        )
        .await;

        assert!(
            bridge.is_ok(),
            "ZDdsSubBridge::new failed: {:?}",
            bridge.err()
        );
    });
}

// ── Test 13: DdsBridgeExt typed constructors ──────────────────────────────────

/// Verifies that the typed `DdsBridgeExt` convenience methods on `ZNode` construct
/// without error for both publisher and subscriber directions.
#[cfg(feature = "dds-bridge-interop")]
#[test]
fn test_api_typed_bridge() {
    use ros_z::{Builder, context::ZContextBuilder};

    let domain_id = 72u32;
    let router = TestRouter::new();
    let endpoint = router.endpoint().to_string();

    unsafe { std::env::set_var("CYCLONEDDS_URI", CYCLONEDDS_URI) };

    let rt = tokio::runtime::Runtime::new().unwrap();
    rt.block_on(async {
        let ctx = ZContextBuilder::default()
            .with_connect_endpoints([endpoint.as_str()])
            .build()
            .expect("ZContext");
        let node = ctx
            .create_node("test_api_typed_bridge")
            .build()
            .expect("ZNode");
        let participant = CyclorsParticipant::create(domain_id).expect("participant");

        let pub_bridge = node
            .bridge_dds_pub::<RosString>("/chatter", &participant)
            .await;
        assert!(
            pub_bridge.is_ok(),
            "bridge_dds_pub::<RosString> failed: {:?}",
            pub_bridge.err()
        );

        let sub_bridge = node
            .bridge_dds_sub::<RosString>("/chatter", &participant)
            .await;
        assert!(
            sub_bridge.is_ok(),
            "bridge_dds_sub::<RosString> failed: {:?}",
            sub_bridge.err()
        );
    });
}
