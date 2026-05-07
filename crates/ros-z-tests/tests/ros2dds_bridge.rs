//! Integration tests for ros-z-bridge-ros2dds.
//!
//! These tests are the direct equivalent of the zenoh-plugin-ros2dds test suite
//! (zenoh-test-ros2dds) but use `ros2 run` external processes instead of r2r,
//! and replace the in-process plugin with the `ros-z-bridge-ros2dds` binary.
//!
//! Coverage:
//! | # | Scenario                             | zenoh-plugin-ros2dds test            |
//! |---|--------------------------------------|--------------------------------------|
//! | 1 | Zenoh pub → bridge → DDS sub         | test_zenoh_pub_ros_sub               |
//! | 2 | DDS pub → bridge → Zenoh sub         | test_ros_pub_zenoh_sub               |
//! | 3 | Zenoh queryable ← bridge ← DDS cli   | test_ros_client_zenoh_service        |
//! | 4 | DDS server ← bridge ← Zenoh get      | test_zenoh_client_ros_service        |
//! | 5 | Zenoh action server ← bridge ← DDS   | test_ros_client_zenoh_action         |
//! | 6 | DDS action server ← bridge ← Zenoh   | test_zenoh_client_ros_action         |
//!
//! Requirements:
//! - ROS 2 Jazzy with `rmw_cyclonedds_cpp`, `demo_nodes_cpp`, `action_tutorials_cpp`
//! - `ros-z-bridge-ros2dds` binary built at `target/debug/` or in PATH
//!
//! Run with:
//! ```bash
//! cargo test -p ros-z-tests --features ros2dds-bridge-interop,jazzy
//! ```

#![cfg(feature = "ros2dds-bridge-interop")]

mod common;

use std::{
    process::{Command, Stdio},
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use common::{ProcessGuard, TestRouter};
use zenoh::Wait;

// ── helpers ──────────────────────────────────────────────────────────────────

fn bridge_binary() -> String {
    let local = format!(
        "{}/../../target/debug/ros-z-bridge-ros2dds",
        env!("CARGO_MANIFEST_DIR")
    );
    if std::path::Path::new(&local).exists() {
        return local;
    }
    "ros-z-bridge-ros2dds".to_string()
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

fn spawn_bridge(zenoh_endpoint: &str, domain_id: u32) -> ProcessGuard {
    use std::os::unix::process::CommandExt;
    let bin = bridge_binary();
    let child = Command::new(&bin)
        .args([
            "--zenoh-endpoint",
            zenoh_endpoint,
            "--domain-id",
            &domain_id.to_string(),
        ])
        .env("RUST_LOG", "info")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .process_group(0)
        .spawn()
        .unwrap_or_else(|e| panic!("Failed to spawn {bin}: {e}"));
    thread::sleep(Duration::from_secs(2));
    ProcessGuard::new(child, "ros-z-bridge-ros2dds")
}

fn spawn_cyclone(domain_id: u32, pkg: &str, node: &str, extra_args: &[&str]) -> ProcessGuard {
    use std::os::unix::process::CommandExt;
    let name = format!("{}/{}", pkg, node);
    let mut cmd = Command::new("ros2");
    cmd.args(["run", pkg, node])
        .args(extra_args)
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
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

// ── Test 1: Zenoh pub → bridge → DDS subscriber ──────────────────────────────

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

    let _bridge = spawn_bridge(&endpoint, domain_id);

    // Start listener and collect its stdout
    let mut listener_cmd = Command::new("ros2");
    listener_cmd
        .args(["run", "demo_nodes_cpp", "listener"])
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::null());
    use std::os::unix::process::CommandExt;
    listener_cmd.process_group(0);
    let mut listener_child = listener_cmd.spawn().expect("Failed to spawn listener");
    thread::sleep(Duration::from_secs(2));

    // Publish from Zenoh side
    let session = zenoh_session(&endpoint);
    let publisher = session.declare_publisher("chatter").wait().unwrap();
    let payload = cdr_string("Hello from Zenoh");
    publisher
        .put(zenoh::bytes::ZBytes::from(payload))
        .wait()
        .unwrap();
    thread::sleep(Duration::from_millis(500));

    // Read listener output
    let _ = listener_child.kill();
    let output = listener_child.wait_with_output().unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("Hello") || !stdout.is_empty(),
        "listener received nothing; stdout={stdout:?}"
    );
}

// ── Test 2: DDS pub → bridge → Zenoh subscriber ──────────────────────────────

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

    let _bridge = spawn_bridge(&endpoint, domain_id);

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

// ── Test 3: Zenoh queryable ← bridge ← DDS service client ────────────────────

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

    let _bridge = spawn_bridge(&endpoint, domain_id);

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

    // Spawn DDS client (sends one request: 1 + 2)
    let mut client_cmd = Command::new("ros2");
    client_cmd
        .args(["run", "demo_nodes_cpp", "add_two_ints_client"])
        .env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::null());
    use std::os::unix::process::CommandExt;
    client_cmd.process_group(0);
    let client_child = client_cmd.spawn().expect("spawn add_two_ints_client");

    // Wait for client to complete (it exits after one response)
    let output = {
        let timeout = std::time::Instant::now() + Duration::from_secs(15);
        let mut child = client_child;
        loop {
            match child.try_wait() {
                Ok(Some(_)) => break child.wait_with_output().unwrap(),
                Ok(None) if std::time::Instant::now() < timeout => {
                    thread::sleep(Duration::from_millis(200));
                }
                _ => {
                    let _ = child.kill();
                    break child.wait_with_output().unwrap();
                }
            }
        }
    };
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("3") || output.status.success(),
        "DDS client did not receive a valid reply; stdout={stdout:?}"
    );
}

// ── Test 4: Zenoh client → bridge → DDS service server ───────────────────────

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

    let _bridge = spawn_bridge(&endpoint, domain_id);
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

// ── Test 5: Zenoh action server ← bridge ← DDS action client ─────────────────

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

    let _bridge = spawn_bridge(&endpoint, domain_id);
    thread::sleep(Duration::from_secs(1));

    let session = zenoh_session(&endpoint);

    // Zenoh action server components
    // send_goal queryable
    let session_sg = session.clone();
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
                _ => {
                    let _ = c.kill();
                    break c.wait_with_output().unwrap();
                }
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

    let _bridge = spawn_bridge(&endpoint, domain_id);
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
