//! Golden hash regression tests for real ROS 2 message types.
//!
//! These tests load real message definitions from the bundled Jazzy assets,
//! resolve the full type tree, and verify that the computed RIHS01 hashes:
//!
//! 1. Have the correct format (`RIHS01_` + 64 hex chars)
//! 2. Are deterministic across multiple calls
//! 3. Match the expected values (computed from the fixed implementation and
//!    verified against `rmw_zenoh_cpp` interop tests)
//!
//! The interop tests in `ros-z-tests` provide end-to-end correctness validation;
//! these tests guard against regressions in future refactors.

use std::{collections::HashMap, path::PathBuf};

use ros_z_codegen::{
    discovery::{discover_messages, discover_services},
    resolver::Resolver,
    types::{ResolvedMessage, ResolvedService},
};

/// Path to the bundled Jazzy message assets.
fn assets_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("assets/jazzy")
}

/// Resolve all messages from the listed packages using the bundled Jazzy assets.
///
/// Returns a map from `pkg/msg/TypeName` to the resolved message (which carries
/// the computed `TypeHash`).
fn resolve_packages(packages: &[&str]) -> HashMap<String, ResolvedMessage> {
    let assets = assets_dir();

    let mut all_messages = Vec::new();
    for pkg in packages {
        let pkg_path = assets.join(pkg);
        let msgs = discover_messages(&pkg_path, pkg)
            .unwrap_or_else(|e| panic!("Failed to discover messages in {pkg}: {e}"));
        all_messages.extend(msgs);
    }

    let mut resolver = Resolver::new(false /* is_humble */);
    let resolved = resolver
        .resolve_messages(all_messages)
        .expect("Failed to resolve messages");

    resolved
        .into_iter()
        .map(|r| {
            let key = format!("{}/msg/{}", r.parsed.package, r.parsed.name);
            (key, r)
        })
        .collect()
}

/// All packages needed to resolve the geometry/sensor/nav message types used
/// in the tests below.
fn all_test_packages() -> &'static [&'static str] {
    &[
        "builtin_interfaces",
        "unique_identifier_msgs",
        "action_msgs",
        "std_msgs",
        "geometry_msgs",
        "sensor_msgs",
        "nav_msgs",
    ]
}

fn get_hash(resolved: &HashMap<String, ResolvedMessage>, type_name: &str) -> String {
    resolved
        .get(type_name)
        .unwrap_or_else(|| panic!("Type {type_name} not found in resolved messages"))
        .type_hash
        .to_rihs_string()
}

// --- Format and determinism tests ---

#[test]
fn test_hash_format_std_msgs_string() {
    // std_msgs/msg/Header depends on builtin_interfaces/msg/Time, so both
    // packages are needed for the resolver to successfully resolve all messages.
    let resolved = resolve_packages(&["builtin_interfaces", "std_msgs"]);
    let hash = get_hash(&resolved, "std_msgs/msg/String");
    assert!(
        hash.starts_with("RIHS01_"),
        "Hash should start with RIHS01_: {hash}"
    );
    assert_eq!(
        hash.len(),
        7 + 64,
        "Hash should be RIHS01_ + 64 hex chars: {hash}"
    );
}

#[test]
fn test_hash_deterministic_twist_stamped() {
    let resolved1 = resolve_packages(all_test_packages());
    let resolved2 = resolve_packages(all_test_packages());
    let hash1 = get_hash(&resolved1, "geometry_msgs/msg/TwistStamped");
    let hash2 = get_hash(&resolved2, "geometry_msgs/msg/TwistStamped");
    assert_eq!(hash1, hash2, "Hash must be deterministic");
}

#[test]
fn test_hashes_differ_by_type() {
    let resolved = resolve_packages(all_test_packages());
    let h_string = get_hash(&resolved, "std_msgs/msg/String");
    let h_header = get_hash(&resolved, "std_msgs/msg/Header");
    let h_twist_stamped = get_hash(&resolved, "geometry_msgs/msg/TwistStamped");
    let h_pose_stamped = get_hash(&resolved, "geometry_msgs/msg/PoseStamped");
    let h_imu = get_hash(&resolved, "sensor_msgs/msg/Imu");
    let h_odom = get_hash(&resolved, "nav_msgs/msg/Odometry");

    let all = [
        &h_string,
        &h_header,
        &h_twist_stamped,
        &h_pose_stamped,
        &h_imu,
        &h_odom,
    ];
    for (i, a) in all.iter().enumerate() {
        for (j, b) in all.iter().enumerate() {
            if i != j {
                assert_ne!(
                    a, b,
                    "Hashes for different types must differ (indices {i}, {j})"
                );
            }
        }
    }
}

// --- Expected value tests ---
//
// These values were computed from the fixed implementation of `collect_referenced_types`
// (using `nested_type_name_to_key`) against the bundled Jazzy assets.
//
// End-to-end correctness (i.e. these match what rmw_zenoh_cpp computes) is
// validated by the interop tests in `ros-z-tests/tests/pubsub_interop.rs`.
// These tests guard against regressions in future refactors.

#[test]
fn test_expected_hash_std_msgs_string() {
    let resolved = resolve_packages(&["builtin_interfaces", "std_msgs"]);
    let hash = get_hash(&resolved, "std_msgs/msg/String");
    assert_eq!(
        hash, "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
        "std_msgs/msg/String hash mismatch"
    );
}

#[test]
fn test_expected_hash_std_msgs_header() {
    let resolved = resolve_packages(&["builtin_interfaces", "std_msgs"]);
    let hash = get_hash(&resolved, "std_msgs/msg/Header");
    assert_eq!(
        hash, "RIHS01_f49fb3ae2cf070f793645ff749683ac6b06203e41c891e17701b1cb597ce6a01",
        "std_msgs/msg/Header hash mismatch"
    );
}

#[test]
fn test_expected_hash_twist_stamped() {
    let resolved = resolve_packages(all_test_packages());
    let hash = get_hash(&resolved, "geometry_msgs/msg/TwistStamped");
    assert_eq!(
        hash, "RIHS01_5f0fcd4f81d5d06ad9b4c4c63e3ea51b82d6ae4d0558f1d475229b1121db6f64",
        "geometry_msgs/msg/TwistStamped hash mismatch"
    );
}

#[test]
fn test_expected_hash_pose_stamped() {
    let resolved = resolve_packages(all_test_packages());
    let hash = get_hash(&resolved, "geometry_msgs/msg/PoseStamped");
    assert_eq!(
        hash, "RIHS01_10f3786d7d40fd2b54367835614bff85d4ad3b5dab62bf8bca0cc232d73b4cd8",
        "geometry_msgs/msg/PoseStamped hash mismatch"
    );
}

#[test]
fn test_expected_hash_imu() {
    let resolved = resolve_packages(all_test_packages());
    let hash = get_hash(&resolved, "sensor_msgs/msg/Imu");
    assert_eq!(
        hash, "RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b",
        "sensor_msgs/msg/Imu hash mismatch"
    );
}

#[test]
fn test_expected_hash_odometry() {
    let resolved = resolve_packages(all_test_packages());
    let hash = get_hash(&resolved, "nav_msgs/msg/Odometry");
    assert_eq!(
        hash, "RIHS01_3cc97dc7fb7502f8714462c526d369e35b603cfc34d946e3f2eda2766dfec6e0",
        "nav_msgs/msg/Odometry hash mismatch"
    );
}

/// Resolve services from the listed packages using the bundled Jazzy assets.
fn resolve_services_from_packages(packages: &[&str]) -> HashMap<String, ResolvedService> {
    let assets = assets_dir();

    let mut all_messages = Vec::new();
    let mut all_services = Vec::new();
    for pkg in packages {
        let pkg_path = assets.join(pkg);
        let msgs = discover_messages(&pkg_path, pkg)
            .unwrap_or_else(|e| panic!("Failed to discover messages in {pkg}: {e}"));
        all_messages.extend(msgs);
        let srvs = discover_services(&pkg_path, pkg)
            .unwrap_or_else(|e| panic!("Failed to discover services in {pkg}: {e}"));
        all_services.extend(srvs);
    }

    let mut resolver = Resolver::new(false /* is_humble */);
    let _ = resolver
        .resolve_messages(all_messages)
        .expect("Failed to resolve messages");
    let resolved = resolver
        .resolve_services(all_services)
        .expect("Failed to resolve services");

    resolved
        .into_iter()
        .map(|r| {
            let key = format!("{}/srv/{}", r.parsed.package, r.parsed.name);
            (key, r)
        })
        .collect()
}

fn get_service_hash(resolved: &HashMap<String, ResolvedService>, type_name: &str) -> String {
    resolved
        .get(type_name)
        .unwrap_or_else(|| panic!("Type {type_name} not found in resolved services"))
        .type_hash
        .to_rihs_string()
}

// --- Helper to print current hash values (run with --ignored --nocapture) ---

#[test]
#[ignore = "utility: prints current hash values for capture and comparison with ROS 2"]
fn print_hashes() {
    let resolved = resolve_packages(all_test_packages());

    let types = [
        "std_msgs/msg/String",
        "std_msgs/msg/Header",
        "geometry_msgs/msg/Twist",
        "geometry_msgs/msg/TwistStamped",
        "geometry_msgs/msg/PoseStamped",
        "sensor_msgs/msg/Imu",
        "nav_msgs/msg/Odometry",
    ];

    println!("\n=== Computed RIHS01 hashes (jazzy assets, fixed nested lookup) ===");
    for t in types {
        let hash = get_hash(&resolved, t);
        println!("  {t}: {hash}");
    }
    println!();
}

#[test]
#[ignore = "utility: prints current service hash values for comparison with wire_types.rs"]
fn print_service_hashes() {
    let packages = &["builtin_interfaces", "service_msgs", "rcl_interfaces"];
    let resolved = resolve_services_from_packages(packages);

    let services = [
        "rcl_interfaces/srv/GetParameters",
        "rcl_interfaces/srv/SetParameters",
        "rcl_interfaces/srv/ListParameters",
        "rcl_interfaces/srv/DescribeParameters",
        "rcl_interfaces/srv/GetParameterTypes",
        "rcl_interfaces/srv/SetParametersAtomically",
    ];

    println!("\n=== Computed RIHS01 service hashes (jazzy assets) ===");
    for t in services {
        if let Some(r) = resolved.get(t) {
            println!("  {t}: {}", r.type_hash.to_rihs_string());
        } else {
            println!("  {t}: NOT FOUND");
        }
    }
    println!();
}
