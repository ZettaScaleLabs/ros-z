use std::{env, path::PathBuf};

use anyhow::Result;

fn main() -> Result<()> {
    let out_dir = PathBuf::from(env::var("OUT_DIR")?);

    // Declare custom cfg for ROS version detection
    println!("cargo:rustc-check-cfg=cfg(ros_humble)");

    // Detect ROS version and emit cfg
    detect_ros_version();

    // Discover ROS packages
    let ros_packages = discover_ros_packages()?;

    println!(
        "cargo:warning=protobuf feature: {}",
        cfg!(feature = "protobuf")
    );
    println!("cargo:warning=ros_packages len: {}", ros_packages.len());

    if !ros_packages.is_empty() {
        println!("cargo:warning=generating messages");
        let config = ros_z_codegen::GeneratorConfig {
            generate_cdr: true, // Always generate for ROS2 compatibility
            generate_protobuf: cfg!(feature = "protobuf"),
            generate_type_info: true,
            output_dir: out_dir.clone(),
        };

        let generator = ros_z_codegen::MessageGenerator::new(config);

        let package_refs: Vec<&std::path::Path> =
            ros_packages.iter().map(|p| p.as_path()).collect();
        generator.generate_from_msg_files(&package_refs)?;
        println!("cargo:warning=generated messages");

        println!(
            "cargo:info=Generated ROS messages from {} packages",
            ros_packages.len()
        );
    }

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-env-changed=CMAKE_PREFIX_PATH");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_PROTOBUF");

    // Ensure generated_proto.rs exists even if protobuf generation is skipped
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let proto_file = out_dir.join("generated_proto.rs");
    if !proto_file.exists() {
        std::fs::write(&proto_file, "// Empty protobuf generated file\n").unwrap();
    }

    Ok(())
}

fn discover_ros_packages() -> Result<Vec<PathBuf>> {
    use std::collections::HashMap;

    // Use HashMap to track packages by name and deduplicate
    let mut package_map: HashMap<String, PathBuf> = HashMap::new();

    let all_packages = get_all_packages();

    // Priority 1: System ROS installation (highest priority - most up-to-date)
    let system_packages = discover_system_packages(&all_packages)?;
    let system_count = system_packages.len();
    for pkg_path in system_packages {
        if let Ok(name) = discover_package_name_from_path(&pkg_path) {
            println!("cargo:info=System: Adding package {}", name);
            package_map.insert(name, pkg_path);
        }
    }

    if system_count > 0 {
        println!(
            "cargo:info=Found {} packages from ROS 2 installation (deduplicated to {})",
            system_count,
            package_map.len()
        );
        return Ok(package_map.into_values().collect());
    }

    // Priority 2: Local bundled assets (standalone mode)
    println!("cargo:info=Using local bundled assets from ros-z-codegen/assets/jazzy");
    let local_asset_packages = discover_local_assets(&all_packages)?;
    for pkg_path in local_asset_packages {
        if let Ok(name) = discover_package_name_from_path(&pkg_path) {
            package_map.entry(name).or_insert(pkg_path);
        }
    }

    println!(
        "cargo:info=Total unique packages discovered: {}",
        package_map.len()
    );

    // Warn if packages are still not found
    let missing: Vec<_> = all_packages
        .iter()
        .filter(|&&pkg| !package_map.contains_key(pkg))
        .collect();

    if !missing.is_empty() {
        println!("cargo:warning=Missing packages: {:?}", missing);
        println!("cargo:warning=Consider installing ROS 2 or checking ros-z-codegen/assets/jazzy");
    }

    Ok(package_map.into_values().collect())
}

/// Extract package name from path using package.xml or directory name
fn discover_package_name_from_path(package_path: &std::path::Path) -> Result<String> {
    ros_z_codegen::discovery::discover_package_name(package_path)
}

/// Get list of all package names based on enabled features
/// All packages are now bundled in ros-z-codegen/assets/jazzy
fn get_all_packages() -> Vec<&'static str> {
    let mut names = vec![
        "builtin_interfaces",     // Always required
        "service_msgs",           // Required for service type hashing
        "action_msgs",            // Required for ROS 2 actions
        "unique_identifier_msgs", // Required by action_msgs
    ];

    #[cfg(feature = "std_msgs")]
    names.push("std_msgs");

    #[cfg(feature = "geometry_msgs")]
    names.push("geometry_msgs");

    #[cfg(feature = "sensor_msgs")]
    names.push("sensor_msgs");

    #[cfg(feature = "nav_msgs")]
    names.push("nav_msgs");

    #[cfg(feature = "example_interfaces")]
    names.push("example_interfaces");

    #[cfg(feature = "action_tutorials_interfaces")]
    names.push("action_tutorials_interfaces");

    #[cfg(feature = "test_msgs")]
    names.push("test_msgs");

    names
}

/// Try to discover packages from system ROS 2 installation
fn discover_system_packages(packages: &[&str]) -> Result<Vec<PathBuf>> {
    if packages.is_empty() {
        return Ok(Vec::new());
    }

    let mut found_packages = Vec::new();

    // 1. Check AMENT_PREFIX_PATH (standard ROS 2 environment variable)
    if let Ok(ament_prefix_path) = env::var("AMENT_PREFIX_PATH") {
        for prefix in ament_prefix_path.split(':') {
            let prefix_path = PathBuf::from(prefix);
            for package_name in packages {
                let package_path = prefix_path.join("share").join(package_name);
                if package_path.exists()
                    && (package_path.join("msg").exists()
                        || package_path.join("srv").exists()
                        || package_path.join("action").exists())
                {
                    found_packages.push(package_path);
                }
            }
        }
    }

    // 2. Check CMAKE_PREFIX_PATH (also commonly set in ROS 2)
    if found_packages.is_empty()
        && let Ok(cmake_prefix_path) = env::var("CMAKE_PREFIX_PATH")
    {
        for prefix in cmake_prefix_path.split(':') {
            let prefix_path = PathBuf::from(prefix);
            for package_name in packages {
                let package_path = prefix_path.join("share").join(package_name);
                if package_path.exists()
                    && (package_path.join("msg").exists()
                        || package_path.join("srv").exists()
                        || package_path.join("action").exists())
                {
                    found_packages.push(package_path);
                }
            }
        }
    }

    // 3. Check common ROS 2 installation paths
    if found_packages.is_empty() {
        let common_install_paths = vec![
            "/opt/ros/rolling",
            "/opt/ros/jazzy",
            "/opt/ros/iron",
            "/opt/ros/humble",
        ];

        for install_path in common_install_paths {
            let install = PathBuf::from(install_path);
            if install.exists() {
                for package_name in packages {
                    let package_path = install.join("share").join(package_name);
                    if package_path.exists()
                        && (package_path.join("msg").exists()
                            || package_path.join("srv").exists()
                            || package_path.join("action").exists())
                    {
                        found_packages.push(package_path);
                    }
                }
                if !found_packages.is_empty() {
                    break;
                }
            }
        }
    }

    Ok(found_packages)
}

/// Discover packages from local bundled assets (ros-z-codegen/assets/jazzy/)
fn discover_local_assets(package_names: &[&str]) -> Result<Vec<PathBuf>> {
    let mut found_packages = Vec::new();

    // Get the path to ros-z-codegen/assets/jazzy relative to this crate
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let assets_dir = manifest_dir
        .parent()
        .expect("Failed to get parent directory")
        .join("ros-z-codegen/assets/jazzy");

    if !assets_dir.exists() {
        println!(
            "cargo:warning=Local assets directory not found: {:?}",
            assets_dir
        );
        return Ok(Vec::new());
    }

    // Search for packages in jazzy assets directory
    for package_name in package_names {
        let package_path = assets_dir.join(package_name);

        if package_path.exists()
            && (package_path.join("msg").exists()
                || package_path.join("srv").exists()
                || package_path.join("action").exists())
        {
            println!(
                "cargo:info=Found {} in local assets: {:?}",
                package_name, package_path
            );
            found_packages.push(package_path);
        }
    }

    Ok(found_packages)
}

/// Detect ROS version and emit cfg(ros_humble) if Humble is detected
fn detect_ros_version() {
    // Check if ROS is installed by looking for AMENT_PREFIX_PATH
    if let Ok(ament_prefix) = env::var("AMENT_PREFIX_PATH") {
        // Jazzy and newer have type_description_interfaces, Humble doesn't
        let has_type_description = ament_prefix.split(':').any(|prefix| {
            PathBuf::from(prefix)
                .join("include/type_description_interfaces")
                .exists()
        });

        if !has_type_description {
            // No type_description_interfaces means Humble
            println!("cargo:rustc-cfg=ros_humble");
            println!("cargo:warning=ROS Humble detected - using Humble-compatible codegen");
        } else {
            println!("cargo:warning=ROS Jazzy+ detected - using modern codegen");
        }
    } else if env::var("CARGO_FEATURE_HUMBLE_COMPAT").is_ok() {
        // Pure Rust mode: user explicitly requested Humble compatibility
        println!("cargo:rustc-cfg=ros_humble");
        println!("cargo:warning=humble_compat feature enabled - using Humble-compatible codegen");
    }
}
