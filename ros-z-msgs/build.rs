use anyhow::Result;
use std::env;
use std::path::PathBuf;

fn main() -> Result<()> {
    let out_dir = PathBuf::from(env::var("OUT_DIR")?);

    // Discover ROS packages
    let ros_packages = discover_ros_packages()?;

    if !ros_packages.is_empty() {
        let config = ros_z_codegen::GeneratorConfig {
            generate_cdr: true, // Always generate for ROS2 compatibility
            #[cfg(feature = "protobuf")]
            generate_protobuf: true,
            #[cfg(not(feature = "protobuf"))]
            generate_protobuf: false,
            generate_type_info: true,
            output_dir: out_dir.clone(),
        };

        let generator = ros_z_codegen::MessageGenerator::new(config);

        let package_refs: Vec<&std::path::Path> =
            ros_packages.iter().map(|p| p.as_path()).collect();
        generator.generate_from_msg_files(&package_refs)?;

        println!(
            "cargo:info=Generated ROS messages from {} packages",
            ros_packages.len()
        );
    }

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-env-changed=CMAKE_PREFIX_PATH");

    Ok(())
}

fn discover_ros_packages() -> Result<Vec<PathBuf>> {
    let mut ros_packages = Vec::new();

    // Categorize packages into bundled (available via roslibrust) and external (require ROS)
    let bundled_packages = get_bundled_packages();
    let external_packages = get_external_packages();

    // Try to find packages from ROS 2 installation first (supports both bundled and external)
    let system_packages = discover_system_packages(&bundled_packages, &external_packages)?;

    if !system_packages.is_empty() {
        println!(
            "cargo:info=Found {} packages from ROS 2 installation",
            system_packages.len()
        );
        return Ok(system_packages);
    }

    // Fallback to roslibrust bundled assets for bundled packages only
    if !bundled_packages.is_empty() {
        println!("cargo:info=No system ROS packages found, using bundled roslibrust assets");
        ros_packages = discover_bundled_packages(&bundled_packages)?;
    }

    // Warn if external packages are requested but not found
    if !external_packages.is_empty() && system_packages.is_empty() {
        for package in &external_packages {
            println!(
                "cargo:warning={} feature enabled but package not found - requires ROS 2 installation",
                package
            );
        }
    }

    Ok(ros_packages)
}

/// Get list of bundled package names based on enabled features
/// These packages are available in roslibrust assets and don't require ROS installation
fn get_bundled_packages() -> Vec<&'static str> {
    let mut names = vec!["builtin_interfaces"]; // Always required

    #[cfg(feature = "std_msgs")]
    names.push("std_msgs");

    #[cfg(feature = "geometry_msgs")]
    names.push("geometry_msgs");

    #[cfg(feature = "sensor_msgs")]
    names.push("sensor_msgs");

    #[cfg(feature = "nav_msgs")]
    names.push("nav_msgs");

    names
}

/// Get list of external package names based on enabled features
/// These packages require a ROS 2 installation and are not bundled
fn get_external_packages() -> Vec<&'static str> {
    #[cfg(feature = "example_interfaces")]
    return vec!["example_interfaces"];

    #[cfg(not(feature = "example_interfaces"))]
    Vec::new()
}

/// Try to discover packages from system ROS 2 installation
fn discover_system_packages(
    bundled_packages: &[&str],
    external_packages: &[&str],
) -> Result<Vec<PathBuf>> {
    let mut all_packages = Vec::new();
    all_packages.extend_from_slice(bundled_packages);
    all_packages.extend_from_slice(external_packages);

    if all_packages.is_empty() {
        return Ok(Vec::new());
    }

    let mut found_packages = Vec::new();

    // 1. Check AMENT_PREFIX_PATH (standard ROS 2 environment variable)
    if let Ok(ament_prefix_path) = env::var("AMENT_PREFIX_PATH") {
        for prefix in ament_prefix_path.split(':') {
            let prefix_path = PathBuf::from(prefix);
            for package_name in &all_packages {
                let package_path = prefix_path.join("share").join(package_name);
                if package_path.exists() && package_path.join("package.xml").exists() {
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
            for package_name in &all_packages {
                let package_path = prefix_path.join("share").join(package_name);
                if package_path.exists() && package_path.join("package.xml").exists() {
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
                for package_name in &all_packages {
                    let package_path = install.join("share").join(package_name);
                    if package_path.exists() && package_path.join("package.xml").exists() {
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

/// Discover bundled packages from roslibrust assets
fn discover_bundled_packages(bundled_packages: &[&str]) -> Result<Vec<PathBuf>> {
    let mut ros_packages = Vec::new();
    let roslibrust_assets = find_roslibrust_assets();

    // builtin_interfaces is in ros2_required_msgs
    if bundled_packages.contains(&"builtin_interfaces") {
        let builtin_interfaces =
            roslibrust_assets.join("ros2_required_msgs/rcl_interfaces/builtin_interfaces");
        if builtin_interfaces.exists() {
            ros_packages.push(builtin_interfaces);
        } else {
            println!("cargo:warning=builtin_interfaces not found in roslibrust assets");
        }
    }

    // Common interfaces are in ros2_common_interfaces
    let common_interfaces = roslibrust_assets.join("ros2_common_interfaces");

    for package_name in bundled_packages {
        if *package_name == "builtin_interfaces" {
            continue; // Already handled above
        }

        let package_path = common_interfaces.join(package_name);
        if package_path.exists() {
            ros_packages.push(package_path);
        } else {
            println!(
                "cargo:warning={} not found in roslibrust assets at {}",
                package_name,
                package_path.display()
            );
        }
    }

    Ok(ros_packages)
}

/// Find roslibrust assets directory from git dependency
/// Returns the base assets directory (not ros2_common_interfaces subdirectory)
fn find_roslibrust_assets() -> PathBuf {
    // Search in cargo's git checkout directory
    // The path will be something like: ~/.cargo/git/checkouts/roslibrust-{hash}/{commit}/assets
    if let Ok(home) = env::var("CARGO_HOME").or_else(|_| env::var("HOME")) {
        let cargo_git = PathBuf::from(home).join(".cargo/git/checkouts");

        if let Ok(entries) = std::fs::read_dir(&cargo_git) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_dir()
                    && path
                        .file_name()
                        .and_then(|n| n.to_str())
                        .is_some_and(|n| n.starts_with("roslibrust-"))
                {
                    // Look for the assets directory in any commit subdirectory
                    if let Ok(commits) = std::fs::read_dir(&path) {
                        for commit_entry in commits.flatten() {
                            let assets_path = commit_entry.path().join("assets");
                            if assets_path.exists() {
                                println!(
                                    "cargo:warning=Using roslibrust git assets at {}",
                                    assets_path.display()
                                );
                                return assets_path;
                            }
                        }
                    }
                }
            }
        }
    }

    // Fallback: panic with helpful error message
    panic!(
        "Could not find roslibrust assets directory!\n\
         Make sure roslibrust is specified as a git dependency in Cargo.toml.\n\
         The build system searches: ~/.cargo/git/checkouts/roslibrust-*/*/assets/"
    );
}
