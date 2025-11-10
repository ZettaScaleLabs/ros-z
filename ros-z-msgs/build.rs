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

        let package_refs: Vec<&std::path::Path> = ros_packages.iter().map(|p| p.as_path()).collect();
        generator.generate_from_msg_files(&package_refs)?;

        println!("cargo:info=Generated ROS messages from {} packages", ros_packages.len());
    }

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-env-changed=CMAKE_PREFIX_PATH");

    Ok(())
}

fn discover_ros_packages() -> Result<Vec<PathBuf>> {
    let mut ros_packages = Vec::new();

    // Determine packages to discover based on enabled features
    let mut package_names = Vec::new();

    // builtin_interfaces is always needed as a dependency for other message types
    package_names.push("builtin_interfaces");

    #[cfg(feature = "std_msgs")]
    package_names.push("std_msgs");

    #[cfg(feature = "geometry_msgs")]
    package_names.push("geometry_msgs");

    #[cfg(feature = "sensor_msgs")]
    package_names.push("sensor_msgs");

    #[cfg(feature = "nav_msgs")]
    package_names.push("nav_msgs");

    #[cfg(feature = "example_interfaces")]
    package_names.push("example_interfaces");

    // Try to find packages using standard ROS 2 discovery mechanisms
    // 1. Check AMENT_PREFIX_PATH (standard ROS 2 environment variable)
    if let Ok(ament_prefix_path) = env::var("AMENT_PREFIX_PATH") {
        for prefix in ament_prefix_path.split(':') {
            let prefix_path = PathBuf::from(prefix);
            for package_name in &package_names {
                let package_path = prefix_path.join("share").join(package_name);
                if package_path.exists() && package_path.join("package.xml").exists() {
                    ros_packages.push(package_path);
                }
            }
        }
    }

    // 2. Check CMAKE_PREFIX_PATH (also commonly set in ROS 2)
    if ros_packages.is_empty() {
        if let Ok(cmake_prefix_path) = env::var("CMAKE_PREFIX_PATH") {
            for prefix in cmake_prefix_path.split(':') {
                let prefix_path = PathBuf::from(prefix);
                for package_name in &package_names {
                    let package_path = prefix_path.join("share").join(package_name);
                    if package_path.exists() && package_path.join("package.xml").exists() {
                        ros_packages.push(package_path);
                    }
                }
            }
        }
    }

    // 3. Check common ROS 2 installation paths
    if ros_packages.is_empty() {
        let common_install_paths = vec![
            "/opt/ros/rolling",
            "/opt/ros/jazzy",
            "/opt/ros/iron",
            "/opt/ros/humble",
        ];

        for install_path in common_install_paths {
            let install = PathBuf::from(install_path);
            if install.exists() {
                for package_name in &package_names {
                    let package_path = install.join("share").join(package_name);
                    if package_path.exists() && package_path.join("package.xml").exists() {
                        ros_packages.push(package_path);
                    }
                }
                if !ros_packages.is_empty() {
                    break;
                }
            }
        }
    }

    // Fallback to roslibrust assets if no system packages found
    if ros_packages.is_empty() {
        println!("cargo:warning=No system ROS packages found, falling back to roslibrust assets");
        let roslibrust_assets = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../roslibrust/assets/ros2_common_interfaces");

        #[cfg(feature = "std_msgs")]
        {
            let std_msgs = roslibrust_assets.join("std_msgs");
            if std_msgs.exists() {
                ros_packages.push(std_msgs);
            }
        }

        #[cfg(feature = "geometry_msgs")]
        {
            let geometry_msgs = roslibrust_assets.join("geometry_msgs");
            if geometry_msgs.exists() {
                ros_packages.push(geometry_msgs);
            }
        }

        #[cfg(feature = "sensor_msgs")]
        {
            let sensor_msgs = roslibrust_assets.join("sensor_msgs");
            if sensor_msgs.exists() {
                ros_packages.push(sensor_msgs);
            }
        }

        #[cfg(feature = "example_interfaces")]
        {
            let example_interfaces = roslibrust_assets.join("example_interfaces");
            if example_interfaces.exists() {
                ros_packages.push(example_interfaces);
            }
        }

        if ros_packages.is_empty() {
            ros_packages.push(roslibrust_assets.join("builtin_interfaces"));
            ros_packages.push(roslibrust_assets.join("std_msgs"));
            ros_packages.push(roslibrust_assets.join("geometry_msgs"));
            ros_packages.push(roslibrust_assets.join("sensor_msgs"));
            ros_packages.push(roslibrust_assets.join("example_interfaces"));
        }
    }

    Ok(ros_packages)
}