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

        #[cfg(feature = "protobuf")]
        {
            // This is a placeholder for getting the message files.
            // In a real implementation, `generate_from_msg_files` would return the parsed message files.
            let msg_files = Vec::new(); 
            let proto_files = generator.msg_to_proto(&msg_files)?;
            generator.generate_from_proto_files(&proto_files)?;
        }

        println!("cargo:info=Generated ROS messages from {} packages", ros_packages.len());
    }

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=ROS_PACKAGE_PATH");

    Ok(())
}

fn discover_ros_packages() -> Result<Vec<PathBuf>> {
    let mut ros_packages = Vec::new();

    // Check for nix-ros installation (rolling)
    let nix_ros_paths = vec![
        "/nix/store/*-ros-rolling-std-msgs-*/share/std_msgs",
        "/nix/store/*-ros-rolling-geometry-msgs-*/share/geometry_msgs",
    ];

    for pattern in nix_ros_paths {
        if let Ok(paths) = glob::glob(pattern) {
            for path in paths.flatten() {
                if path.exists() && path.join("package.xml").exists() {
                    ros_packages.push(path);
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

        if ros_packages.is_empty() {
            ros_packages.push(roslibrust_assets.join("std_msgs"));
            ros_packages.push(roslibrust_assets.join("geometry_msgs"));
        }
    }
    Ok(ros_packages)
}