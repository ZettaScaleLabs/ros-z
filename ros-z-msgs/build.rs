use anyhow::Result;
use std::env;
use std::path::PathBuf;

fn main() -> Result<()> {
    let out_dir = PathBuf::from(env::var("OUT_DIR")?);

    // Try to find ROS2 packages from system installation
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

    // Generate message types using ros-z-codegen
    if !ros_packages.is_empty() {
        let config = ros_z_codegen::GeneratorConfig {
            generate_type_info: true,
            output_dir: out_dir,
        };

        let generator = ros_z_codegen::MessageGenerator::new(config);

        // Convert to slices for the API
        let package_refs: Vec<&std::path::Path> = ros_packages.iter().map(|p| p.as_path()).collect();

        println!("cargo:warning=Generating from packages: {:?}", ros_packages);
        generator.generate_from_packages(&package_refs)?;

        println!("cargo:info=Generated ROS messages from {} packages", ros_packages.len());
    }

    // Rerun build script if build.rs changes
    println!("cargo:rerun-if-changed=build.rs");

    Ok(())
}
