use std::path::PathBuf;

// Include config module from src/ to access ConfigOverride pattern
// Allow dead_code since build.rs only uses a subset of config.rs APIs
#[allow(dead_code)]
#[path = "src/config.rs"]
mod config;

fn main() {
    println!("cargo:rerun-if-changed=src/config.rs");
    println!("cargo:rerun-if-env-changed=ROS_Z_CONFIG_OUTPUT_DIR");

    // Determine output directory early to set up rerun-if-changed
    let config_dir = if let Ok(custom_dir) = std::env::var("ROS_Z_CONFIG_OUTPUT_DIR") {
        let path = PathBuf::from(&custom_dir);
        if path.is_absolute() {
            path
        } else {
            let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
            manifest_dir.join(path)
        }
    } else {
        let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
        out_dir.join("ros_z_config")
    };

    // Always set rerun-if-changed on the output files to ensure regeneration if deleted
    println!(
        "cargo:rerun-if-changed={}",
        config_dir
            .join("DEFAULT_ROSZ_ROUTER_CONFIG.json5")
            .display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        config_dir
            .join("DEFAULT_ROSZ_SESSION_CONFIG.json5")
            .display()
    );

    // Only generate configs if the feature is enabled
    #[cfg(feature = "generate-configs")]
    {
        // Determine output directory:
        // 1. Use ROS_Z_CONFIG_OUTPUT_DIR if set (absolute or relative to CARGO_MANIFEST_DIR)
        // 2. Otherwise use OUT_DIR/ros_z_config
        let config_dir = if let Ok(custom_dir) = std::env::var("ROS_Z_CONFIG_OUTPUT_DIR") {
            let path = PathBuf::from(&custom_dir);
            if path.is_absolute() {
                path
            } else {
                // Resolve relative paths from CARGO_MANIFEST_DIR (package root)
                let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
                manifest_dir.join(path)
            }
        } else {
            let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
            out_dir.join("ros_z_config")
        };

        // Create config directory
        if let Err(e) = std::fs::create_dir_all(&config_dir) {
            eprintln!("Warning: Failed to create config directory: {}", e);
            return;
        }

        // Generate router config JSON5 using ConfigOverride pattern
        let router_json5 = config::generate_json5(&config::router_overrides(), "Router Config");
        if let Err(e) = std::fs::write(
            config_dir.join("DEFAULT_ROSZ_ROUTER_CONFIG.json5"),
            router_json5,
        ) {
            eprintln!("Warning: Failed to write router config: {}", e);
        }

        // Generate session config JSON5 using ConfigOverride pattern
        let session_json5 = config::generate_json5(&config::session_overrides(), "Session Config");
        if let Err(e) = std::fs::write(
            config_dir.join("DEFAULT_ROSZ_SESSION_CONFIG.json5"),
            session_json5,
        ) {
            eprintln!("Warning: Failed to write session config: {}", e);
        }

        println!(
            "cargo:warning=Generated ROS configs: {}",
            config_dir.display()
        );
    }

    #[cfg(not(feature = "generate-configs"))]
    {
        println!(
            "cargo:warning=Config generation disabled. Enable with --features generate-configs"
        );
    }
}
