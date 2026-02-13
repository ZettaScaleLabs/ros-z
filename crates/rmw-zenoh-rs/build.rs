extern crate bindgen;

use std::{env, path::PathBuf};

/// Packages needed for bindgen (rmw bindings)
const BINDGEN_PACKAGES: &[&str] = &[
    "rmw",
    "rcutils",
    "rosidl_runtime_c",
    "rosidl_typesupport_interface",
    "fastcdr",
];

/// Additional packages needed for CXX bridge (type support serialization)
const CXX_EXTRA_PACKAGES: &[&str] = &[
    "rosidl_typesupport_fastrtps_c",
    "rosidl_typesupport_fastrtps_cpp",
];

fn main() {
    // Single source of truth: AMENT_PREFIX_PATH
    let ament_prefix =
        env::var("AMENT_PREFIX_PATH").unwrap_or_else(|_| "/opt/ros/jazzy".to_string());

    // Detect Humble by checking for type_description_interfaces (not present in Humble)
    let is_humble = !ament_prefix.split(':').any(|prefix| {
        PathBuf::from(prefix)
            .join("include/type_description_interfaces")
            .exists()
    });

    // Helper to find package include dirs from ament prefixes
    let find_include_dirs = |packages: &[&str]| -> Vec<PathBuf> {
        let mut dirs = Vec::new();
        for pkg in packages {
            for prefix in ament_prefix.split(':') {
                let pkg_path = PathBuf::from(prefix).join(format!("include/{}", pkg));
                if pkg_path.exists() {
                    dirs.push(pkg_path);
                    break;
                }
            }
        }
        dirs
    };

    // Find include directories for bindgen
    let bindgen_include_dirs = find_include_dirs(BINDGEN_PACKAGES);
    let include_args: Vec<String> = bindgen_include_dirs
        .iter()
        .map(|p| format!("-I{}", p.display()))
        .collect();

    let bindgen_out_path = PathBuf::from("src");

    // Add ROS_DISTRO_HUMBLE macro for conditional compilation
    let mut clang_args = include_args;
    if is_humble {
        clang_args.push("-DROS_DISTRO_HUMBLE".to_string());
    }

    // Generate bindings
    let bindings = bindgen::Builder::default()
        .header("binding.hpp")
        .clang_args(&clang_args)
        // Allow utility functions
        .allowlist_function("rcutils_.*")
        .allowlist_function("rmw_get_zero_initialized_.*")
        .allowlist_function("rmw_names_and_types_.*")
        .allowlist_function("rmw_check_zero_rmw_string_array")
        .allowlist_function("rmw_security_options_.*")
        .allowlist_function("rmw_discovery_options_.*")
        .allowlist_function("rmw_validate_.*")
        .allowlist_function("rmw_event_fini")
        .allowlist_function("rmw_topic_endpoint_info_.*")
        // Allow types and constants
        .allowlist_type("rmw_.*")
        .allowlist_type("rcutils_.*")
        .allowlist_type("rosidl_.*")
        .allowlist_var("RMW_.*")
        // Blocklist problematic functions that require unavailable headers
        .blocklist_function("rmw_take_dynamic_message")
        .blocklist_function("rmw_take_dynamic_message_with_info")
        .blocklist_function("rmw_serialization_support_init")
        .blocklist_type("rosidl_dynamic_typesupport_serialization_support_t")
        .blocklist_type("rosidl_dynamic_typesupport_serialization_support_impl_t")
        .derive_default(true)
        .generate()
        .expect("Unable to generate bindings");

    bindings
        .write_to_file(bindgen_out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");

    // Build CXX bridge for type support serialization
    // Needs both bindgen packages and extra type support packages
    let mut cxx_include_dirs = bindgen_include_dirs;
    cxx_include_dirs.extend(find_include_dirs(CXX_EXTRA_PACKAGES));

    cxx_build::bridge("src/type_support.rs")
        .file("src/serde_bridge.cc")
        .include("include")
        .includes(&cxx_include_dirs)
        .std("c++17")
        .compile("serde_bridge");

    // Link libraries from ament prefix
    for prefix in ament_prefix.split(':') {
        let lib_path = PathBuf::from(prefix).join("lib");
        if lib_path.exists() {
            println!("cargo:rustc-link-search=native={}", lib_path.display());
        }
    }

    println!("cargo:rustc-link-lib=dylib=rmw");
    println!("cargo:rustc-link-lib=dylib=rcutils");
    println!("cargo:rustc-link-lib=dylib=rosidl_runtime_c");
    println!("cargo:rustc-link-lib=dylib=fastcdr");
    println!("cargo:rustc-link-lib=dylib=rosidl_typesupport_fastrtps_c");
    println!("cargo:rustc-link-lib=dylib=rosidl_typesupport_fastrtps_cpp");

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/type_support.rs");
    println!("cargo:rerun-if-changed=src/serde_bridge.cc");
    println!("cargo:rerun-if-changed=include/serde_bridge.h");
    println!("cargo:rerun-if-changed=binding.hpp");
}
