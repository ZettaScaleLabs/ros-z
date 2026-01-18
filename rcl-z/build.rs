use std::{env, path::PathBuf};

const INCLUDE_PACKAGES: &[&str] = &[
    "rcl",
    "rmw",
    "rcutils",
    "rcl_yaml_param_parser",
    "rosidl_runtime_c",
    "service_msgs",
    "builtin_interfaces",
    "rosidl_typesupport_interface",
    "rosidl_dynamic_typesupport",
    "fastcdr",
];

fn main() {
    // Declare custom cfg for auto-detection
    println!("cargo:rustc-check-cfg=cfg(has_test_msgs)");
    println!("cargo:rustc-check-cfg=cfg(has_type_description_interfaces)");
    println!("cargo:rustc-check-cfg=cfg(ros_humble)");

    // Check if ROS is installed
    let ros_installed = env::var("AMENT_PREFIX_PATH").is_ok();
    let humble_compat = env::var("CARGO_FEATURE_HUMBLE_COMPAT").is_ok();

    // Get AMENT_PREFIX_PATH for the rest of the build (only if ROS is installed)
    if !ros_installed {
        println!(
            "cargo:warning=ROS not detected (no AMENT_PREFIX_PATH) - using feature flag for API selection"
        );
        // Pure Rust mode: user chooses via feature
        if humble_compat {
            println!("cargo:rustc-cfg=ros_humble");
        }
        println!("cargo:warning=Skipping ROS-dependent build steps");
        return;
    }

    let ament_prefix = env::var("AMENT_PREFIX_PATH").unwrap();

    // Check if type_description_interfaces is available (not in Humble)
    let has_type_description = ament_prefix.split(':').any(|prefix| {
        PathBuf::from(prefix)
            .join("include/type_description_interfaces")
            .exists()
    });

    let is_humble = !has_type_description;

    if is_humble && !humble_compat {
        panic!(
            "ROS Humble detected but 'humble_compat' feature not enabled. Build with --features humble_compat"
        );
    }

    if !is_humble && humble_compat {
        panic!(
            "humble_compat feature enabled but ROS Jazzy+ detected. The humble_compat feature is only for ROS Humble."
        );
    }

    // Check for distro feature conflicts
    if !is_humble && env::var("CARGO_FEATURE_HUMBLE").is_ok() {
        panic!(
            "humble feature enabled but ROS Jazzy+ detected. The humble feature is only for ROS Humble."
        );
    }

    if has_type_description {
        println!("cargo:rustc-cfg=has_type_description_interfaces");
        println!("cargo:warning=type_description_interfaces found - ROS Jazzy+ detected");
    } else {
        println!("cargo:warning=type_description_interfaces not found - ROS Humble detected");
    }

    // Emit cfg based on what mode we're in
    if is_humble {
        println!("cargo:rustc-cfg=ros_humble");
        println!("cargo:warning=Using Humble-compatible codegen");
        if humble_compat {
            println!("cargo:rustc-cfg=humble_compat_enabled");
        }
    } else {
        println!("cargo:warning=Using modern codegen");
    }

    if has_type_description {
        println!("cargo:rustc-cfg=has_type_description_interfaces");
        println!("cargo:warning=type_description_interfaces found - ROS Jazzy+ detected");
    } else {
        println!("cargo:warning=type_description_interfaces not found - ROS Humble detected");
    }

    // Emit cfg based on what mode we're in
    let use_humble_mode = is_humble || humble_compat;
    if use_humble_mode {
        println!("cargo:rustc-cfg=ros_humble");
    }

    // Check FastCDR version - Humble uses FastCDR 1.0.x which doesn't have CdrVersion enum
    // Jazzy+ uses FastCDR 1.1.0+ which has CdrVersion::DDS_CDR
    let has_fastcdr_v2 = ament_prefix.split(':').any(|prefix| {
        let version_file = PathBuf::from(prefix).join("share/fastcdr/package.xml");
        if let Ok(content) = std::fs::read_to_string(&version_file) {
            // Check if version is >= 1.1.0 (simple heuristic: contains "1.1" or "1.2" etc)
            content.contains("<version>1.1")
                || content.contains("<version>1.2")
                || content.contains("<version>2.")
        } else {
            false
        }
    });

    if has_fastcdr_v2 || !is_humble {
        println!("cargo:rustc-cfg=has_fastcdr_v2");
        if has_fastcdr_v2 {
            println!("cargo:warning=FastCDR v1.1+ detected - using CdrVersion API");
        } else {
            println!(
                "cargo:warning=ROS Jazzy+ detected - using CdrVersion API despite FastCDR version"
            );
        }
    } else {
        println!("cargo:warning=FastCDR v1.0 detected - using legacy API (Humble)");
    }
    println!("cargo:rustc-check-cfg=cfg(has_fastcdr_v2)");

    // Collect all include paths
    let mut include_args = Vec::new();
    for pkg in INCLUDE_PACKAGES {
        for prefix in ament_prefix.split(':') {
            let pkg_path = PathBuf::from(prefix).join(format!("include/{}", pkg));
            if pkg_path.exists() {
                include_args.push(format!("-I{}", pkg_path.display()));
                break;
            }
        }
    }

    // Add type_description_interfaces if available
    if has_type_description {
        for prefix in ament_prefix.split(':') {
            let pkg_path = PathBuf::from(prefix).join("include/type_description_interfaces");
            if pkg_path.exists() {
                include_args.push(format!("-I{}", pkg_path.display()));
                break;
            }
        }
        // Define macro for C++ code
        include_args.push("-DHAS_TYPE_DESCRIPTION_INTERFACES".to_string());
    }
    // println!("cargo:warning=include_args: {:?}", include_args);

    // Build the bindgen builder
    let mut builder = bindgen::Builder::default()
        .header("wrapper.hpp")
        .generate_comments(false)
        .clang_args(include_args)
        .allowlist_function("rcl_.*")
        .allowlist_function("rmw_get_gid_for_publisher")
        .allowlist_function("rmw_publisher_count_matched_subscriptions")
        .allowlist_function("rmw_subscription_count_matched_publishers")
        .allowlist_type("rcl_.*")
        .blocklist_type("rmw_qos_profile_s")
        .blocklist_type("rmw_qos_.*_policy_e")
        .allowlist_var("RCL_.*")
        .allowlist_var("RMW_.*")
        // .no_default("rmw_qos_profile_s")
        .default_enum_style(bindgen::EnumVariation::Rust {
            non_exhaustive: false,
        })
        .prepend_enum_name(false)
        .derive_debug(true)
        .derive_default(true)
        .derive_partialeq(true)
        .derive_eq(true);

    // Only add type_description_interfaces support if available
    if has_type_description {
        builder = builder.allowlist_function("rosidl_typesupport_c__get_service_type_support_handle__type_description_interfaces__srv__GetTypeDescription");
    }

    // Allowlist Jazzy types
    builder = builder
        .allowlist_type("rcl_timer_call_info_t")
        .allowlist_function("rcl_get_type_description_services")
        .allowlist_function("rcl_type_description_service_init")
        .allowlist_function("rcl_type_description_service_fini")
        .allowlist_var("RCL_SERVICE_INTROSPECTION_PUBLISHER")
        .allowlist_var("RCL_SERVICE_INTROSPECTION_SUBSCRIPTION")
        .allowlist_var("RCL_SERVICE_INTROSPECTION_OFF");

    let blocked_functions = [
        "rcl_init_options_init",
        "rcl_init",
        "rcl_shutdown",
        "rcl_context_fini",
        "rcl_node_init",
        "rcl_publisher_init",
        "rcl_publish",
        "rcl_publish_serialized_message",
        "rcl_subscription_init",
        "rcl_wait_set_add_subscription",
        "rcl_wait_set_init",
        "rcl_wait_set_clear",
        "rcl_wait",
        "rcl_take",
        "rcl_wait_set_fini",
        "rcl_publisher_fini",
        "rcl_get_zero_initialized_wait_set",
        "rcl_get_zero_initialized_context",
        "rcl_subscription_fini",
        "rcl_node_fini",
        "rcl_init_options_get_allocator",
    ];

    for func in blocked_functions {
        builder = builder.blocklist_function(func);
    }

    // Generate the bindings
    let bindings = builder.generate().expect("Unable to generate bindings");

    // Write them to $OUT_DIR/bindings.rs
    let out_path = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");

    // CXX
    let include_pkgs = [
        "rmw",
        "fastcdr",
        "rcutils",
        "rosidl_runtime_c",
        "rosidl_typesupport_interface",
        "rosidl_typesupport_fastrtps_c",
        "rosidl_typesupport_fastrtps_cpp",
    ];
    let mut include_dirs = Vec::new();
    for pkg in include_pkgs {
        for prefix in ament_prefix.split(':') {
            let pkg_path = std::path::PathBuf::from(prefix).join(format!("include/{}", pkg));
            if pkg_path.exists() {
                include_dirs.push(pkg_path.display().to_string());
                break;
            }
        }
    }
    // cxx_build::bridge("src/lib.rs")
    let mut cxx_builder = cxx_build::bridge("src/type_support.rs");
    cxx_builder
        .file("src/serde_bridge.cc")
        .include("include")
        .includes(include_dirs)
        .std("c++17");

    // Pass FastCDR version flag to C++ compiler
    if has_fastcdr_v2 {
        cxx_builder.define("HAS_FASTCDR_V2", None);
    }

    cxx_builder.compile("serde_bridge");

    println!("cargo:rerun-if-changed=wrapper.hpp");
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/type_support.rs");
    println!("cargo:rerun-if-changed=src/serde_bridge.cc");
    println!("cargo:rerun-if-changed=include/serde_bridge.h");

    println!("cargo:rustc-link-search=native=/usr/local/lib");
    println!("cargo:rustc-link-search=native={ament_prefix}/lib/");

    println!("cargo:rustc-link-lib=dylib=rcl");
    println!("cargo:rustc-link-lib=dylib=rmw");
    println!("cargo:rustc-link-lib=dylib=rcutils");
    println!("cargo:rustc-link-lib=dylib=fastcdr");
    println!("cargo:rustc-link-lib=dylib=rosidl_runtime_c");
    println!("cargo:rustc-link-lib=dylib=rosidl_typesupport_fastrtps_c");
    println!("cargo:rustc-link-lib=dylib=rosidl_typesupport_fastrtps_cpp");

    // Only link type_description_interfaces if available
    if has_type_description {
        println!("cargo:rustc-link-lib=dylib=type_description_interfaces__rosidl_typesupport_c");
    }

    // Link test_msgs for tests
    #[cfg(feature = "test-msgs")]
    {
        // Explicit feature: always link (fail if not available)
        println!("cargo:rustc-link-lib=dylib=test_msgs__rosidl_generator_c");
        println!("cargo:rustc-link-lib=dylib=test_msgs__rosidl_typesupport_c");
        println!("cargo:rustc-link-lib=dylib=test_msgs__rosidl_typesupport_fastrtps_c");
        println!("cargo:rustc-link-lib=dylib=test_msgs__rosidl_typesupport_fastrtps_cpp");
    }
}
