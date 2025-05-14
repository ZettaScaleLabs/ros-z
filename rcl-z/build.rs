use std::env;
use std::path::PathBuf;

const INCLUDE_PACKAGES: &[&str] = &[
    "rcl",
    "rmw",
    "rcutils",
    "rcl_yaml_param_parser",
    "rosidl_runtime_c",
    "type_description_interfaces",
    "service_msgs",
    "builtin_interfaces",
    "rosidl_typesupport_interface",
    "rosidl_dynamic_typesupport",
    "fastcdr",
];

fn main() {
    // Get AMENT_PREFIX_PATH from the environment
    let ament_prefix = env::var("AMENT_PREFIX_PATH").expect("AMENT_PREFIX_PATH is missing!");

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
    // println!("cargo:warning=include_args: {:?}", include_args);

    // Build the bindgen builder
    let mut builder = bindgen::Builder::default()
        .header("wrapper.hpp")
        .generate_comments(false)
        .clang_args(include_args)
        .allowlist_function("rcl_.*")
        .allowlist_type("rcl_.*")
        .allowlist_var("RCL_.*")
        .derive_default(true);

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
    cxx_build::bridge("src/type_support.rs")
        .file("src/serde_bridge.cc")
        .include("include")
        .includes(include_dirs)
        .std("c++17")
        .compile("serde_bridge");

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/type_support.rs");
    println!("cargo:rerun-if-changed=src/serde_bridge.cc");
    println!("cargo:rerun-if-changed=include/serde_bridge.h");

    println!("cargo:rustc-link-search=native=/usr/local/lib");
    println!("cargo:rustc-link-search=native={ament_prefix}/lib/");
    println!("cargo:rustc-link-lib=dylib=rmw");
    println!("cargo:rustc-link-lib=dylib=rcutils");
    println!("cargo:rustc-link-lib=dylib=fastcdr");
    println!("cargo:rustc-link-lib=dylib=rosidl_runtime_c");
    println!("cargo:rustc-link-lib=dylib=rosidl_typesupport_fastrtps_c");
    println!("cargo:rustc-link-lib=dylib=rosidl_typesupport_fastrtps_cpp");
}
