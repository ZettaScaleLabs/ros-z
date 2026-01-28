fn main() {
    // Declare custom cfg flags for package availability
    // These are set by ros-z-msgs build.rs when packages are actually found
    println!("cargo::rustc-check-cfg=cfg(has_example_interfaces)");
    println!("cargo::rustc-check-cfg=cfg(has_test_msgs)");
}
