fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut config = prost_build::Config::new();
    // Enable serde support for generated protobuf messages
    config.type_attribute(".", "#[derive(serde::Serialize, serde::Deserialize)]");
    config.compile_protos(&["proto/sensor_data.proto"], &["proto/"])?;
    println!("cargo:rerun-if-changed=proto/sensor_data.proto");
    Ok(())
}
