fn main() -> Result<(), Box<dyn std::error::Error>> {
    prost_build::compile_protos(&["proto/sensor_data.proto"], &["proto/"])?;
    println!("cargo:rerun-if-changed=proto/sensor_data.proto");
    Ok(())
}
