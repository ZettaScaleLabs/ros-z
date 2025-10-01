use std::{env, fs::File, io::Write, path::Path};

use which::which;

fn main() -> std::io::Result<()> {
    let out_path = Path::new(&env::var("OUT_DIR").unwrap()).join("example.rs");

    // If protoc is not installed, we cheat because building protoc from source
    // with protobuf-src is way too long
    if which("protoc").is_err() {
        const PROTO_WITH_IMPL: &str = r#"
#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Entity {
    #[prost(uint32, tag = "1")]
    pub id: u32,
    #[prost(string, tag = "2")]
    pub name: ::prost::alloc::string::String,
}

// Auto-generated ZMessage implementation using macro
ros_z::impl_zmessage_protobuf!(Entity);
"#;
        File::create(out_path)?.write_all(PROTO_WITH_IMPL.as_bytes())?;
        return Ok(());
    }

    // Generate protobuf code
    prost_build::compile_protos(&["examples/example.proto"], &["examples/"])?;

    // Read the generated file and append ZMessage implementation using macro
    let generated_content = std::fs::read_to_string(&out_path)?;
    let content_with_impl = format!("{}\n\n// Auto-generated ZMessage implementation using macro\nros_z::impl_zmessage_protobuf!(Entity);\n", generated_content);
    std::fs::write(out_path, content_with_impl)?;

    Ok(())
}
