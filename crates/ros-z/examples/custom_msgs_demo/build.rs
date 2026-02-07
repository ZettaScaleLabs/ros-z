use std::path::PathBuf;
use std::env;

fn main() -> anyhow::Result<()> {
    let out_dir = PathBuf::from(env::var("OUT_DIR")?);

    // Generate user messages from ROS_Z_MSG_PATH environment variable
    // The generated code will reference standard types (geometry_msgs, builtin_interfaces)
    // from ros_z_msgs using fully qualified paths
    ros_z_codegen::generate_user_messages(&out_dir, false)?;

    println!("cargo:rerun-if-env-changed=ROS_Z_MSG_PATH");
    Ok(())
}
