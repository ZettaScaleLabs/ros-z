use ros_z::{Builder, Result, MessageTypeInfo};

#[cfg(feature = "protobuf")]
use ros_z::msg::ProtobufSerdes;

#[cfg(feature = "protobuf")]
use std::time::Duration;

// Manually define a protobuf version of Vector3 for demonstration
// In a real implementation, this would be auto-generated from .proto files
#[cfg(feature = "protobuf")]
#[derive(Clone, PartialEq, prost::Message)]
pub struct Vector3Proto {
    #[prost(double, tag = "1")]
    pub x: f64,
    #[prost(double, tag = "2")]
    pub y: f64,
    #[prost(double, tag = "3")]
    pub z: f64,
}

// Implement MessageTypeInfo to use with ros-z
#[cfg(feature = "protobuf")]
impl MessageTypeInfo for Vector3Proto {
    fn type_name() -> &'static str {
        "geometry_msgs::msg::dds_::Vector3_"
    }

    fn type_hash() -> ros_z::entity::TypeHash {
        ros_z::entity::TypeHash::from_rihs_string(
            "RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d"
        ).expect("Invalid RIHS hash")
    }
}

// Implement ZMessage to specify protobuf serialization
#[cfg(feature = "protobuf")]
impl ros_z::msg::ZMessage for Vector3Proto {
    type Serdes = ProtobufSerdes<Vector3Proto>;
}

fn main() -> Result<()> {
    #[cfg(not(feature = "protobuf"))]
    {
        println!("This example requires the 'protobuf' feature.");
        println!("Run with: cargo run --package ros-z --example vector3_protobuf_demo --features protobuf");
        return Ok(());
    }

    #[cfg(feature = "protobuf")]
    {
        zenoh::init_log_from_env_or("info");

        println!("\n=== Protobuf Serialization Demo ===");
        println!("This demonstrates using protobuf with ros-z");
        println!("====================================\n");

        let ctx = ros_z::context::ZContextBuilder::default().build()?;
        let node = ctx.create_node("protobuf_demo").build()?;

        // Create a publisher using ProtobufSerdes
        let zpub = node.create_pub::<Vector3Proto>("/vector_proto")
            .with_type_info(Vector3Proto::type_info())
            .with_serdes::<ProtobufSerdes<Vector3Proto>>()  // ← Use protobuf serialization
            .build()?;

        println!("Publisher created with Protobuf serialization");
        println!("Publishing messages...\n");

        // Publish some messages
        for i in 0..5 {
            let msg = Vector3Proto {
                x: i as f64,
                y: (i as f64) * 2.0,
                z: (i as f64) * 3.0,
            };

            zpub.publish(&msg)?;
            println!("Published (Protobuf): {:?}", msg);

            std::thread::sleep(Duration::from_millis(500));
        }

        println!("\n✅ Successfully demonstrated protobuf serialization!");
        println!("\nKey points:");
        println!("1. Vector3Proto derives prost::Message for protobuf serialization");
        println!("2. Used .with_serdes::<ProtobufSerdes<T>>() to select protobuf format");
        println!("3. Messages are serialized as protobuf instead of CDR");
        println!("\nNote: To use this with ros-z-msgs, we need to:");
        println!("- Generate .proto files from .msg files (Phase 2 of proposal)");
        println!("- Use prost to generate Rust structs with prost::Message derives");
        println!("- Add MessageTypeInfo implementations to the generated types");
    }

    Ok(())
}
