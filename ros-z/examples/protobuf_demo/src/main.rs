use ros_z::{Result, msg::ProtobufSerdes};
use std::time::Duration;

// Import ROS-generated protobuf messages
use ros_z_msgs::proto::geometry_msgs::Vector3 as Vector3Proto;

// Include custom protobuf message generated from sensor_data.proto
pub mod sensor_data {
    include!(concat!(env!("OUT_DIR"), "/examples.rs"));
}

use sensor_data::SensorData;

fn main() -> Result<()> {
    zenoh::init_log_from_env_or("info");

    println!("\n=== Protobuf Serialization Demo ===");
    println!("This demonstrates two ways to use protobuf with ros-z:");
    println!("1. ROS messages with protobuf serialization (from ros-z-msgs)");
    println!("2. Custom protobuf messages (from .proto files)");
    println!("=====================================================\n");

    let ctx = ros_z::context::ZContextBuilder::default().build()?;
    let node = ctx.create_node("protobuf_demo").build()?;

    // Part 1: ROS message with protobuf serialization
    println!("--- Part 1: ROS geometry_msgs/Vector3 with Protobuf ---");
    let ros_pub = node.create_pub::<Vector3Proto>("/vector_proto")
        .with_serdes::<ProtobufSerdes<Vector3Proto>>()
        .build()?;

    println!("Publishing ROS Vector3 messages...\n");
    for i in 0..3 {
        let msg = Vector3Proto {
            x: i as f64,
            y: (i as f64) * 2.0,
            z: (i as f64) * 3.0,
        };

        ros_pub.publish(&msg)?;
        println!("  Published Vector3: x={}, y={}, z={}", msg.x, msg.y, msg.z);
        std::thread::sleep(Duration::from_millis(500));
    }

    // Part 2: Custom protobuf message
    println!("\n--- Part 2: Custom SensorData message (pure protobuf) ---");
    let custom_pub = node.create_pub::<SensorData>("/sensor_data")
        .with_serdes::<ProtobufSerdes<SensorData>>()
        .build()?;

    println!("Publishing custom SensorData messages...\n");
    for i in 0..3 {
        let msg = SensorData {
            sensor_id: format!("sensor_{}", i),
            temperature: 20.0 + (i as f64) * 0.5,
            humidity: 45.0 + (i as f64) * 2.0,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs() as i64,
        };

        custom_pub.publish(&msg)?;
        println!("  Published SensorData: id={}, temp={:.1}°C, humidity={:.1}%, ts={}",
                 msg.sensor_id, msg.temperature, msg.humidity, msg.timestamp);
        std::thread::sleep(Duration::from_millis(500));
    }

    println!("\n✅ Successfully demonstrated both protobuf approaches!");
    println!("\nKey points:");
    println!("1. ROS messages (Vector3Proto): Auto-generated from ros-z-msgs with MessageTypeInfo");
    println!("2. Custom messages (SensorData): Generated from your own .proto files");
    println!("3. Both use .with_serdes::<ProtobufSerdes<T>>() for protobuf serialization");
    println!("4. ros-z is transport-agnostic - works with ANY protobuf message!");

    Ok(())
}
