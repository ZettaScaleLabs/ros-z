//! Tests for dynamic pub/sub.

use std::sync::Arc;

use crate::dynamic::message::DynamicMessage;
use crate::dynamic::schema::{FieldType, MessageSchema};
use crate::dynamic::serdes::DynamicCdrSerdes;
use crate::msg::{ZDeserializer, ZSerializer};

fn create_test_schema() -> Arc<MessageSchema> {
    MessageSchema::builder("std_msgs/msg/String")
        .field("data", FieldType::String)
        .build()
        .unwrap()
}

fn create_point_schema() -> Arc<MessageSchema> {
    MessageSchema::builder("geometry_msgs/msg/Point")
        .field("x", FieldType::Float64)
        .field("y", FieldType::Float64)
        .field("z", FieldType::Float64)
        .build()
        .unwrap()
}

// Note: Full integration tests for DynPub/DynSub require a Zenoh session.
// These are basic schema validation tests.

#[test]
fn test_builder_creation() {
    // This tests that the schema can be created properly for pub/sub
    let schema = create_test_schema();
    assert_eq!(schema.type_name, "std_msgs/msg/String");
    assert_eq!(schema.fields.len(), 1);
    assert_eq!(schema.fields[0].name, "data");
}

#[test]
fn test_complex_schema_for_pubsub() {
    let vector3 = MessageSchema::builder("geometry_msgs/msg/Vector3")
        .field("x", FieldType::Float64)
        .field("y", FieldType::Float64)
        .field("z", FieldType::Float64)
        .build()
        .unwrap();

    let twist = MessageSchema::builder("geometry_msgs/msg/Twist")
        .field("linear", FieldType::Message(vector3.clone()))
        .field("angular", FieldType::Message(vector3))
        .build()
        .unwrap();

    assert_eq!(twist.type_name, "geometry_msgs/msg/Twist");
    assert_eq!(twist.fields.len(), 2);
}

// Tests for unified pub/sub using DynamicCdrSerdes

#[test]
fn test_dynamic_cdr_serdes_roundtrip() {
    let schema = create_point_schema();
    let mut msg = DynamicMessage::new(&schema);
    msg.set("x", 1.5f64).unwrap();
    msg.set("y", 2.5f64).unwrap();
    msg.set("z", 3.5f64).unwrap();

    // Serialize using DynamicCdrSerdes (ZSerializer trait)
    let bytes = DynamicCdrSerdes::serialize(&msg);
    assert!(!bytes.is_empty());

    // Deserialize using DynamicCdrSerdes (ZDeserializer trait)
    let deserialized = DynamicCdrSerdes::deserialize((&bytes, &schema)).unwrap();

    assert_eq!(deserialized.get::<f64>("x").unwrap(), 1.5);
    assert_eq!(deserialized.get::<f64>("y").unwrap(), 2.5);
    assert_eq!(deserialized.get::<f64>("z").unwrap(), 3.5);
}

#[test]
fn test_dynamic_cdr_serdes_zbuf() {
    use zenoh_buffers::buffer::{Buffer, SplitBuffer};

    let schema = create_test_schema();
    let mut msg = DynamicMessage::new(&schema);
    msg.set("data", "Hello, unified pubsub!").unwrap();

    // Serialize to ZBuf
    let zbuf = DynamicCdrSerdes::serialize_to_zbuf(&msg);
    assert!(zbuf.len() > 0);

    // Convert to bytes and deserialize
    let bytes: Vec<u8> = zbuf.contiguous().to_vec();
    let deserialized = DynamicCdrSerdes::deserialize((&bytes, &schema)).unwrap();

    assert_eq!(
        deserialized.get::<String>("data").unwrap(),
        "Hello, unified pubsub!"
    );
}

#[test]
fn test_dynamic_cdr_serdes_to_buf() {
    let schema = create_point_schema();
    let mut msg = DynamicMessage::new(&schema);
    msg.set("x", 10.0f64).unwrap();
    msg.set("y", 20.0f64).unwrap();
    msg.set("z", 30.0f64).unwrap();

    // Serialize to existing buffer
    let mut buffer = Vec::new();
    DynamicCdrSerdes::serialize_to_buf(&msg, &mut buffer);

    // Should match serialize() output
    let direct = DynamicCdrSerdes::serialize(&msg);
    assert_eq!(buffer, direct);

    // Verify deserialize works
    let deserialized = DynamicCdrSerdes::deserialize((&buffer, &schema)).unwrap();
    assert_eq!(deserialized.get::<f64>("x").unwrap(), 10.0);
}

#[test]
fn test_zmessage_impl_for_dynamic_message() {
    use crate::msg::ZMessage;

    let schema = create_point_schema();
    let mut msg = DynamicMessage::new(&schema);
    msg.set("x", 5.0f64).unwrap();
    msg.set("y", 6.0f64).unwrap();
    msg.set("z", 7.0f64).unwrap();

    // Use ZMessage trait method (must use fully qualified syntax because
    // DynamicMessage has its own serialize method that shadows the trait method)
    let bytes = <DynamicMessage as ZMessage>::serialize(&msg);
    assert!(!bytes.is_empty());

    // Verify CDR header
    assert_eq!(&bytes[0..4], &[0x00, 0x01, 0x00, 0x00]);
}

#[test]
fn test_type_aliases_exist() {
    // Verify type aliases are accessible
    use crate::dynamic::{DynPub, DynPubBuilder, DynSub, DynSubBuilder};

    // These are compile-time checks that the types exist
    fn _check_types() {
        let _: Option<DynPub> = None;
        let _: Option<DynSub> = None;
        let _: Option<DynPubBuilder> = None;
        let _: Option<DynSubBuilder> = None;
    }
}
