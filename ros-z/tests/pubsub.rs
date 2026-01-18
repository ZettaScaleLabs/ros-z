use std::time::Duration;

use ros_z::{Builder, TypeHash, context::ZContextBuilder, ros_msg::MessageTypeInfo};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
struct TestMessage {
    data: Vec<u8>,
    counter: u64,
}

impl MessageTypeInfo for TestMessage {
    fn type_name() -> &'static str {
        "test_msgs::msg::dds_::TestMessage_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero()
    }
}

impl ros_z::ros_msg::WithTypeInfo for TestMessage {}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn test_basic_pubsub() {
    let ctx = ZContextBuilder::default()
        .build()
        .expect("Failed to create context");
    let node = ctx
        .create_node("test_node")
        .build()
        .expect("Failed to create node");

    // Create publisher
    let publisher = node
        .create_pub::<TestMessage>("/test_topic")
        .build()
        .unwrap();

    // Create subscriber
    let subscriber = node
        .create_sub::<TestMessage>("/test_topic")
        .build()
        .unwrap();

    // Give some time for discovery
    tokio::time::sleep(Duration::from_millis(100)).await;

    // Publish message
    let msg = TestMessage {
        data: vec![1, 2, 3, 4, 5],
        counter: 42,
    };
    publisher.publish(&msg).unwrap();

    // Receive message
    let received = subscriber.recv_timeout(Duration::from_secs(1));
    assert!(received.is_ok());
    let received_msg = received.unwrap();
    assert_eq!(received_msg, msg);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn test_multiple_messages() {
    let ctx = ZContextBuilder::default()
        .build()
        .expect("Failed to create context");
    let node = ctx
        .create_node("multi_msg_node")
        .build()
        .expect("Failed to create node");

    let publisher = node
        .create_pub::<TestMessage>("/multi_topic")
        .build()
        .unwrap();

    let subscriber = node
        .create_sub::<TestMessage>("/multi_topic")
        .build()
        .unwrap();

    tokio::time::sleep(Duration::from_millis(100)).await;

    // Publish multiple messages
    for i in 0..5 {
        let msg = TestMessage {
            data: vec![i as u8; 100],
            counter: i,
        };
        publisher.publish(&msg).unwrap();
    }

    // Receive all messages
    for i in 0..5 {
        let received = subscriber.recv_timeout(Duration::from_secs(1));
        assert!(received.is_ok());
        let received_msg = received.unwrap();
        assert_eq!(received_msg.counter, i);
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn test_large_payload() {
    let ctx = ZContextBuilder::default()
        .build()
        .expect("Failed to create context");
    let node = ctx
        .create_node("large_payload_node")
        .build()
        .expect("Failed to create node");

    let publisher = node
        .create_pub::<TestMessage>("/large_topic")
        .build()
        .unwrap();

    let subscriber = node
        .create_sub::<TestMessage>("/large_topic")
        .build()
        .unwrap();

    tokio::time::sleep(Duration::from_millis(100)).await;

    // Publish 1MB message
    let msg = TestMessage {
        data: vec![0xAB; 1024 * 1024],
        counter: 999,
    };
    publisher.publish(&msg).unwrap();

    let received = subscriber.recv_timeout(Duration::from_secs(2));
    assert!(received.is_ok());
    let received_msg = received.unwrap();
    assert_eq!(received_msg.counter, 999);
    assert_eq!(received_msg.data.len(), 1024 * 1024);
    assert_eq!(received_msg.data[0], 0xAB);
}
