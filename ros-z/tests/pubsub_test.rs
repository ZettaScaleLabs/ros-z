use std::{thread, time::Duration};

use ros_z::{Builder, context::ZContextBuilder};
use ros_z_msgs::std_msgs::ByteMultiArray;
use serde_json::json;
use zenoh_buffers::{
    ZBuf,
    buffer::{Buffer, SplitBuffer},
};

#[test]
fn test_bytemultiarray_pubsub_with_zbuf() {
    // Test that pub/sub works correctly with ByteMultiArray using ZBuf

    let ctx = ZContextBuilder::default()
        .disable_multicast_scouting()
        .with_json("connect/endpoints", json!([]))
        .build()
        .expect("Failed to create context");

    let publisher_handle = thread::spawn({
        let ctx = ctx.clone();
        move || {
            let node = ctx
                .create_node("test_publisher")
                .build()
                .expect("Failed to create node");

            let publisher = node
                .create_pub::<ByteMultiArray>("test_topic")
                .build()
                .expect("Failed to create publisher");

            // Create message with ZBuf
            let mut buffer = vec![0xAA; 16];
            buffer[0..8].copy_from_slice(&42u64.to_le_bytes()); // timestamp

            let msg = ByteMultiArray {
                data: ZBuf::from(buffer),
                ..Default::default()
            };

            println!("Publishing ByteMultiArray with ZBuf...");
            publisher.publish(&msg).expect("Failed to publish");

            // Wait a bit
            thread::sleep(Duration::from_millis(100));
        }
    });

    let subscriber_handle = thread::spawn({
        let ctx = ctx.clone();
        move || {
            let node = ctx
                .create_node("test_subscriber")
                .build()
                .expect("Failed to create node");

            let subscriber = node
                .create_sub::<ByteMultiArray>("test_topic")
                .build()
                .expect("Failed to create subscriber");

            println!("Waiting for message...");
            match subscriber.recv_timeout(Duration::from_secs(2)) {
                Ok(received_msg) => {
                    println!(
                        "Received message successfully: data len = {}",
                        received_msg.data.len()
                    );
                    assert_eq!(received_msg.data.len(), 16);
                    // Check the timestamp
                    let timestamp_bytes = &received_msg.data.contiguous()[0..8];
                    let timestamp = u64::from_le_bytes(timestamp_bytes.try_into().unwrap());
                    assert_eq!(timestamp, 42);
                }
                Err(e) => {
                    panic!("Failed to receive message: {}", e);
                }
            }
        }
    });

    publisher_handle.join().expect("Publisher thread panicked");
    subscriber_handle
        .join()
        .expect("Subscriber thread panicked");
}
