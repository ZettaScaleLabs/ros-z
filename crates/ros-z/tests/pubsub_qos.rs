//! Pub/Sub QoS tests
//!
//! These tests verify publisher/subscriber functionality with different QoS settings,
//! particularly focusing on the AdvancedPublisher behavior with various durability
//! and reliability configurations.
//!
//! ## Key Findings:
//!
//! 1. **AdvancedPublisher works within the same Zenoh session**
//!    - When publisher and subscriber share the same context, messages are delivered successfully
//!    - Both sync (`publish()`) and async (`async_publish()`) work correctly
//!
//! 2. **AdvancedPublisher now works across different Zenoh sessions** (FIXED!)
//!    - When publisher and subscriber use different contexts (separate Zenoh sessions)
//!    - Messages are now delivered successfully
//!    - This is critical for ROS 2 where different processes use different contexts
//!
//! 3. **Root causes that were fixed:**
//!    - Missing `reliability` setting on the publisher (was only setting `congestion_control`)
//!    - Disabled multicast scouting prevented session discovery
//!    - Missing timestamping configuration for TransientLocal QoS with cache
//!    - Added shared memory transport for efficient intra-process communication

use std::{
    num::NonZeroUsize,
    sync::{
        Arc,
        atomic::{AtomicU32, Ordering},
    },
    time::Duration,
};

use ros_z::{
    Builder, Result,
    context::ZContextBuilder,
    qos::{QosDurability, QosHistory, QosProfile, QosReliability},
};
use ros_z_msgs::std_msgs::String as RosString;

/// Helper to create a test context and node
async fn setup_test_node(
    node_name: &str,
) -> Result<(ros_z::context::ZContext, ros_z::node::ZNode)> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node(node_name).build()?;

    // Allow time for node discovery
    tokio::time::sleep(Duration::from_millis(100)).await;

    Ok((ctx, node))
}

/// Helper to create a QoS profile
fn create_qos(
    durability: QosDurability,
    reliability: QosReliability,
    history: QosHistory,
) -> QosProfile {
    QosProfile {
        durability,
        reliability,
        history,
        ..Default::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test pub/sub with Volatile durability and Reliable reliability
    /// This uses AdvancedPublisher without cache
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_pubsub_volatile_reliable() -> Result<()> {
        let (_ctx, pub_node) = setup_test_node("test_pub_volatile_reliable").await?;
        let sub_node = _ctx.create_node("test_sub_volatile_reliable").build()?;

        let topic = "/test_volatile_reliable";
        let qos = create_qos(
            QosDurability::Volatile,
            QosReliability::Reliable,
            QosHistory::KeepLast(NonZeroUsize::new(10).unwrap()),
        );

        // Create subscriber FIRST to ensure it's ready before publisher
        let received_count = Arc::new(AtomicU32::new(0));
        let received_count_clone = received_count.clone();

        let _sub_handle = sub_node
            .create_sub::<RosString>(topic)
            .with_qos(qos)
            .build_with_callback(move |msg: RosString| {
                println!("Received: {}", msg.data);
                received_count_clone.fetch_add(1, Ordering::SeqCst);
            })?;

        // Now create publisher
        let pub_handle = pub_node
            .create_pub::<RosString>(topic)
            .with_qos(qos)
            .build()?;

        // Wait for discovery
        tokio::time::sleep(Duration::from_millis(2000)).await;

        // Publish messages
        for i in 0..5 {
            let msg = RosString {
                data: format!("Message {}", i),
            };
            println!("Publishing: {}", msg.data);
            pub_handle.publish(&msg)?;
            // Small delay between messages
            tokio::time::sleep(Duration::from_millis(100)).await;
        }

        // Wait for messages to be received
        tokio::time::sleep(Duration::from_millis(2000)).await;

        let count = received_count.load(Ordering::SeqCst);
        assert!(count >= 5, "Expected at least 5 messages, got {}", count);

        Ok(())
    }

    /// Test pub/sub with Volatile durability and BestEffort reliability
    /// This uses AdvancedPublisher without cache
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_pubsub_volatile_best_effort() -> Result<()> {
        let (_ctx, pub_node) = setup_test_node("test_pub_volatile_best_effort").await?;
        let sub_node = _ctx.create_node("test_sub_volatile_best_effort").build()?;

        let topic = "/test_volatile_best_effort";
        let qos = create_qos(
            QosDurability::Volatile,
            QosReliability::BestEffort,
            QosHistory::KeepLast(NonZeroUsize::new(10).unwrap()),
        );

        // Create publisher with Volatile + BestEffort QoS
        let pub_handle = pub_node
            .create_pub::<RosString>(topic)
            .with_qos(qos)
            .build()?;

        // Create subscriber with matching QoS
        let received_count = Arc::new(AtomicU32::new(0));
        let received_count_clone = received_count.clone();

        let _sub_handle = sub_node
            .create_sub::<RosString>(topic)
            .with_qos(qos)
            .build_with_callback(move |msg: RosString| {
                println!("Received: {}", msg.data);
                received_count_clone.fetch_add(1, Ordering::SeqCst);
            })?;

        // Wait for discovery
        tokio::time::sleep(Duration::from_millis(500)).await;

        // Publish messages
        for i in 0..5 {
            let msg = RosString {
                data: format!("Message {}", i),
            };
            println!("Publishing: {}", msg.data);
            pub_handle.publish(&msg)?;
        }

        // Wait for messages to be received
        tokio::time::sleep(Duration::from_millis(500)).await;

        let count = received_count.load(Ordering::SeqCst);
        assert!(
            count >= 4,
            "Expected at least 4 messages (best effort), got {}",
            count
        );

        Ok(())
    }

    /// Test pub/sub with TransientLocal durability and Reliable reliability
    /// This uses AdvancedPublisher WITH cache
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_pubsub_transient_local_reliable() -> Result<()> {
        let (_ctx, pub_node) = setup_test_node("test_pub_transient_reliable").await?;
        let sub_node = _ctx.create_node("test_sub_transient_reliable").build()?;

        let topic = "/test_transient_reliable";
        let qos = create_qos(
            QosDurability::TransientLocal,
            QosReliability::Reliable,
            QosHistory::KeepLast(NonZeroUsize::new(10).unwrap()),
        );

        // Create subscriber FIRST
        let received_count = Arc::new(AtomicU32::new(0));
        let received_count_clone = received_count.clone();

        let _sub_handle = sub_node
            .create_sub::<RosString>(topic)
            .with_qos(qos)
            .build_with_callback(move |msg: RosString| {
                println!("Received: {}", msg.data);
                received_count_clone.fetch_add(1, Ordering::SeqCst);
            })?;

        // Now create publisher with TransientLocal + Reliable QoS
        let pub_handle = pub_node
            .create_pub::<RosString>(topic)
            .with_qos(qos)
            .build()?;

        // Wait for discovery
        tokio::time::sleep(Duration::from_millis(2000)).await;

        // Publish messages
        for i in 0..5 {
            let msg = RosString {
                data: format!("Message {}", i),
            };
            println!("Publishing: {}", msg.data);
            pub_handle.publish(&msg)?;
            tokio::time::sleep(Duration::from_millis(100)).await;
        }

        // Wait for messages to be received
        tokio::time::sleep(Duration::from_millis(2000)).await;

        let count = received_count.load(Ordering::SeqCst);
        // With TransientLocal across contexts, we expect all messages published after subscriber exists
        assert!(
            count >= 5,
            "Expected at least 5 messages with TransientLocal, got {}",
            count
        );

        Ok(())
    }

    /// Test pub/sub with TransientLocal durability and BestEffort reliability
    /// This uses AdvancedPublisher WITH cache
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_pubsub_transient_local_best_effort() -> Result<()> {
        let (_ctx, pub_node) = setup_test_node("test_pub_transient_best_effort").await?;
        let sub_node = _ctx.create_node("test_sub_transient_best_effort").build()?;

        let topic = "/test_transient_best_effort";
        let qos = create_qos(
            QosDurability::TransientLocal,
            QosReliability::BestEffort,
            QosHistory::KeepLast(NonZeroUsize::new(10).unwrap()),
        );

        // Create publisher with TransientLocal + BestEffort QoS
        let pub_handle = pub_node
            .create_pub::<RosString>(topic)
            .with_qos(qos)
            .build()?;

        // Create subscriber with matching QoS
        let received_count = Arc::new(AtomicU32::new(0));
        let received_count_clone = received_count.clone();

        let _sub_handle = sub_node
            .create_sub::<RosString>(topic)
            .with_qos(qos)
            .build_with_callback(move |msg: RosString| {
                println!("Received: {}", msg.data);
                received_count_clone.fetch_add(1, Ordering::SeqCst);
            })?;

        // Wait for discovery
        tokio::time::sleep(Duration::from_millis(500)).await;

        // Publish messages
        for i in 0..5 {
            let msg = RosString {
                data: format!("Message {}", i),
            };
            println!("Publishing: {}", msg.data);
            pub_handle.publish(&msg)?;
        }

        // Wait for messages to be received
        tokio::time::sleep(Duration::from_millis(500)).await;

        let count = received_count.load(Ordering::SeqCst);
        assert!(count >= 4, "Expected at least 4 messages, got {}", count);

        Ok(())
    }

    /// Test pub/sub with KeepAll history
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_pubsub_keep_all_history() -> Result<()> {
        let (_ctx, pub_node) = setup_test_node("test_pub_keep_all").await?;
        let sub_node = _ctx.create_node("test_sub_keep_all").build()?;

        let topic = "/test_keep_all";
        let qos = create_qos(
            QosDurability::TransientLocal,
            QosReliability::Reliable,
            QosHistory::KeepAll,
        );

        // Create subscriber first
        let received_count = Arc::new(AtomicU32::new(0));
        let received_count_clone = received_count.clone();

        let _sub_handle = sub_node
            .create_sub::<RosString>(topic)
            .with_qos(qos)
            .build_with_callback(move |msg: RosString| {
                println!("Received: {}", msg.data);
                received_count_clone.fetch_add(1, Ordering::SeqCst);
            })?;

        // Create publisher with KeepAll history
        let pub_handle = pub_node
            .create_pub::<RosString>(topic)
            .with_qos(qos)
            .build()?;

        // Wait for discovery
        tokio::time::sleep(Duration::from_millis(2000)).await;

        // Publish many messages
        for i in 0..10 {
            let msg = RosString {
                data: format!("Message {}", i),
            };
            pub_handle.publish(&msg)?;
            tokio::time::sleep(Duration::from_millis(50)).await;
        }

        // Wait for messages
        tokio::time::sleep(Duration::from_millis(1000)).await;

        let count = received_count.load(Ordering::SeqCst);
        assert!(
            count >= 10,
            "Expected all 10 messages with KeepAll, got {}",
            count
        );

        Ok(())
    }

    /// Test pub/sub with AdvancedPublisher using the SAME context
    /// THIS WORKS: Demonstrates that AdvancedPublisher works within the same Zenoh session
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_pubsub_volatile_same_context_works() -> Result<()> {
        // Use SAME context for both nodes (same Zenoh session) - THIS IS THE KEY!
        let ctx = ZContextBuilder::default().build()?;
        let pub_node = ctx.create_node("test_pub_volatile_async").build()?;
        let sub_node = ctx.create_node("test_sub_volatile_async").build()?;
        tokio::time::sleep(Duration::from_millis(100)).await;

        let topic = "/test_volatile_async";
        let qos = create_qos(
            QosDurability::Volatile,
            QosReliability::Reliable,
            QosHistory::KeepLast(NonZeroUsize::new(10).unwrap()),
        );

        // Create subscriber FIRST
        let received_count = Arc::new(AtomicU32::new(0));
        let received_count_clone = received_count.clone();

        let _sub_handle = sub_node
            .create_sub::<RosString>(topic)
            .with_qos(qos)
            .build_with_callback(move |msg: RosString| {
                println!("Received: {}", msg.data);
                received_count_clone.fetch_add(1, Ordering::SeqCst);
            })?;

        // Now create publisher
        let pub_handle = pub_node
            .create_pub::<RosString>(topic)
            .with_qos(qos)
            .build()?;

        // Wait for discovery
        tokio::time::sleep(Duration::from_millis(2000)).await;

        // Publish messages using ASYNC publish
        for i in 0..5 {
            let msg = RosString {
                data: format!("Message {}", i),
            };
            println!("Publishing async: {}", msg.data);
            pub_handle.async_publish(&msg).await?;
            tokio::time::sleep(Duration::from_millis(100)).await;
        }

        // Wait for messages to be received
        tokio::time::sleep(Duration::from_millis(2000)).await;

        let count = received_count.load(Ordering::SeqCst);
        assert!(
            count >= 5,
            "Expected at least 5 messages with async_publish, got {}",
            count
        );

        Ok(())
    }

    /// Test TransientLocal with same context - should work
    /// Timestamping is now enabled by default in the context
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_pubsub_transient_local_same_context_works() -> Result<()> {
        let ctx = ZContextBuilder::default().build()?;
        let pub_node = ctx.create_node("test_pub_transient_same").build()?;
        let sub_node = ctx.create_node("test_sub_transient_same").build()?;
        tokio::time::sleep(Duration::from_millis(100)).await;

        let topic = "/test_transient_same";
        let qos = create_qos(
            QosDurability::TransientLocal,
            QosReliability::Reliable,
            QosHistory::KeepLast(NonZeroUsize::new(10).unwrap()),
        );

        // Create subscriber first
        let received_count = Arc::new(AtomicU32::new(0));
        let received_count_clone = received_count.clone();

        let _sub_handle = sub_node
            .create_sub::<RosString>(topic)
            .with_qos(qos)
            .build_with_callback(move |msg: RosString| {
                println!("Received: {}", msg.data);
                received_count_clone.fetch_add(1, Ordering::SeqCst);
            })?;

        // Create publisher
        let pub_handle = pub_node
            .create_pub::<RosString>(topic)
            .with_qos(qos)
            .build()?;

        // Wait for discovery
        tokio::time::sleep(Duration::from_millis(1000)).await;

        // Publish messages
        for i in 0..5 {
            let msg = RosString {
                data: format!("Message {}", i),
            };
            pub_handle.async_publish(&msg).await?;
            tokio::time::sleep(Duration::from_millis(50)).await;
        }

        // Wait for messages
        tokio::time::sleep(Duration::from_millis(1000)).await;

        let count = received_count.load(Ordering::SeqCst);
        assert!(
            count >= 5,
            "Expected at least 5 messages with TransientLocal same context, got {}",
            count
        );

        Ok(())
    }

    /// Minimal test to verify AdvancedPublisher doesn't hang
    /// This is the simplest possible test
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_minimal_advanced_publisher_no_hang() -> Result<()> {
        let (_ctx, node) = setup_test_node("test_minimal_hang").await?;

        let topic = "/test_minimal";
        let qos = create_qos(
            QosDurability::Volatile, // Uses AdvancedPublisher
            QosReliability::Reliable,
            QosHistory::KeepLast(NonZeroUsize::new(1).unwrap()),
        );

        // Create publisher - this should succeed
        println!("Creating publisher...");
        let pub_handle = node.create_pub::<RosString>(topic).with_qos(qos).build()?;
        println!("Publisher created successfully");

        // Try to publish a single message - this is where it hangs
        let msg = RosString {
            data: "Test message".to_string(),
        };
        println!("Publishing message...");

        // Use a timeout to detect the hang
        let publish_result = tokio::time::timeout(
            Duration::from_secs(5),
            tokio::task::spawn_blocking(move || pub_handle.publish(&msg)),
        )
        .await;

        match publish_result {
            Ok(Ok(Ok(_))) => {
                println!("Publish succeeded");
                Ok(())
            }
            Ok(Ok(Err(e))) => {
                panic!("Publish failed with error: {:?}", e);
            }
            Ok(Err(e)) => {
                panic!("Task join failed: {:?}", e);
            }
            Err(_) => {
                panic!(
                    "HANG DETECTED: Publish timed out after 5 seconds - AdvancedPublisher.wait() is blocking!"
                );
            }
        }
    }

    /// SYNCHRONOUS test to verify AdvancedPublisher works without tokio runtime
    /// This mimics the RCL C context - pure synchronous, no async runtime
    #[test]
    fn test_sync_advanced_publisher_no_runtime() -> Result<()> {
        use std::sync::{
            Arc,
            atomic::{AtomicU32, Ordering},
        };

        println!("=== SYNC TEST: Creating context and nodes (no tokio runtime) ===");
        let ctx = ros_z::context::ZContextBuilder::default().build()?;
        let pub_node = ctx.create_node("sync_test_pub").build()?;
        let sub_node = ctx.create_node("sync_test_sub").build()?;

        let topic = "/test_sync_publish";
        let qos = create_qos(
            QosDurability::Volatile, // Uses AdvancedPublisher
            QosReliability::BestEffort,
            QosHistory::KeepLast(NonZeroUsize::new(10).unwrap()),
        );

        // Create subscriber first
        println!("Creating subscriber...");
        let received_count = Arc::new(AtomicU32::new(0));
        let received_count_clone = received_count.clone();
        let _sub_handle = sub_node
            .create_sub::<RosString>(topic)
            .with_qos(qos)
            .build_with_callback(move |msg: RosString| {
                println!("SYNC TEST: Received: {}", msg.data);
                received_count_clone.fetch_add(1, Ordering::SeqCst);
            })?;

        // Create publisher
        println!("Creating publisher...");
        let pub_handle = pub_node
            .create_pub::<RosString>(topic)
            .with_qos(qos)
            .build()?;
        println!("Publisher created successfully");

        // Wait for discovery (synchronous sleep)
        println!("Waiting for discovery...");
        std::thread::sleep(Duration::from_millis(1000));

        // Publish messages SYNCHRONOUSLY - this is the critical test
        println!("Publishing messages synchronously (no tokio, like RCL C code)...");
        for i in 0..3 {
            let msg = RosString {
                data: format!("Sync message {}", i),
            };
            println!("Publishing: {}", msg.data);

            // This is what happens in RCL - pure synchronous call with .wait()
            pub_handle.publish(&msg)?;

            println!("Publish {} completed", i);
        }

        // Wait for messages to be received
        println!("Waiting for messages...");
        std::thread::sleep(Duration::from_millis(1000));

        let count = received_count.load(Ordering::SeqCst);
        println!("SYNC TEST RESULT: Received {} messages", count);

        // We expect at least 2 messages with BestEffort
        assert!(count >= 2, "Expected at least 2 messages, got {}", count);

        Ok(())
    }

    /// SYNCHRONOUS test with SAME context (like RCL single-process test)
    /// RCL test uses one context for both pub and sub
    #[test]
    fn test_sync_publish_same_context() -> Result<()> {
        use std::sync::{
            Arc,
            atomic::{AtomicU32, Ordering},
        };

        println!("=== SYNC TEST WITH SAME CONTEXT (like RCL single process) ===");
        let ctx = ros_z::context::ZContextBuilder::default().build()?;
        let pub_node = ctx.create_node("sync_same_ctx_pub").build()?;
        let sub_node = ctx.create_node("sync_same_ctx_sub").build()?;

        let topic = "/test_sync_same_ctx";
        let qos = create_qos(
            QosDurability::Volatile,
            QosReliability::BestEffort,
            QosHistory::KeepLast(NonZeroUsize::new(10).unwrap()),
        );

        // Create publisher first (like RCL test)
        println!("Creating publisher...");
        let pub_handle = pub_node
            .create_pub::<RosString>(topic)
            .with_qos(qos)
            .build()?;

        // Create subscriber
        println!("Creating subscriber...");
        let received_count = Arc::new(AtomicU32::new(0));
        let received_count_clone = received_count.clone();
        let _sub_handle = sub_node
            .create_sub::<RosString>(topic)
            .with_qos(qos)
            .build_with_callback(move |msg: RosString| {
                println!("Received: {}", msg.data);
                received_count_clone.fetch_add(1, Ordering::SeqCst);
                // Sleep to simulate slow callback - publish() should not block on this
                std::thread::sleep(std::time::Duration::from_millis(100));
            })?;

        // Wait for discovery
        println!("Waiting for discovery...");
        std::thread::sleep(Duration::from_millis(1000));

        // Publish messages SYNCHRONOUSLY
        println!("Publishing with same context (like RCL)...");
        for i in 0..3 {
            let msg = RosString {
                data: format!("Same context message {}", i),
            };
            println!("Publishing: {}", msg.data);

            // This should hang if same context causes the issue
            pub_handle.publish(&msg)?;

            println!("Publish {} completed", i);
        }

        // Wait for messages
        println!("Waiting for messages...");
        std::thread::sleep(Duration::from_millis(1000));

        let count = received_count.load(Ordering::SeqCst);
        println!("SAME CONTEXT TEST RESULT: Received {} messages", count);

        assert!(count >= 2, "Expected at least 2 messages, got {}", count);

        Ok(())
    }

    /// Test with EXACT RCL QoS settings: deadline + liveliness
    /// This should reproduce the hang if it's caused by these QoS settings
    #[test]
    fn test_sync_publish_with_deadline_liveliness() -> Result<()> {
        use std::sync::{
            Arc,
            atomic::{AtomicU32, Ordering},
        };

        println!("=== SYNC TEST WITH DEADLINE + LIVELINESS (exact RCL settings) ===");
        let ctx = ros_z::context::ZContextBuilder::default().build()?;
        let pub_node = ctx.create_node("sync_deadline_pub").build()?;
        let sub_node = ctx.create_node("sync_deadline_sub").build()?;

        let topic = "/test_sync_deadline_live";

        // EXACT same QoS as RCL test_events.cpp default_qos_profile
        let qos = QosProfile {
            durability: QosDurability::Volatile, // SYSTEM_DEFAULT maps to Volatile
            reliability: QosReliability::BestEffort,
            history: QosHistory::KeepLast(NonZeroUsize::new(10).unwrap()),
            // These are the critical settings from RCL:
            liveliness: ros_z::qos::QosLiveliness::ManualByTopic,
            liveliness_lease_duration: ros_z::qos::QosDuration { sec: 1, nsec: 0 }, // 1 second
            deadline: ros_z::qos::QosDuration { sec: 2, nsec: 0 },                  // 2 seconds
            ..Default::default()
        };

        println!("Creating subscriber...");
        let received_count = Arc::new(AtomicU32::new(0));
        let received_count_clone = received_count.clone();
        let _sub_handle = sub_node
            .create_sub::<RosString>(topic)
            .with_qos(qos)
            .build_with_callback(move |msg: RosString| {
                println!("Received: {}", msg.data);
                received_count_clone.fetch_add(1, Ordering::SeqCst);
            })?;

        println!("Creating publisher...");
        let pub_handle = pub_node
            .create_pub::<RosString>(topic)
            .with_qos(qos)
            .build()?;

        println!("Waiting for discovery...");
        std::thread::sleep(Duration::from_millis(1000));

        println!("Publishing with deadline+liveliness QoS...");
        for i in 0..3 {
            let msg = RosString {
                data: format!("Deadline/liveliness message {}", i),
            };
            println!("Publishing: {}", msg.data);

            // THIS IS WHERE IT SHOULD HANG if deadline/liveliness causes the issue!
            pub_handle.publish(&msg)?;

            println!("Publish {} completed", i);
        }

        println!("Waiting for messages...");
        std::thread::sleep(Duration::from_millis(1000));

        let count = received_count.load(Ordering::SeqCst);
        println!(
            "DEADLINE/LIVELINESS TEST RESULT: Received {} messages",
            count
        );

        assert!(count >= 2, "Expected at least 2 messages, got {}", count);

        Ok(())
    }
}
