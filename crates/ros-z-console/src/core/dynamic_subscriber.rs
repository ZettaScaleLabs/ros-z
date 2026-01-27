//! Dynamic topic subscriber for any ROS message type
//!
//! Provides automatic schema discovery and message reception for topics
//! without compile-time knowledge of message types.

use std::{sync::Arc, time::Duration};

use flume::Receiver;
use ros_z::{
    node::ZNode,
    dynamic::{DynamicMessage, MessageSchema, DynamicCdrSerdes},
    pubsub::ZSub,
};
use zenoh::sample::Sample;

/// Manages subscription to a single topic with dynamic message types
pub struct DynamicTopicSubscriber {
    /// Topic name being subscribed to
    #[allow(dead_code)]
    pub topic: String,
    /// Discovered message schema
    schema: Arc<MessageSchema>,
    /// RIHS01 type hash for compatibility verification
    type_hash: String,
    /// Channel for receiving messages asynchronously
    message_rx: Receiver<DynamicMessage>,
    /// Subscriber handle (kept alive to maintain subscription)
    _subscriber: Arc<ZSub<DynamicMessage, Sample, DynamicCdrSerdes>>,
}

impl DynamicTopicSubscriber {
    /// Create a new dynamic subscriber with automatic schema discovery
    ///
    /// # Arguments
    ///
    /// * `node` - ROS node with type description service enabled
    /// * `topic` - Topic name to subscribe to (e.g., "/chatter")
    /// * `discovery_timeout` - Maximum time to wait for schema discovery
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - No publishers found on topic within timeout
    /// - Schema discovery fails
    /// - Subscriber creation fails
    pub async fn new(
        node: &ZNode,
        topic: &str,
        discovery_timeout: Duration,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        // Use the node's auto-discovery method to get subscriber and schema
        let (subscriber, schema) = node.create_dyn_sub_auto(topic, discovery_timeout).await?;

        // Compute type hash for informational purposes
        use ros_z::dynamic::MessageSchemaTypeDescription;
        let type_hash = schema
            .compute_type_hash()
            .map(|h| h.to_rihs_string())
            .unwrap_or_else(|_| "unknown".to_string());

        // Create channel for async message handling
        let (tx, rx) = flume::unbounded();

        // Spawn background task to forward messages to channel
        let subscriber = Arc::new(subscriber);
        let sub_clone = subscriber.clone();
        tokio::task::spawn_blocking(move || {
            loop {
                match sub_clone.recv() {
                    Ok(msg) => {
                        if tx.send(msg).is_err() {
                            // Receiver dropped, exit task
                            break;
                        }
                    }
                    Err(e) => {
                        tracing::warn!("Subscriber recv error: {}", e);
                        break;
                    }
                }
            }
        });

        Ok(Self {
            topic: topic.to_string(),
            schema,
            type_hash,
            message_rx: rx,
            _subscriber: subscriber,
        })
    }

    /// Get the discovered message schema
    pub fn schema(&self) -> &MessageSchema {
        &self.schema
    }

    /// Get the RIHS01 type hash
    pub fn type_hash(&self) -> &str {
        &self.type_hash
    }

    /// Receive the next message (blocking)
    ///
    /// Blocks until a message is available or the channel is closed.
    #[allow(dead_code)]
    pub fn recv(&self) -> Result<DynamicMessage, flume::RecvError> {
        self.message_rx.recv()
    }

    /// Try to receive a message without blocking
    ///
    /// Returns `Ok(Some(msg))` if a message is available,
    /// `Ok(None)` if no message is ready,
    /// or `Err` if the channel is closed.
    pub fn try_recv(&self) -> Result<Option<DynamicMessage>, flume::TryRecvError> {
        match self.message_rx.try_recv() {
            Ok(msg) => Ok(Some(msg)),
            Err(flume::TryRecvError::Empty) => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// Check if there are any messages waiting
    #[allow(dead_code)]
    pub fn has_messages(&self) -> bool {
        !self.message_rx.is_empty()
    }

    /// Get the number of messages currently buffered
    #[allow(dead_code)]
    pub fn message_count(&self) -> usize {
        self.message_rx.len()
    }
}

#[cfg(test)]
mod tests {
    // Note: Integration tests would require a running ROS system
    // Unit tests are limited for this component
}
