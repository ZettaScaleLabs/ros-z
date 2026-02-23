//! Async dynamic topic subscriber
//!
//! Wraps a blocking [`ZSub`] with a flume channel so callers can poll
//! messages from async contexts without blocking the executor.

use std::{sync::Arc, time::Duration};

use flume::Receiver;
use zenoh::sample::Sample;

use crate::{
    dynamic::{DynamicCdrSerdes, DynamicMessage, MessageSchema, MessageSchemaTypeDescription},
    node::ZNode,
    pubsub::ZSub,
};

/// Manages subscription to a single topic with dynamic message types.
///
/// Automatically discovers the message schema via the type description service,
/// then bridges the blocking [`ZSub`] to a [`flume`] channel for use in async code.
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
    /// Create a new dynamic subscriber with automatic schema discovery.
    ///
    /// # Arguments
    ///
    /// * `node` - ROS node with type description service enabled
    /// * `topic` - Topic name to subscribe to (e.g., `/chatter`)
    /// * `discovery_timeout` - Maximum time to wait for schema discovery
    ///
    /// # Errors
    ///
    /// Returns an error if no publishers are found within the timeout or if
    /// schema discovery fails.
    pub async fn new(
        node: &ZNode,
        topic: &str,
        discovery_timeout: Duration,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let (subscriber, schema) = node.create_dyn_sub_auto(topic, discovery_timeout).await?;

        let type_hash = schema
            .compute_type_hash()
            .map(|h| h.to_rihs_string())
            .unwrap_or_else(|_| "unknown".to_string());

        let (tx, rx) = flume::unbounded();

        let subscriber = Arc::new(subscriber);
        let sub_clone = subscriber.clone();
        tokio::task::spawn_blocking(move || {
            loop {
                match sub_clone.recv() {
                    Ok(msg) => {
                        if tx.send(msg).is_err() {
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

    /// Get the discovered message schema.
    pub fn schema(&self) -> &MessageSchema {
        &self.schema
    }

    /// Get the RIHS01 type hash.
    pub fn type_hash(&self) -> &str {
        &self.type_hash
    }

    /// Try to receive a message without blocking.
    pub fn try_recv(&self) -> Result<Option<DynamicMessage>, flume::TryRecvError> {
        match self.message_rx.try_recv() {
            Ok(msg) => Ok(Some(msg)),
            Err(flume::TryRecvError::Empty) => Ok(None),
            Err(e) => Err(e),
        }
    }
}
