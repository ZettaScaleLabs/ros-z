use anyhow::Result;
use ros_z::msg::{ZMessage, ZSerializer, ZDeserializer};
use ros_z::pubsub::{ZPub, ZSub};
use std::time::Duration;
use zenoh::sample::Sample;

/// Type-erased publisher trait for Python interop
pub(crate) trait RawPublisher: Send + Sync {
    fn publish_serialized(&self, data: &[u8]) -> Result<()>;
}

/// Type-erased subscriber trait for Python interop
pub(crate) trait RawSubscriber: Send + Sync {
    fn recv_serialized(&self, timeout: Option<Duration>) -> Result<Vec<u8>>;
    fn try_recv_serialized(&self) -> Result<Option<Vec<u8>>>;
}

/// Wrapper for ZPub that implements RawPublisher
pub struct ZPubWrapper<T: ZMessage, S: ZSerializer> {
    inner: ZPub<T, S>,
}

impl<T: ZMessage, S: ZSerializer> ZPubWrapper<T, S> {
    pub fn new(inner: ZPub<T, S>) -> Self {
        Self { inner }
    }
}

impl<T, S> RawPublisher for ZPubWrapper<T, S>
where
    T: ZMessage + 'static,
    S: for<'a> ZSerializer<Input<'a> = &'a T> + Send + Sync + 'static,
{
    fn publish_serialized(&self, data: &[u8]) -> Result<()> {
        self.inner.publish_serialized_message(data).map_err(|e| anyhow::anyhow!(e))
    }
}

/// Wrapper for ZSub that implements RawSubscriber
pub struct ZSubWrapper<T: ZMessage, S: ZDeserializer> {
    inner: ZSub<T, Sample, S>,
}

impl<T: ZMessage, S: ZDeserializer> ZSubWrapper<T, S> {
    pub fn new(inner: ZSub<T, Sample, S>) -> Self {
        Self { inner }
    }
}

impl<T, S> RawSubscriber for ZSubWrapper<T, S>
where
    T: ZMessage + 'static,
    S: for<'a> ZDeserializer<Input<'a> = &'a [u8]> + Send + Sync + 'static,
{
    fn recv_serialized(&self, timeout: Option<Duration>) -> Result<Vec<u8>> {
        let sample = if let Some(t) = timeout {
            // recv_timeout returns Result<S::Output, ...> so we can't use it directly
            // Instead, use recv_serialized which returns Sample
            let queue = self.inner.queue.as_ref()
                .ok_or_else(|| anyhow::anyhow!("Subscriber was built with callback, no queue available"))?;
            queue.recv_timeout(t).map_err(|e| anyhow::anyhow!(e))?
        } else {
            self.inner.recv_serialized().map_err(|e| anyhow::anyhow!(e))?
        };
        Ok(sample.payload().to_bytes().to_vec())
    }

    fn try_recv_serialized(&self) -> Result<Option<Vec<u8>>> {
        // For try_recv, we need to use a zero timeout
        let queue = self.inner.queue.as_ref()
            .ok_or_else(|| anyhow::anyhow!("Subscriber was built with callback, no queue available"))?;

        match queue.try_recv() {
            Ok(sample) => Ok(Some(sample.payload().to_bytes().to_vec())),
            Err(flume::TryRecvError::Empty) => Ok(None),
            Err(flume::TryRecvError::Disconnected) => {
                Err(anyhow::anyhow!("Subscriber queue disconnected"))
            }
        }
    }
}
