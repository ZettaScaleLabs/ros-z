use anyhow::Result;
use ros_z::msg::{ZDeserializer, ZMessage, ZSerializer};
use ros_z::pubsub::{ZPub, ZSub};
use std::time::Duration;
use zenoh::bytes::ZBytes;
use zenoh::sample::Sample;

/// Type-erased publisher trait for Python interop
pub(crate) trait RawPublisher: Send + Sync {
    /// Publish pre-serialized data
    ///
    /// Accepts ZBytes - callers should use `.into()` to convert from:
    /// - `ZBuf` (zero-copy path)
    /// - `&[u8]` or `Vec<u8>`
    fn publish(&self, data: ZBytes) -> Result<()>;
}

/// Type-erased subscriber trait for Python interop
pub(crate) trait RawSubscriber: Send + Sync {
    /// Receive a Sample (zero-copy path - preferred)
    fn recv_sample(&self, timeout: Option<Duration>) -> Result<Sample>;
    /// Try to receive a Sample without blocking (zero-copy path - preferred)
    fn try_recv_sample(&self) -> Result<Option<Sample>>;
    /// Receive pre-serialized bytes (legacy method)
    fn recv_serialized(&self, timeout: Option<Duration>) -> Result<Vec<u8>>;
    /// Try to receive pre-serialized bytes without blocking (legacy method)
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
    fn publish(&self, data: ZBytes) -> Result<()> {
        self.inner
            .publish_serialized(data)
            .map_err(|e| anyhow::anyhow!(e))
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
    fn recv_sample(&self, timeout: Option<Duration>) -> Result<Sample> {
        if let Some(t) = timeout {
            let queue = self.inner.queue.as_ref().ok_or_else(|| {
                anyhow::anyhow!("Subscriber was built with callback, no queue available")
            })?;
            queue
                .recv_timeout(t)
                .ok_or_else(|| anyhow::anyhow!("Receive timeout"))
        } else {
            self.inner.recv_serialized().map_err(|e| anyhow::anyhow!(e))
        }
    }

    fn try_recv_sample(&self) -> Result<Option<Sample>> {
        let queue = self.inner.queue.as_ref().ok_or_else(|| {
            anyhow::anyhow!("Subscriber was built with callback, no queue available")
        })?;

        Ok(queue.try_recv())
    }

    fn recv_serialized(&self, timeout: Option<Duration>) -> Result<Vec<u8>> {
        let sample = self.recv_sample(timeout)?;
        Ok(sample.payload().to_bytes().to_vec())
    }

    fn try_recv_serialized(&self) -> Result<Option<Vec<u8>>> {
        match self.try_recv_sample()? {
            Some(sample) => Ok(Some(sample.payload().to_bytes().to_vec())),
            None => Ok(None),
        }
    }
}
