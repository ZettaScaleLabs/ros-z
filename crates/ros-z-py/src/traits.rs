use anyhow::Result;
use ros_z::pubsub::{ZPub, ZSub};
use std::time::Duration;
use zenoh::bytes::ZBytes;
use zenoh::sample::Sample;

use crate::raw_bytes::{RawBytesCdrSerdes, RawBytesMessage, RawBytesService};

/// Type-erased publisher trait for Python interop
pub(crate) trait RawPublisher: Send + Sync {
    /// Publish pre-serialized data
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

// -- Generic wrappers using RawBytesMessage (universal, no per-type code needed) --

/// Generic publisher wrapper using RawBytesMessage
pub struct GenericPubWrapper {
    inner: ZPub<RawBytesMessage, RawBytesCdrSerdes>,
}

impl GenericPubWrapper {
    pub fn new(inner: ZPub<RawBytesMessage, RawBytesCdrSerdes>) -> Self {
        Self { inner }
    }
}

impl RawPublisher for GenericPubWrapper {
    fn publish(&self, data: ZBytes) -> Result<()> {
        self.inner
            .publish_serialized(data)
            .map_err(|e| anyhow::anyhow!(e))
    }
}

/// Generic subscriber wrapper using RawBytesMessage
pub struct GenericSubWrapper {
    inner: ZSub<RawBytesMessage, Sample, RawBytesCdrSerdes>,
}

impl GenericSubWrapper {
    pub fn new(inner: ZSub<RawBytesMessage, Sample, RawBytesCdrSerdes>) -> Self {
        Self { inner }
    }
}

impl RawSubscriber for GenericSubWrapper {
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

// -- Generic service wrappers using RawBytesService --

use ros_z::service::{QueryKey, ZClient, ZServer};

/// Type-erased client trait for Python interop
pub(crate) trait RawClient: Send + Sync {
    fn send_request_serialized(&self, data: &[u8]) -> Result<()>;
    fn take_response_serialized(&self, timeout: Option<Duration>) -> Result<Vec<u8>>;
    fn try_take_response_serialized(&self) -> Result<Option<Vec<u8>>>;
}

/// Type-erased server trait for Python interop
pub(crate) trait RawServer: Send + Sync {
    fn take_request_serialized(&self) -> Result<(QueryKey, Vec<u8>)>;
    fn send_response_serialized(&self, data: &[u8], key: &QueryKey) -> Result<()>;
}

/// Generic client wrapper using RawBytesService
pub struct GenericClientWrapper {
    inner: ZClient<RawBytesService>,
}

impl GenericClientWrapper {
    pub fn new(inner: ZClient<RawBytesService>) -> Self {
        Self { inner }
    }
}

impl RawClient for GenericClientWrapper {
    fn send_request_serialized(&self, data: &[u8]) -> Result<()> {
        let request = RawBytesMessage(data.to_vec());

        let rt = tokio::runtime::Handle::try_current()
            .or_else(|_| tokio::runtime::Runtime::new().map(|rt| rt.handle().clone()))?;

        rt.block_on(async {
            self.inner
                .send_request(&request)
                .await
                .map_err(|e| anyhow::anyhow!("Failed to send request: {}", e))
        })
    }

    fn take_response_serialized(&self, timeout: Option<Duration>) -> Result<Vec<u8>> {
        let timeout_duration = timeout.unwrap_or(Duration::from_secs(3600));
        let response = self
            .inner
            .take_response_timeout(timeout_duration)
            .map_err(|e| anyhow::anyhow!("Failed to receive response: {}", e))?;

        Ok(response.0)
    }

    fn try_take_response_serialized(&self) -> Result<Option<Vec<u8>>> {
        match self.inner.take_response_timeout(Duration::from_millis(1)) {
            Ok(response) => Ok(Some(response.0)),
            Err(e) => {
                let err_str = e.to_string();
                if err_str.contains("timeout")
                    || err_str.contains("Timeout")
                    || err_str.contains("No sample available")
                {
                    Ok(None)
                } else {
                    Err(anyhow::anyhow!("Failed to receive response: {}", e))
                }
            }
        }
    }
}

/// Generic server wrapper using RawBytesService
pub struct GenericServerWrapper {
    inner: std::sync::Mutex<ZServer<RawBytesService>>,
}

impl GenericServerWrapper {
    pub fn new(inner: ZServer<RawBytesService>) -> Self {
        Self {
            inner: std::sync::Mutex::new(inner),
        }
    }
}

impl RawServer for GenericServerWrapper {
    fn take_request_serialized(&self) -> Result<(QueryKey, Vec<u8>)> {
        let mut server = self
            .inner
            .lock()
            .map_err(|e| anyhow::anyhow!("Failed to lock server: {}", e))?;

        let (key, request) = server
            .take_request()
            .map_err(|e| anyhow::anyhow!("Failed to receive request: {}", e))?;

        Ok((key, request.0))
    }

    fn send_response_serialized(&self, data: &[u8], key: &QueryKey) -> Result<()> {
        let response = RawBytesMessage(data.to_vec());

        let mut server = self
            .inner
            .lock()
            .map_err(|e| anyhow::anyhow!("Failed to lock server: {}", e))?;

        server
            .send_response(&response, key)
            .map_err(|e| anyhow::anyhow!("Failed to send response: {}", e))
    }
}
