use crate::node::ZNode;
use std::sync::Arc;
use zenoh::{Result, Session, Wait};

pub struct ZContext {
    session: Arc<Session>,
}

impl ZContext {
    pub fn new() -> Result<Self> {
        zenoh::init_log_from_env_or("error");
        tracing::trace!("rcl_init");

        let config = zenoh::Config::default();
        let session = zenoh::open(config).wait()?;
        Ok(Self {
            session: Arc::new(session),
        })
    }

    pub fn create_node(&self) -> ZNode {
        ZNode::new(self.session.clone())
    }

    pub fn shutdown(&self) -> Result<()> {
        self.session.close().wait()
    }
}
