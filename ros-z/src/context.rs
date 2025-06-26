use std::sync::{Arc, atomic::AtomicUsize};

use zenoh::{Result, Session, Wait};

use crate::{node::ZNodeBuilder, Builder};

#[derive(Debug, Default)]
pub struct GlobalCounter(AtomicUsize);

impl GlobalCounter {
    pub fn increment(&self) -> usize {
        self.0.fetch_add(1, std::sync::atomic::Ordering::AcqRel)
    }
}

#[derive(Debug, Default)]
pub struct ZContextBuilder {
    domain_id: usize,
}

impl Builder for ZContextBuilder {
    type Output = ZContext;
    fn build(self) -> Result<ZContext> {
        zenoh::init_log_from_env_or("error");
        tracing::trace!("rcl_init");

        let config = zenoh::Config::default();
        let session = zenoh::open(config).wait()?;
        Ok(ZContext {
            session: Arc::new(session),
            counter: Arc::new(GlobalCounter::default()),
            domain_id: self.domain_id,
        })
    }
}

impl ZContextBuilder {
    pub fn with_domain_id(mut self, domain_id: usize) -> Self {
        self.domain_id = domain_id;
        self
    }
}

pub struct ZContext {
    session: Arc<Session>,
    // Global counter for the participants
    counter: Arc<GlobalCounter>,
    domain_id: usize,
}

impl ZContext {
    pub fn create_node<S: AsRef<str>>(&self, name: S) -> ZNodeBuilder {
        ZNodeBuilder {
            domain_id: self.domain_id,
            name: name.as_ref().to_owned(),
            namespace: None,
            session: self.session.clone(),
            counter: self.counter.clone(),
        }
    }

    pub fn shutdown(&self) -> Result<()> {
        self.session.close().wait()
    }
}
