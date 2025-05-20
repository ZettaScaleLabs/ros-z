use std::sync::{Arc, atomic::AtomicUsize};

use zenoh::{Result, Session, Wait};

use crate::entity::Entity;
use crate::node::ZNodeBuilder;

#[derive(Debug, Default)]
pub struct GlobalCounter(AtomicUsize);

impl GlobalCounter {
    pub fn increment(&self) -> usize {
        self.0.fetch_add(1, std::sync::atomic::Ordering::AcqRel)
    }
}

pub struct ZContext {
    session: Arc<Session>,
    // Global counter for the participants
    counter: Arc<GlobalCounter>,
}

impl ZContext {
    pub fn new() -> Result<Self> {
        zenoh::init_log_from_env_or("error");
        tracing::trace!("rcl_init");

        let config = zenoh::Config::default();
        let session = zenoh::open(config).wait()?;
        Ok(Self {
            session: Arc::new(session),
            counter: Arc::new(GlobalCounter::default()),
        })
    }

    pub fn create_node<'a>(&self, name: &'a str) -> ZNodeBuilder<'a> {
        ZNodeBuilder {
            name,
            session: self.session.clone(),
            counter: self.counter.clone(),
        }
    }

    pub fn shutdown(&self) -> Result<()> {
        self.session.close().wait()
    }
}
