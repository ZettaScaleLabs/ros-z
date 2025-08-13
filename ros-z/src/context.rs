use std::sync::{Arc, atomic::AtomicUsize};

use zenoh::{Result, Session, Wait};

use crate::{Builder, graph::Graph, node::ZNodeBuilder};

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
        // FIXME
        let config = match std::env::var("ROSZ_CONFIG_FILE") {
            Ok(path) => zenoh::Config::from_file(path)?,
            Err(_) => zenoh::Config::default(),
        };
        let session = zenoh::open(config).wait()?;
        let domain_id = self.domain_id;
        let graph = Arc::new(Graph::new(&session, domain_id)?);
        Ok(ZContext {
            session: Arc::new(session),
            counter: Arc::new(GlobalCounter::default()),
            domain_id,
            graph,
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
    graph: Arc<Graph>,
}

impl ZContext {
    pub fn create_node<S: AsRef<str>>(&self, name: S) -> ZNodeBuilder {
        ZNodeBuilder {
            domain_id: self.domain_id,
            name: name.as_ref().to_owned(),
            namespace: "".to_string(),
            session: self.session.clone(),
            counter: self.counter.clone(),
            graph: self.graph.clone(),
        }
    }

    pub fn shutdown(&self) -> Result<()> {
        self.session.close().wait()
    }
}
