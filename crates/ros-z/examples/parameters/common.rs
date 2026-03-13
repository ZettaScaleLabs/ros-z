use ros_z::{
    Builder, Result,
    context::{ZContext, ZContextBuilder},
};

const DEFAULT_ENDPOINT: &str = "tcp/localhost:7447";

pub fn init() {
    zenoh::init_log_from_env_or("error");
}

pub fn create_context() -> Result<ZContext> {
    ZContextBuilder::default()
        .with_connect_endpoints([DEFAULT_ENDPOINT])
        .build()
}
