use ros_z::{
    Builder, Result,
    context::{ZContext, ZContextBuilder},
};

pub fn init() {
    zenoh::init_log_from_env_or("error");
}

/// Build a context connected to a Zenoh router.
///
/// Uses the `ZENOH_ROUTER` environment variable if set, otherwise falls back
/// to `tcp/127.0.0.1:7447`.
pub fn create_context() -> Result<ZContext> {
    let endpoint =
        std::env::var("ZENOH_ROUTER").unwrap_or_else(|_| "tcp/127.0.0.1:7447".to_string());
    ZContextBuilder::default()
        .with_connect_endpoints([endpoint])
        .build()
}
