use ros_z::{
    Builder, Result,
    context::{ZContext, ZContextBuilder},
};

pub fn init() {
    zenoh::init_log_from_env_or("error");
}

/// Build a context, optionally connecting to a router.
///
/// If the `ZENOH_ROUTER` environment variable is set (e.g.
/// `ZENOH_ROUTER=tcp/localhost:7447`), the context connects to that endpoint.
/// Otherwise Zenoh uses its default peer-discovery mode — no router required.
pub fn create_context() -> Result<ZContext> {
    if let Ok(endpoint) = std::env::var("ZENOH_ROUTER") {
        ZContextBuilder::default()
            .with_connect_endpoints([endpoint])
            .build()
    } else {
        ZContextBuilder::default().build()
    }
}
