//! Backend abstraction for Zenoh key expression generation.
//!
//! ROS-Z supports two backend formats for Zenoh communication:
//! - `rmw_zenoh`: Compatible with rmw_zenoh (default)
//! - `ros2dds`: Compatible with zenoh-plugin-ros2dds
//!
//! Use feature flags to select the backend at compile time:
//! - `--features rmw-zenoh` (default)
//! - `--features ros2dds`

#[cfg(feature = "rmw-zenoh")]
pub mod rmw_zenoh;

#[cfg(feature = "ros2dds")]
pub mod ros2dds;

use zenoh::{key_expr::KeyExpr, session::ZenohId, Result};

use crate::{
    entity::{EndpointEntity, Entity, LivelinessKE, NodeEntity, TopicKE},
    qos::QosProfile,
};

/// Backend-specific key expression generation trait.
///
/// This trait abstracts the differences between key expression formats
/// used by rmw_zenoh and zenoh-plugin-ros2dds.
pub trait KeyExprBackend {
    /// Escape character used to replace slashes in key expressions.
    const ESCAPE_CHAR: char;

    /// Admin space prefix for liveliness tokens.
    const ADMIN_SPACE: &'static str;

    /// Generate topic key expression for data publication/subscription.
    fn topic_key_expr(entity: &EndpointEntity) -> Result<TopicKE>;

    /// Generate liveliness token for endpoint entity discovery.
    fn liveliness_key_expr(entity: &EndpointEntity, zid: &ZenohId) -> Result<LivelinessKE>;

    /// Generate liveliness token for node entity discovery.
    fn node_liveliness_key_expr(entity: &NodeEntity) -> Result<LivelinessKE>;

    /// Parse liveliness token back to entity.
    fn parse_liveliness(ke: &KeyExpr) -> Result<Entity>;

    /// Mangle a name (replace slashes with escape char).
    fn mangle_name(name: &str) -> String {
        name.replace('/', &Self::ESCAPE_CHAR.to_string())
    }

    /// Demangle a name (restore slashes from escape char).
    fn demangle_name(name: &str) -> String {
        name.replace(Self::ESCAPE_CHAR, "/")
    }

    /// Encode QoS for liveliness token.
    fn encode_qos(qos: &QosProfile, keyless: bool) -> String;

    /// Decode QoS from liveliness token.
    fn decode_qos(s: &str) -> Result<(bool, QosProfile)>;
}

/// Enum for runtime backend selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BackendKind {
    /// rmw_zenoh compatible backend (default)
    #[default]
    RmwZenoh,
    /// zenoh-plugin-ros2dds compatible backend
    Ros2Dds,
}

// Default backend type alias based on feature flags
#[cfg(all(feature = "rmw-zenoh", not(feature = "ros2dds")))]
pub type DefaultBackend = rmw_zenoh::RmwZenohBackend;

#[cfg(all(feature = "ros2dds", not(feature = "rmw-zenoh")))]
pub type DefaultBackend = ros2dds::Ros2DdsBackend;

// When both features are enabled, prefer rmw-zenoh (more established, backwards compatible)
#[cfg(all(feature = "rmw-zenoh", feature = "ros2dds"))]
pub type DefaultBackend = rmw_zenoh::RmwZenohBackend;

// When no backend feature is enabled, default to rmw_zenoh
#[cfg(not(any(feature = "rmw-zenoh", feature = "ros2dds")))]
compile_error!("At least one backend feature must be enabled: 'rmw-zenoh' or 'ros2dds'");

// Re-export commonly used types
#[cfg(feature = "rmw-zenoh")]
pub use rmw_zenoh::RmwZenohBackend;

#[cfg(feature = "ros2dds")]
pub use ros2dds::Ros2DdsBackend;
