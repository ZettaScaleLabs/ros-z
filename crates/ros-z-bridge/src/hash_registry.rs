//! Static registry of ROS type_name → Jazzy TypeHash.
//!
//! Built at startup from the generated ros-z-msgs types.  All types compiled
//! into this binary with the `jazzy` feature are registered here.  At runtime
//! the bridge looks up a type name to obtain the hash that Jazzy nodes expect.
//!
//! Note: the generated Rust module structure is:
//!   ros_z_msgs::ros::std_msgs::String    (flat — no msg/ submodule in Rust)
//! But the ROS type name string is:
//!   "std_msgs::msg::dds_::String_"
//! The registry maps the ROS type name string to its TypeHash.

use std::collections::HashMap;
use std::sync::OnceLock;

use ros_z::MessageTypeInfo;
use ros_z_protocol::TypeHash;

// ---------------------------------------------------------------------------
// Registry
// ---------------------------------------------------------------------------

/// Lazily-initialised global registry.
static REGISTRY: OnceLock<HashMap<&'static str, TypeHash>> = OnceLock::new();

/// Return the global registry, initialising it on first call.
pub fn registry() -> &'static HashMap<&'static str, TypeHash> {
    REGISTRY.get_or_init(build_registry)
}

/// Look up the Jazzy `TypeHash` for a given ROS type name.
///
/// Returns `None` if the type is not registered (not compiled into this
/// binary, or the name does not match exactly).
pub fn lookup(type_name: &str) -> Option<&'static TypeHash> {
    registry().get(type_name)
}

// ---------------------------------------------------------------------------
// Registry builder
// ---------------------------------------------------------------------------

/// Helper macro: register a single `MessageTypeInfo` implementor.
macro_rules! reg {
    ($map:expr, $T:ty) => {
        $map.insert(<$T>::type_name(), <$T>::type_hash());
    };
}

fn build_registry() -> HashMap<&'static str, TypeHash> {
    // Generated types live under ros_z_msgs::ros::{package}::{TypeName}
    // (no `msg` submodule in the Rust code; `msg` only appears in the
    //  ROS type name string returned by type_name()).
    use ros_z_msgs::ros::std_msgs;

    let mut m: HashMap<&'static str, TypeHash> = HashMap::new();

    // std_msgs — always present when the std_msgs feature is enabled
    {
        reg!(m, std_msgs::Bool);
        reg!(m, std_msgs::Byte);
        reg!(m, std_msgs::Char);
        reg!(m, std_msgs::Empty);
        reg!(m, std_msgs::Float32);
        reg!(m, std_msgs::Float64);
        reg!(m, std_msgs::Int8);
        reg!(m, std_msgs::Int16);
        reg!(m, std_msgs::Int32);
        reg!(m, std_msgs::Int64);
        reg!(m, std_msgs::UInt8);
        reg!(m, std_msgs::UInt16);
        reg!(m, std_msgs::UInt32);
        reg!(m, std_msgs::UInt64);
        reg!(m, std_msgs::String);
        reg!(m, std_msgs::Header);
        reg!(m, std_msgs::ColorRGBA);
        reg!(m, std_msgs::MultiArrayDimension);
        reg!(m, std_msgs::MultiArrayLayout);
        reg!(m, std_msgs::ByteMultiArray);
        reg!(m, std_msgs::Float32MultiArray);
        reg!(m, std_msgs::Float64MultiArray);
        reg!(m, std_msgs::Int8MultiArray);
        reg!(m, std_msgs::Int16MultiArray);
        reg!(m, std_msgs::Int32MultiArray);
        reg!(m, std_msgs::Int64MultiArray);
        reg!(m, std_msgs::UInt8MultiArray);
        reg!(m, std_msgs::UInt16MultiArray);
        reg!(m, std_msgs::UInt32MultiArray);
        reg!(m, std_msgs::UInt64MultiArray);
    }

    // geometry_msgs
    {
        use ros_z_msgs::ros::geometry_msgs as gm;
        reg!(m, gm::Accel);
        reg!(m, gm::AccelStamped);
        reg!(m, gm::AccelWithCovariance);
        reg!(m, gm::AccelWithCovarianceStamped);
        reg!(m, gm::Inertia);
        reg!(m, gm::InertiaStamped);
        reg!(m, gm::Point);
        reg!(m, gm::Point32);
        reg!(m, gm::PointStamped);
        reg!(m, gm::Polygon);
        reg!(m, gm::PolygonStamped);
        reg!(m, gm::Pose);
        reg!(m, gm::Pose2D);
        reg!(m, gm::PoseArray);
        reg!(m, gm::PoseStamped);
        reg!(m, gm::PoseWithCovariance);
        reg!(m, gm::PoseWithCovarianceStamped);
        reg!(m, gm::Quaternion);
        reg!(m, gm::QuaternionStamped);
        reg!(m, gm::Transform);
        reg!(m, gm::TransformStamped);
        reg!(m, gm::Twist);
        reg!(m, gm::TwistStamped);
        reg!(m, gm::TwistWithCovariance);
        reg!(m, gm::TwistWithCovarianceStamped);
        reg!(m, gm::Vector3);
        reg!(m, gm::Vector3Stamped);
        reg!(m, gm::Wrench);
        reg!(m, gm::WrenchStamped);
    }

    // sensor_msgs
    {
        use ros_z_msgs::ros::sensor_msgs as sm;
        reg!(m, sm::BatteryState);
        reg!(m, sm::CameraInfo);
        reg!(m, sm::ChannelFloat32);
        reg!(m, sm::CompressedImage);
        reg!(m, sm::FluidPressure);
        reg!(m, sm::Illuminance);
        reg!(m, sm::Image);
        reg!(m, sm::Imu);
        reg!(m, sm::JointState);
        reg!(m, sm::Joy);
        reg!(m, sm::JoyFeedback);
        reg!(m, sm::JoyFeedbackArray);
        reg!(m, sm::LaserEcho);
        reg!(m, sm::LaserScan);
        reg!(m, sm::MagneticField);
        reg!(m, sm::MultiDOFJointState);
        reg!(m, sm::MultiEchoLaserScan);
        reg!(m, sm::NavSatFix);
        reg!(m, sm::NavSatStatus);
        reg!(m, sm::PointCloud);
        reg!(m, sm::PointCloud2);
        reg!(m, sm::PointField);
        reg!(m, sm::Range);
        reg!(m, sm::RegionOfInterest);
        reg!(m, sm::RelativeHumidity);
        reg!(m, sm::Temperature);
        reg!(m, sm::TimeReference);
    }

    // nav_msgs
    {
        use ros_z_msgs::ros::nav_msgs as nm;
        reg!(m, nm::GridCells);
        reg!(m, nm::MapMetaData);
        reg!(m, nm::OccupancyGrid);
        reg!(m, nm::Odometry);
        reg!(m, nm::Path);
    }

    m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_not_empty() {
        assert!(!registry().is_empty(), "registry should have entries");
    }

    #[cfg(feature = "jazzy")]
    #[test]
    fn std_msgs_string_registered() {
        // type_name() returns "std_msgs::msg::dds_::String_"
        let hash = lookup("std_msgs::msg::dds_::String_");
        assert!(hash.is_some(), "std_msgs/String should be in the registry");
        // Jazzy hashes are non-zero
        let h = hash.unwrap();
        assert!(
            h.value != [0u8; 32],
            "jazzy hash must not be all-zeros sentinel"
        );
    }

    #[cfg(feature = "jazzy")]
    #[test]
    fn geometry_msgs_twist_registered() {
        let hash = lookup("geometry_msgs::msg::dds_::Twist_");
        assert!(
            hash.is_some(),
            "geometry_msgs/Twist should be in the registry"
        );
    }

    #[test]
    fn unknown_type_returns_none() {
        assert!(lookup("nonexistent::msg::dds_::Bogus_").is_none());
    }
}
