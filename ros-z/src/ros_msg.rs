use serde::{Deserialize, Serialize};
use crate::entity::{TypeInfo, TypeHash};

/// Trait for ROS messages that provides compile-time message metadata
/// This trait is independent of serialization format
pub trait MessageTypeInfo {
    /// Returns the ROS message type name (e.g., "geometry_msgs::msg::dds_::Vector3_")
    fn type_name() -> &'static str;

    /// Returns the type hash (RIHS01 for ROS2, MD5 for ROS1)
    fn type_hash() -> TypeHash;

    /// Returns complete TypeInfo combining name and hash
    fn type_info() -> TypeInfo {
        TypeInfo::new(Self::type_name(), Self::type_hash())
    }

    /// Returns the package name (extracted from type name)
    fn package_name() -> &'static str {
        Self::type_name().split("::").next().unwrap_or("unknown")
    }

    /// Returns whether this message has a fixed size (for optimization)
    fn is_fixed_size() -> bool {
        false
    }
}

/// Backward compatibility alias for existing code
pub trait WithTypeInfo: MessageTypeInfo {}

#[derive(Debug, Serialize, Deserialize, Default, Clone, Copy)]
pub struct Vector3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl MessageTypeInfo for Vector3D {
    fn type_name() -> &'static str {
        "geometry_msgs::msg::dds_::Vector3_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::from_rihs_string("RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d").unwrap()
    }

    fn is_fixed_size() -> bool {
        true
    }
}

impl WithTypeInfo for Vector3D {}

#[derive(Debug, Serialize, Deserialize, Default, Clone, Copy)]
pub struct Twist {
    pub linear: Vector3D,
    pub angular: Vector3D,
}

impl MessageTypeInfo for Twist {
    fn type_name() -> &'static str {
        "geometry_msgs::msg::dds_::Twist_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::from_rihs_string("RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a").unwrap()
    }

    fn is_fixed_size() -> bool {
        true
    }
}

impl WithTypeInfo for Twist {}

#[derive(Debug, Serialize, Deserialize, Default, Clone, Copy)]
pub struct Bool {
    pub data: bool,
}

impl MessageTypeInfo for Bool {
    fn type_name() -> &'static str {
        "std_msgs::msg::dds_::Bool_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::from_rihs_string("RIHS01_feb91e995ff9ebd09c0cb3d2aed18b11077585839fb5db80193b62d74528f6c9").unwrap()
    }

    fn is_fixed_size() -> bool {
        true
    }
}

impl WithTypeInfo for Bool {}

#[derive(Debug, Serialize, Deserialize, Default, Clone, Copy)]
pub struct Int16 {
    pub data: i16,
}

impl MessageTypeInfo for Int16 {
    fn type_name() -> &'static str {
        "std_msgs::msg::dds_::Int16_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::from_rihs_string("RIHS01_1dcc3464e47c288a55f943a389d337cdb06804de3f5cd7a266b0de718eee17e5").unwrap()
    }

    fn is_fixed_size() -> bool {
        true
    }
}

impl WithTypeInfo for Int16 {}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct RosString {
    pub data: std::string::String,
}

impl MessageTypeInfo for RosString {
    fn type_name() -> &'static str {
        "std_msgs::msg::dds_::String_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::from_rihs_string("RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18").unwrap()
    }
}

impl WithTypeInfo for RosString {}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct BatteryState {
    pub header: Header,
    pub voltage: f32,
    pub temperature: f32,
    pub current: f32,
    pub charge: f32,
    pub capacity: f32,
    pub design_capacity: f32,
    pub percentage: f32,
    pub power_supply_status: u8,
    pub power_supply_health: u8,
    pub power_supply_technology: u8,
    pub present: bool,
    pub cell_voltage: Vec<f32>,
    pub cell_temperature: Vec<f32>,
    pub location: String,
    pub serial_number: String,
}

impl BatteryState {
    // Power supply status constants
    pub const POWER_SUPPLY_STATUS_UNKNOWN: u8 = 0;
    pub const POWER_SUPPLY_STATUS_CHARGING: u8 = 1;
    pub const POWER_SUPPLY_STATUS_DISCHARGING: u8 = 2;
    pub const POWER_SUPPLY_STATUS_NOT_CHARGING: u8 = 3;
    pub const POWER_SUPPLY_STATUS_FULL: u8 = 4;

    // Power supply health constants
    pub const POWER_SUPPLY_HEALTH_UNKNOWN: u8 = 0;
    pub const POWER_SUPPLY_HEALTH_GOOD: u8 = 1;
    pub const POWER_SUPPLY_HEALTH_OVERHEAT: u8 = 2;
    pub const POWER_SUPPLY_HEALTH_DEAD: u8 = 3;
    pub const POWER_SUPPLY_HEALTH_OVERVOLTAGE: u8 = 4;
    pub const POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: u8 = 5;
    pub const POWER_SUPPLY_HEALTH_COLD: u8 = 6;
    pub const POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: u8 = 7;
    pub const POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: u8 = 8;

    // Power supply technology constants
    pub const POWER_SUPPLY_TECHNOLOGY_UNKNOWN: u8 = 0;
    pub const POWER_SUPPLY_TECHNOLOGY_NIMH: u8 = 1;
    pub const POWER_SUPPLY_TECHNOLOGY_LION: u8 = 2;
    pub const POWER_SUPPLY_TECHNOLOGY_LIPO: u8 = 3;
    pub const POWER_SUPPLY_TECHNOLOGY_LIFE: u8 = 4;
    pub const POWER_SUPPLY_TECHNOLOGY_NICD: u8 = 5;
    pub const POWER_SUPPLY_TECHNOLOGY_LIMN: u8 = 6;
}

impl Default for BatteryState {
    fn default() -> Self {
        Self {
            header: Header::default(),
            voltage: 0.0,
            temperature: f32::NAN,
            current: f32::NAN,
            charge: f32::NAN,
            capacity: f32::NAN,
            design_capacity: f32::NAN,
            percentage: f32::NAN,
            power_supply_status: Self::POWER_SUPPLY_STATUS_UNKNOWN,
            power_supply_health: Self::POWER_SUPPLY_HEALTH_UNKNOWN,
            power_supply_technology: Self::POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
            present: false,
            cell_voltage: Vec::new(),
            cell_temperature: Vec::new(),
            location: String::new(),
            serial_number: String::new(),
        }
    }
}

impl MessageTypeInfo for BatteryState {
    fn type_name() -> &'static str {
        "sensor_msgs::msg::dds_::BatteryState_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::from_rihs_string("RIHS01_4bee5dfce981c98faa6828b868307a0a73f992ed0789f374ee96c8f840e69741").unwrap()
    }
}

impl WithTypeInfo for BatteryState {}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct LaserScan {
    pub header: Header,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub time_increment: f32,
    pub scan_time: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub ranges: Vec<f32>,
    pub intensities: Vec<f32>,
}

impl MessageTypeInfo for LaserScan {
    fn type_name() -> &'static str {
        "sensor_msgs::msg::dds_::LaserScan_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::from_rihs_string("RIHS01_64c191398013af96509d518dac71d5164f9382553fce5c1f8cca5be7924bd828").unwrap()
    }
}

impl WithTypeInfo for LaserScan {}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct ByteMultiArray {
    pub layout: MultiArrayLayout,
    pub data: Vec<u8>,
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct MultiArrayLayout {
    pub dim: Vec<MultiArrayDimension>,
    pub data_offset: u32,
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct MultiArrayDimension {
    pub label: String,
    pub size: u32,
    pub stride: u32,
}

pub mod srv {

    use crate::msg::{ZMessage, ZService};

    #[allow(non_snake_case)]
    pub mod AddTwoInts {
        use serde::{Deserialize, Serialize};

        pub type Service = (Request, Response);

        #[derive(Debug, Serialize, Deserialize, Default, Clone)]
        pub struct Request {
            pub a: i64,
            pub b: i64,
        }

        #[derive(Debug, Serialize, Deserialize, Default, Clone)]
        pub struct Response {
            pub sum: i64,
        }
    }

    pub enum ZSrv<L, R> {
        L(L),
        R(R),
    }
    impl<RQ: ZMessage, RP: ZMessage> ZService for ZSrv<RQ, RP> {
        type Request = RQ;
        type Response = RP;
    }
    impl<RQ: ZMessage, RP: ZMessage> ZService for (RQ, RP) {
        type Request = RQ;
        type Response = RP;
    }
}

#[cfg(test)]
mod tests {
    use cdr::{CdrLe, Infinite};

    use super::*;

    #[test]
    fn test_bytes() {
        let msg = ByteMultiArray::default();
        cdr::serialize::<_, _, CdrLe>(&msg, Infinite).unwrap();
    }
}
