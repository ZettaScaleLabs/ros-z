#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals, unused)]

#[allow(clippy::upper_case_acronyms)]
#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq, strum::FromRepr)]
pub enum rmw_qos_reliability_policy_e {
    SYSTEM_DEFAULT = 0,
    RELIABLE = 1,
    BEST_EFFORT = 2,
    UNKNOWN = 3,
    BEST_AVAILABLE = 4,
}
pub use self::rmw_qos_reliability_policy_e as rmw_qos_reliability_policy_t;

#[allow(clippy::upper_case_acronyms)]
#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq, strum::FromRepr)]
pub enum rmw_qos_history_policy_e {
    SYSTEM_DEFAULT = 0,
    KEEP_LAST = 1,
    KEEP_ALL = 2,
    UNKNOWN = 3,
}
pub use self::rmw_qos_history_policy_e as rmw_qos_history_policy_t;

#[allow(clippy::upper_case_acronyms)]
#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq, strum::FromRepr)]
pub enum rmw_qos_durability_policy_e {
    SYSTEM_DEFAULT = 0,
    TRANSIENT_LOCAL = 1,
    VOLATILE = 2,
    UNKNOWN = 3,
    BEST_AVAILABLE = 4,
}
pub use self::rmw_qos_durability_policy_e as rmw_qos_durability_policy_t;

#[allow(clippy::upper_case_acronyms)]
#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq, strum::FromRepr)]
pub enum rmw_qos_liveliness_policy_e {
    SYSTEM_DEFAULT = 0,
    AUTOMATIC = 1,
    MANUAL_BY_NODE = 2,
    MANUAL_BY_TOPIC = 3,
    UNKNOWN = 4,
    BEST_AVAILABLE = 5,
}
pub use self::rmw_qos_liveliness_policy_e as rmw_qos_liveliness_policy_t;

#[repr(C)]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct rmw_qos_profile_s {
    pub history: rmw_qos_history_policy_e,
    pub depth: usize,
    pub reliability: rmw_qos_reliability_policy_e,
    pub durability: rmw_qos_durability_policy_e,
    pub deadline: rmw_time_s,
    pub lifespan: rmw_time_s,
    pub liveliness: rmw_qos_liveliness_policy_e,
    pub liveliness_lease_duration: rmw_time_s,
    pub avoid_ros_namespace_conventions: bool,
}

const RMW_TIME_INFINITE: rmw_time_s = rmw_time_s {
    sec: 9223372036,
    nsec: 854775807,
};

impl Default for rmw_qos_profile_s {
    fn default() -> Self {
        Self {
            history: rmw_qos_history_policy_t::KEEP_LAST,
            depth: 42,
            reliability: rmw_qos_reliability_policy_t::RELIABLE,
            durability: rmw_qos_durability_policy_t::VOLATILE,
            deadline: RMW_TIME_INFINITE,
            lifespan: RMW_TIME_INFINITE,
            liveliness: rmw_qos_liveliness_policy_t::AUTOMATIC,
            liveliness_lease_duration: RMW_TIME_INFINITE,
            avoid_ros_namespace_conventions: false,
        }
    }
}

include!(concat!(env!("CARGO_MANIFEST_DIR"), "/bindings.rs"));
