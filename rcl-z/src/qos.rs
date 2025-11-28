use std::str::FromStr;

use crate::ros::*;
use ros_z::qos::{Duration, QosDurability, QosHistory, QosLiveliness, QosProfile, QosReliability};

/// Convert RMW QoS profile to ros-z QoS profile
pub fn rmw_qos_to_ros_z_qos(rmw_qos: &rmw_qos_profile_t) -> QosProfile {
    let reliability = match rmw_qos.reliability {
        rmw_qos_reliability_policy_e::RELIABLE => QosReliability::Reliable,
        rmw_qos_reliability_policy_e::BEST_EFFORT => QosReliability::BestEffort,
        _ => QosReliability::default(),
    };

    let durability = match rmw_qos.durability {
        rmw_qos_durability_policy_e::TRANSIENT_LOCAL => QosDurability::TransientLocal,
        rmw_qos_durability_policy_e::VOLATILE => QosDurability::Volatile,
        _ => QosDurability::default(),
    };

    let history = match rmw_qos.history {
        rmw_qos_history_policy_e::KEEP_LAST => QosHistory::KeepLast(rmw_qos.depth),
        rmw_qos_history_policy_e::KEEP_ALL => QosHistory::KeepAll,
        _ => QosHistory::default(),
    };

    let deadline = Duration {
        sec: rmw_qos.deadline.sec,
        nsec: rmw_qos.deadline.nsec,
    };

    let lifespan = Duration {
        sec: rmw_qos.lifespan.sec,
        nsec: rmw_qos.lifespan.nsec,
    };

    let liveliness = match rmw_qos.liveliness {
        rmw_qos_liveliness_policy_e::AUTOMATIC => QosLiveliness::Automatic,
        rmw_qos_liveliness_policy_e::MANUAL_BY_NODE => QosLiveliness::ManualByNode,
        rmw_qos_liveliness_policy_e::MANUAL_BY_TOPIC => QosLiveliness::ManualByTopic,
        _ => QosLiveliness::default(),
    };

    let liveliness_lease_duration = Duration {
        sec: rmw_qos.liveliness_lease_duration.sec,
        nsec: rmw_qos.liveliness_lease_duration.nsec,
    };

    QosProfile {
        reliability,
        durability,
        history,
        deadline,
        lifespan,
        liveliness,
        liveliness_lease_duration,
    }
}

const QOS_COMPONENT_DELIMITER: &str = ",";
const QOS_DELIMITER: &str = ":";

impl std::fmt::Display for rmw_qos_profile_t {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let default_qos = rmw_qos_profile_t::default();
        macro_rules! format_field {
            ($field:ident) => {
                if self.$field != default_qos.$field {
                    format!("{}", self.$field as u64)
                } else {
                    "".to_string()
                }
            };

            ($field:ident.$sub_field:ident) => {
                if self.$field.$sub_field != default_qos.$field.$sub_field {
                    format!("{}", self.$field.$sub_field as u64)
                } else {
                    "".to_string()
                }
            };
        }

        macro_rules! join_components {
            ( $( $s:expr ),* $(,)? ) => {
                {
                    [$($s),*].join(QOS_COMPONENT_DELIMITER)
                }
            };
        }
        let result = [
            format_field!(reliability),
            format_field!(durability),
            join_components!(format_field!(history), format_field!(depth)),
            join_components!(format_field!(deadline.sec), format_field!(deadline.nsec)),
            join_components!(format_field!(lifespan.sec), format_field!(lifespan.nsec)),
            join_components!(
                format_field!(liveliness),
                format_field!(liveliness_lease_duration.sec),
                format_field!(liveliness_lease_duration.nsec),
            ),
        ]
        .join(QOS_DELIMITER);
        write!(f, "{}", result)
    }
}

impl FromStr for rmw_qos_profile_t {
    type Err = &'static str;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut qos = rmw_qos_profile_t::default();
        let mut fields = s.split(QOS_DELIMITER);

        // Reliability
        if let Ok(x) = fields.next().ok_or("Reliability is missing")?.parse() {
            qos.reliability = rmw_qos_reliability_policy_t::from_repr(x).ok_or("Invalid value")?;
        }

        // Durability
        if let Ok(x) = fields.next().ok_or("Durability is missing")?.parse() {
            qos.durability = rmw_qos_durability_policy_t::from_repr(x).ok_or("Invalid value")?;
        }

        // History
        {
            let mut components = fields
                .next()
                .ok_or("History is missing")?
                .split(QOS_COMPONENT_DELIMITER);
            if let Ok(x) = components
                .next()
                .ok_or("History:history is missing")?
                .parse()
            {
                qos.history = rmw_qos_history_policy_t::from_repr(x).ok_or("Invalid value")?;
            }
            if let Ok(x) = components.next().ok_or("History:depth is missing")?.parse() {
                qos.depth = x;
            }
        }

        // Deadline
        {
            let mut components = fields
                .next()
                .ok_or("Deadline is missing")?
                .split(QOS_COMPONENT_DELIMITER);
            if let Ok(x) = components.next().ok_or("Deadline:sec is missing")?.parse() {
                qos.deadline.sec = x;
            }
            if let Ok(x) = components.next().ok_or("Deadline:nsec is missing")?.parse() {
                qos.deadline.nsec = x;
            }
        }

        // Lifespan
        {
            let mut components = fields
                .next()
                .ok_or("Lifespan is missing")?
                .split(QOS_COMPONENT_DELIMITER);
            if let Ok(x) = components.next().ok_or("Lifespan:sec is missing")?.parse() {
                qos.lifespan.sec = x;
            }
            if let Ok(x) = components.next().ok_or("Lifespan:nsec is missing")?.parse() {
                qos.lifespan.nsec = x;
            }
        }

        // Liveliness
        {
            let mut components = fields
                .next()
                .ok_or("Liveliness is missing")?
                .split(QOS_COMPONENT_DELIMITER);
            if let Ok(x) = components
                .next()
                .ok_or("Liveliness:liveliness is missing")?
                .parse()
            {
                qos.liveliness =
                    rmw_qos_liveliness_policy_t::from_repr(x).ok_or("Invalid value")?;
            }
            if let Ok(x) = components
                .next()
                .ok_or("Liveliness:lease_duration:sec is missing")?
                .parse()
            {
                qos.liveliness_lease_duration.sec = x;
            }
            if let Ok(x) = components
                .next()
                .ok_or("Liveliness:lease_duration:nsec is missing")?
                .parse()
            {
                qos.liveliness_lease_duration.nsec = x;
            }
        }

        Ok(qos)
    }
}

impl From<&QosProfile> for rmw_qos_profile_t {
    fn from(qos: &QosProfile) -> Self {
        let reliability = match qos.reliability {
            QosReliability::Reliable => rmw_qos_reliability_policy_e::RELIABLE,
            QosReliability::BestEffort => rmw_qos_reliability_policy_e::BEST_EFFORT,
        };

        let durability = match qos.durability {
            QosDurability::TransientLocal => rmw_qos_durability_policy_e::TRANSIENT_LOCAL,
            QosDurability::Volatile => rmw_qos_durability_policy_e::VOLATILE,
        };

        let (history, depth) = match qos.history {
            QosHistory::KeepLast(d) => (rmw_qos_history_policy_e::KEEP_LAST, d),
            QosHistory::KeepAll => (rmw_qos_history_policy_e::KEEP_ALL, 0),
        };

        let deadline = rmw_time_s {
            sec: qos.deadline.sec,
            nsec: qos.deadline.nsec,
        };

        let lifespan = rmw_time_s {
            sec: qos.lifespan.sec,
            nsec: qos.lifespan.nsec,
        };

        let liveliness = match qos.liveliness {
            QosLiveliness::Automatic => rmw_qos_liveliness_policy_e::AUTOMATIC,
            QosLiveliness::ManualByNode => rmw_qos_liveliness_policy_e::MANUAL_BY_NODE,
            QosLiveliness::ManualByTopic => rmw_qos_liveliness_policy_e::MANUAL_BY_TOPIC,
        };

        let liveliness_lease_duration = rmw_time_s {
            sec: qos.liveliness_lease_duration.sec,
            nsec: qos.liveliness_lease_duration.nsec,
        };

        rmw_qos_profile_s {
            history,
            depth,
            reliability,
            durability,
            deadline,
            lifespan,
            liveliness,
            liveliness_lease_duration,
            avoid_ros_namespace_conventions: false,
        }
    }
}

#[cfg(test)]
mod tests {
    // Import parent module functions
    use super::*;

    #[test]
    fn test_qos_to_string() {
        assert_eq!(rmw_qos_profile_t::default().to_string(), "::,:,:,:,,");

        let x = rmw_qos_profile_t::default();
        let y = x.to_string();
        let z = rmw_qos_profile_t::from_str(&y).unwrap();
        assert_eq!(x, z);
    }
}
