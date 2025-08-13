use std::str::FromStr;

use crate::ros::*;
use ros_z::qos::{QosProfile, QosReliability, QosDurability, QosHistory};

const QOS_COMPONENT_DELIMITER: &str = ",";
const QOS_DELIMITER: &str = ":";

impl ToString for rmw_qos_profile_t {
    fn to_string(&self) -> String {
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
        [
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
        .join(QOS_DELIMITER)
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
            if let Ok(x) = components
                .next()
                .ok_or("History:depth is missing")?
                .parse()
            {
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
            if let Ok(x) = components.next().ok_or("Liveliness:liveliness is missing")?.parse() {
                qos.liveliness = rmw_qos_liveliness_policy_t::from_repr(x).ok_or("Invalid value")?;
            }
            if let Ok(x) = components.next().ok_or("Liveliness:lease_duration:sec is missing")?.parse() {
                qos.liveliness_lease_duration.sec = x;
            }
            if let Ok(x) = components.next().ok_or("Liveliness:lease_duration:nsec is missing")?.parse() {
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

        let infinite_time = rmw_time_s {
            sec: 9223372036,
            nsec: 854775807,
        };

        rmw_qos_profile_s {
            history,
            depth,
            reliability,
            durability,
            deadline: infinite_time,
            lifespan: infinite_time,
            liveliness: rmw_qos_liveliness_policy_e::AUTOMATIC,
            liveliness_lease_duration: infinite_time,
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

        let x= rmw_qos_profile_t::default();
        let y= x.to_string();
        let z = rmw_qos_profile_t::from_str(&y).unwrap();
        assert_eq!(x, z);
    }
}
