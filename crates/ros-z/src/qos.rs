use std::fmt;
use std::num::NonZeroUsize;

#[derive(Debug, Default, Hash, PartialEq, Eq, Clone, Copy)]
pub enum QosReliability {
    #[default]
    Reliable,
    BestEffort,
}

impl fmt::Display for QosReliability {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Reliable => write!(f, "Reliable"),
            Self::BestEffort => write!(f, "Best Effort"),
        }
    }
}

/// Default depth for KEEP_LAST when SYSTEM_DEFAULT (depth=0) is used
/// This matches ROS 2 and rmw_zenoh_cpp behavior
pub const DEFAULT_HISTORY_DEPTH: usize = 10;

#[derive(Debug, Hash, PartialEq, Eq, Clone, Copy)]
pub enum QosHistory {
    KeepLast(NonZeroUsize),
    KeepAll,
}

impl Default for QosHistory {
    fn default() -> Self {
        Self::KeepLast(NonZeroUsize::new(DEFAULT_HISTORY_DEPTH).unwrap())
    }
}

impl QosHistory {
    /// Normalize depth by replacing 0 with the default depth
    /// Used when converting from RMW QoS (which allows depth=0 for SYSTEM_DEFAULT)
    pub fn from_depth(depth: usize) -> Self {
        let normalized_depth = if depth == 0 {
            DEFAULT_HISTORY_DEPTH
        } else {
            depth
        };
        Self::KeepLast(
            NonZeroUsize::new(normalized_depth)
                .unwrap_or_else(|| NonZeroUsize::new(DEFAULT_HISTORY_DEPTH).unwrap()),
        )
    }
}

impl fmt::Display for QosHistory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::KeepLast(depth) => write!(f, "Keep Last ({})", depth),
            Self::KeepAll => write!(f, "Keep All"),
        }
    }
}

#[derive(Debug, Default, Hash, PartialEq, Eq, Clone, Copy)]
pub enum QosDurability {
    TransientLocal,
    #[default]
    Volatile,
}

impl fmt::Display for QosDurability {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TransientLocal => write!(f, "Transient Local"),
            Self::Volatile => write!(f, "Volatile"),
        }
    }
}

#[derive(Debug, Default, Hash, PartialEq, Eq, Clone, Copy)]
pub enum QosLiveliness {
    #[default]
    Automatic,
    ManualByNode,
    ManualByTopic,
}

impl fmt::Display for QosLiveliness {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Automatic => write!(f, "Automatic"),
            Self::ManualByNode => write!(f, "Manual by Node"),
            Self::ManualByTopic => write!(f, "Manual by Topic"),
        }
    }
}

/// Represents a QoS duration in seconds and nanoseconds.
///
/// This is distinct from [`std::time::Duration`] and is used exclusively for
/// configuring QoS deadline, lifespan, and liveliness lease duration.
/// Use [`QosDuration::INFINITE`] (the default) to disable a QoS time constraint.
#[derive(Debug, Hash, PartialEq, Eq, Clone, Copy)]
pub struct QosDuration {
    pub sec: u64,
    pub nsec: u64,
}

impl QosDuration {
    pub const INFINITE: QosDuration = QosDuration {
        sec: 9223372036,
        nsec: 854775807,
    };
}

impl Default for QosDuration {
    fn default() -> Self {
        Self::INFINITE
    }
}

impl fmt::Display for QosDuration {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if *self == Self::INFINITE {
            write!(f, "Infinite")
        } else if self.nsec == 0 {
            write!(f, "{}s", self.sec)
        } else {
            write!(f, "{}s {}ns", self.sec, self.nsec)
        }
    }
}

#[derive(Debug, Default, Hash, PartialEq, Eq, Clone, Copy)]
pub struct QosProfile {
    pub reliability: QosReliability,
    pub durability: QosDurability,
    pub history: QosHistory,
    pub deadline: QosDuration,
    pub lifespan: QosDuration,
    pub liveliness: QosLiveliness,
    pub liveliness_lease_duration: QosDuration,
}

impl QosProfile {
    /// Convert to ros-z-protocol's QosProfile for key expression generation
    pub fn to_protocol_qos(&self) -> ros_z_protocol::qos::QosProfile {
        ros_z_protocol::qos::QosProfile {
            reliability: match self.reliability {
                QosReliability::Reliable => ros_z_protocol::qos::QosReliability::Reliable,
                QosReliability::BestEffort => ros_z_protocol::qos::QosReliability::BestEffort,
            },
            durability: match self.durability {
                QosDurability::TransientLocal => ros_z_protocol::qos::QosDurability::TransientLocal,
                QosDurability::Volatile => ros_z_protocol::qos::QosDurability::Volatile,
            },
            history: match self.history {
                QosHistory::KeepLast(depth) => {
                    ros_z_protocol::qos::QosHistory::KeepLast(depth.get())
                }
                QosHistory::KeepAll => ros_z_protocol::qos::QosHistory::KeepAll,
            },
        }
    }
}

impl fmt::Display for QosProfile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "QoS({}, {}, {}",
            self.reliability, self.durability, self.history
        )?;
        if self.deadline != QosDuration::INFINITE {
            write!(f, ", deadline={}", self.deadline)?;
        }
        if self.lifespan != QosDuration::INFINITE {
            write!(f, ", lifespan={}", self.lifespan)?;
        }
        if self.liveliness != QosLiveliness::Automatic {
            write!(f, ", liveliness={}", self.liveliness)?;
        }
        if self.liveliness_lease_duration != QosDuration::INFINITE {
            write!(f, ", lease={}", self.liveliness_lease_duration)?;
        }
        write!(f, ")")
    }
}

const QOS_DELIMITER: &str = ":";

#[derive(Debug)]
pub enum QosDecodeError {
    IncompleteQos,
    InvalidReliability,
    InvalidDurability,
    InvalidHistory,
    InvalidHistoryDepth,
}

impl std::fmt::Display for QosDecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::IncompleteQos => write!(f, "Incomplete QoS string"),
            Self::InvalidReliability => write!(f, "Invalid reliability value in QoS"),
            Self::InvalidDurability => write!(f, "Invalid durability value in QoS"),
            Self::InvalidHistory => write!(f, "Invalid history value in QoS"),
            Self::InvalidHistoryDepth => write!(f, "Invalid history depth value in QoS"),
        }
    }
}

impl std::error::Error for QosDecodeError {}

impl QosProfile {
    // This format comes from rmw_zenoh
    // <ReliabilityKind>:<DurabilityKind>:<HistoryKind>,<HistoryDepth>:<DeadlineSec, DeadlineNSec>:<LifespanSec, LifespanNSec>:<Liveliness, LivelinessSec, LivelinessNSec>"
    pub fn encode(&self) -> String {
        let default_qos = Self::default();

        // Reliability - empty if default
        let reliability = if self.reliability != default_qos.reliability {
            match self.reliability {
                QosReliability::Reliable => "1",
                QosReliability::BestEffort => "2",
            }
        } else {
            ""
        };

        // Durability - empty if default
        let durability = if self.durability != default_qos.durability {
            match self.durability {
                QosDurability::TransientLocal => "1",
                QosDurability::Volatile => "2",
            }
        } else {
            ""
        };

        // History format: <history_kind>,<depth>
        // Only include kind if it's non-default
        // Always include depth (even if default)
        let history = match self.history {
            QosHistory::KeepLast(depth) => {
                if self.history != default_qos.history {
                    // Non-default history kind - include both kind and depth
                    format!("1,{}", depth.get())
                } else {
                    // Default history kind - only include depth
                    format!(",{}", depth.get())
                }
            }
            QosHistory::KeepAll => "2,".to_string(),
        };

        // Deadline - empty if default (infinite)
        let deadline = if self.deadline != default_qos.deadline {
            format!("{},{}", self.deadline.sec, self.deadline.nsec)
        } else {
            ",".to_string()
        };

        // Lifespan - empty if default (infinite)
        let lifespan = if self.lifespan != default_qos.lifespan {
            format!("{},{}", self.lifespan.sec, self.lifespan.nsec)
        } else {
            ",".to_string()
        };

        // Liveliness - format: <liveliness_kind>,<lease_sec>,<lease_nsec>
        let liveliness = if self.liveliness != default_qos.liveliness
            || self.liveliness_lease_duration != default_qos.liveliness_lease_duration
        {
            let kind = match self.liveliness {
                QosLiveliness::Automatic => "1",
                QosLiveliness::ManualByNode => "2",
                QosLiveliness::ManualByTopic => "3",
            };
            format!(
                "{},{},{}",
                kind, self.liveliness_lease_duration.sec, self.liveliness_lease_duration.nsec
            )
        } else {
            ",,".to_string()
        };

        format!(
            "{}:{}:{}:{}:{}:{}",
            reliability, durability, history, deadline, lifespan, liveliness
        )
    }

    pub fn decode(encoded: impl AsRef<str>) -> Result<Self, QosDecodeError> {
        let mut fields = encoded.as_ref().split(QOS_DELIMITER);
        let reliability = match fields.next() {
            Some(x) => match x {
                "0" | "" => QosReliability::default(),
                "1" => QosReliability::Reliable,
                "2" => QosReliability::BestEffort,
                _ => return Err(QosDecodeError::InvalidReliability),
            },
            None => return Err(QosDecodeError::IncompleteQos),
        };
        let durability = match fields.next() {
            Some(x) => match x {
                "0" | "" => QosDurability::default(),
                "1" => QosDurability::TransientLocal,
                "2" => QosDurability::Volatile,
                _ => return Err(QosDecodeError::InvalidDurability),
            },
            None => return Err(QosDecodeError::IncompleteQos),
        };
        let history = match fields.next() {
            Some(x) => match x {
                "," | "" => QosHistory::default(),
                x => {
                    let mut iter = x.split(",");
                    let Some(kind) = iter.next() else {
                        return Err(QosDecodeError::IncompleteQos);
                    };
                    let Some(depth) = iter.next() else {
                        return Err(QosDecodeError::IncompleteQos);
                    };
                    match (kind, depth) {
                        ("", d) | ("0", d) | ("1", d) => {
                            let depth_usize: usize =
                                d.parse().map_err(|_| QosDecodeError::InvalidHistory)?;
                            let non_zero_depth = NonZeroUsize::new(depth_usize)
                                .ok_or(QosDecodeError::InvalidHistoryDepth)?;
                            QosHistory::KeepLast(non_zero_depth)
                        }
                        ("2", _) => QosHistory::KeepAll,
                        _ => return Err(QosDecodeError::InvalidHistory),
                    }
                }
            },
            None => return Err(QosDecodeError::IncompleteQos),
        };

        // Deadline - format: <sec>,<nsec>
        let deadline = match fields.next() {
            Some(x) if x.is_empty() || x == "," => QosDuration::default(),
            Some(x) => {
                let mut iter = x.split(",");
                let sec = iter
                    .next()
                    .unwrap_or("")
                    .parse()
                    .unwrap_or(QosDuration::INFINITE.sec);
                let nsec = iter
                    .next()
                    .unwrap_or("")
                    .parse()
                    .unwrap_or(QosDuration::INFINITE.nsec);
                QosDuration { sec, nsec }
            }
            None => QosDuration::default(),
        };

        // Lifespan - format: <sec>,<nsec>
        let lifespan = match fields.next() {
            Some(x) if x.is_empty() || x == "," => QosDuration::default(),
            Some(x) => {
                let mut iter = x.split(",");
                let sec = iter
                    .next()
                    .unwrap_or("")
                    .parse()
                    .unwrap_or(QosDuration::INFINITE.sec);
                let nsec = iter
                    .next()
                    .unwrap_or("")
                    .parse()
                    .unwrap_or(QosDuration::INFINITE.nsec);
                QosDuration { sec, nsec }
            }
            None => QosDuration::default(),
        };

        // Liveliness - format: <kind>,<lease_sec>,<lease_nsec>
        let (liveliness, liveliness_lease_duration) = match fields.next() {
            Some(x) if x.is_empty() || x == ",," => {
                (QosLiveliness::default(), QosDuration::default())
            }
            Some(x) => {
                let mut iter = x.split(",");
                let kind = match iter.next().unwrap_or("") {
                    "" | "0" | "1" => QosLiveliness::Automatic,
                    "2" => QosLiveliness::ManualByNode,
                    "3" => QosLiveliness::ManualByTopic,
                    _ => QosLiveliness::default(),
                };
                let sec = iter
                    .next()
                    .unwrap_or("")
                    .parse()
                    .unwrap_or(QosDuration::INFINITE.sec);
                let nsec = iter
                    .next()
                    .unwrap_or("")
                    .parse()
                    .unwrap_or(QosDuration::INFINITE.nsec);
                (kind, QosDuration { sec, nsec })
            }
            None => (QosLiveliness::default(), QosDuration::default()),
        };

        Ok(Self {
            reliability,
            durability,
            history,
            deadline,
            lifespan,
            liveliness,
            liveliness_lease_duration,
        })
    }
}
