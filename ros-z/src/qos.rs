use std::fmt;

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

#[derive(Debug, Hash, PartialEq, Eq, Clone, Copy)]
pub enum QosHistory {
    KeepLast(usize),
    KeepAll,
}

impl Default for QosHistory {
    fn default() -> Self {
        Self::KeepLast(10)
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

/// Represents a duration in seconds and nanoseconds
#[derive(Debug, Hash, PartialEq, Eq, Clone, Copy)]
pub struct Duration {
    pub sec: u64,
    pub nsec: u64,
}

impl Duration {
    pub const INFINITE: Duration = Duration {
        sec: 9223372036,
        nsec: 854775807,
    };
}

impl Default for Duration {
    fn default() -> Self {
        Self::INFINITE
    }
}

impl fmt::Display for Duration {
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
    pub deadline: Duration,
    pub lifespan: Duration,
    pub liveliness: QosLiveliness,
    pub liveliness_lease_duration: Duration,
}

impl fmt::Display for QosProfile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "QoS({}, {}, {}",
            self.reliability, self.durability, self.history
        )?;
        if self.deadline != Duration::INFINITE {
            write!(f, ", deadline={}", self.deadline)?;
        }
        if self.lifespan != Duration::INFINITE {
            write!(f, ", lifespan={}", self.lifespan)?;
        }
        if self.liveliness != QosLiveliness::Automatic {
            write!(f, ", liveliness={}", self.liveliness)?;
        }
        if self.liveliness_lease_duration != Duration::INFINITE {
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
}

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
                    format!("1,{}", depth)
                } else {
                    // Default history kind - only include depth
                    format!(",{}", depth)
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
                kind,
                self.liveliness_lease_duration.sec,
                self.liveliness_lease_duration.nsec
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
                            if let Ok(depth) = d.parse() {
                                QosHistory::KeepLast(depth)
                            } else {
                                return Err(QosDecodeError::InvalidHistory);
                            }
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
            Some(x) if x.is_empty() || x == "," => Duration::default(),
            Some(x) => {
                let mut iter = x.split(",");
                let sec = iter.next().unwrap_or("").parse().unwrap_or(Duration::INFINITE.sec);
                let nsec = iter.next().unwrap_or("").parse().unwrap_or(Duration::INFINITE.nsec);
                Duration { sec, nsec }
            }
            None => Duration::default(),
        };

        // Lifespan - format: <sec>,<nsec>
        let lifespan = match fields.next() {
            Some(x) if x.is_empty() || x == "," => Duration::default(),
            Some(x) => {
                let mut iter = x.split(",");
                let sec = iter.next().unwrap_or("").parse().unwrap_or(Duration::INFINITE.sec);
                let nsec = iter.next().unwrap_or("").parse().unwrap_or(Duration::INFINITE.nsec);
                Duration { sec, nsec }
            }
            None => Duration::default(),
        };

        // Liveliness - format: <kind>,<lease_sec>,<lease_nsec>
        let (liveliness, liveliness_lease_duration) = match fields.next() {
            Some(x) if x.is_empty() || x == ",," => (QosLiveliness::default(), Duration::default()),
            Some(x) => {
                let mut iter = x.split(",");
                let kind = match iter.next().unwrap_or("") {
                    "" | "0" | "1" => QosLiveliness::Automatic,
                    "2" => QosLiveliness::ManualByNode,
                    "3" => QosLiveliness::ManualByTopic,
                    _ => QosLiveliness::default(),
                };
                let sec = iter.next().unwrap_or("").parse().unwrap_or(Duration::INFINITE.sec);
                let nsec = iter.next().unwrap_or("").parse().unwrap_or(Duration::INFINITE.nsec);
                (kind, Duration { sec, nsec })
            }
            None => (QosLiveliness::default(), Duration::default()),
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
