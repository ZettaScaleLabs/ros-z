#[derive(Debug, Default, Hash, PartialEq, Eq, Clone, Copy)]
pub enum QosReliability {
    #[default]
    Reliable,
    BestEffort,
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

#[derive(Debug, Default, Hash, PartialEq, Eq, Clone, Copy)]
pub enum QosDurability {
    TransientLocal,
    #[default]
    Volatile,
}

#[derive(Debug, Default, Hash, PartialEq, Eq, Clone, Copy)]
pub struct QosProfile {
    pub reliability: QosReliability,
    pub durability: QosDurability,
    pub history: QosHistory,
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

        // Deadline, lifespan, liveliness (all empty for now)
        let deadline = ",";
        let lifespan = ",";
        let liveliness = ",,";

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
        Ok(Self {
            reliability,
            durability,
            history,
        })
    }
}
