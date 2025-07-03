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

const QOS_DELIMITER: &'static str = ",";

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
        // Only reliability, durability, and history are supported.
        let reliability = match self.reliability {
            QosReliability::Reliable => 1,
            QosReliability::BestEffort => 2,
        };
        let durability = match self.durability {
            QosDurability::TransientLocal => 1,
            QosDurability::Volatile => 2,
        };
        let history = match self.history {
            QosHistory::KeepLast(depth) => format!("1,{depth}"),
            QosHistory::KeepAll => "2,".into(),
        };
        // All other fields are left blank.
        let deadline = ",";
        let lifespan = ",";
        let liveliness = ",,";

        [
            reliability.to_string(),
            durability.to_string(),
            history,
            deadline.into(),
            lifespan.into(),
            liveliness.into(),
        ]
        .join(QOS_DELIMITER)
        .into()
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
            ..Default::default()
        })
    }
}
