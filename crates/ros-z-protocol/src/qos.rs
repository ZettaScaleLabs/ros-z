//! QoS profile encoding/decoding for liveliness tokens.

#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use alloc::string::String;
use core::fmt::Display;

/// QoS profile for ROS 2 entities.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct QosProfile {
    pub reliability: QosReliability,
    pub durability: QosDurability,
    pub history: QosHistory,
}

impl QosProfile {
    /// Encode QoS to string for liveliness token.
    pub fn encode(&self) -> String {
        use alloc::format;
        format!(
            ":{reliability}:,{depth}:{durability}:,:,,",
            reliability = self.reliability as u8,
            depth = self.history.depth(),
            durability = self.durability as u8,
        )
    }

    /// Decode QoS from liveliness token string.
    ///
    /// ROS 2's rmw_zenoh uses conditional encoding - empty fields mean default values.
    pub fn decode(s: &str) -> Result<Self, QosDecodeError> {
        let parts: alloc::vec::Vec<&str> = s.split(&[':', ',']).collect();
        if parts.len() < 4 {
            return Err(QosDecodeError::InvalidFormat);
        }

        // Parse reliability - empty means default (BestEffort)
        let reliability = if parts[1].is_empty() {
            QosReliability::default()
        } else {
            let val = parts[1]
                .parse::<u8>()
                .map_err(|_| QosDecodeError::InvalidReliability)?;
            QosReliability::from_u8(val).ok_or(QosDecodeError::InvalidReliability)?
        };

        // Parse depth - required field
        let depth = parts[3]
            .parse::<usize>()
            .map_err(|_| QosDecodeError::InvalidHistory)?;

        // Parse durability - empty means default (Volatile)
        let durability = if parts[4].is_empty() {
            QosDurability::default()
        } else {
            let val = parts[4]
                .parse::<u8>()
                .map_err(|_| QosDecodeError::InvalidDurability)?;
            QosDurability::from_u8(val).ok_or(QosDecodeError::InvalidDurability)?
        };

        Ok(QosProfile {
            reliability,
            durability,
            history: QosHistory::KeepLast(depth),
        })
    }
}

/// QoS reliability policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(u8)]
pub enum QosReliability {
    #[default]
    BestEffort = 0,
    Reliable = 1,
}

impl QosReliability {
    fn from_u8(val: u8) -> Option<Self> {
        match val {
            0 => Some(QosReliability::BestEffort),
            1 => Some(QosReliability::Reliable),
            _ => None,
        }
    }
}

/// QoS durability policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(u8)]
pub enum QosDurability {
    #[default]
    Volatile = 0,
    TransientLocal = 1,
}

impl QosDurability {
    fn from_u8(val: u8) -> Option<Self> {
        match val {
            0 => Some(QosDurability::Volatile),
            1 => Some(QosDurability::TransientLocal),
            _ => None,
        }
    }
}

/// QoS history policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum QosHistory {
    KeepLast(usize),
    KeepAll,
}

impl QosHistory {
    pub fn from_depth(depth: usize) -> Self {
        QosHistory::KeepLast(depth)
    }

    pub fn depth(&self) -> usize {
        match self {
            QosHistory::KeepLast(d) => *d,
            QosHistory::KeepAll => 0,
        }
    }
}

impl Default for QosHistory {
    fn default() -> Self {
        QosHistory::KeepLast(10)
    }
}

/// QoS decoding errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QosDecodeError {
    InvalidFormat,
    InvalidReliability,
    InvalidDurability,
    InvalidHistory,
}

impl Display for QosDecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for QosDecodeError {}
