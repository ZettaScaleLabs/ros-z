use crate::msg::ZMessage;
use serde::{Deserialize, Serialize};
use std::time::SystemTime;

pub mod messages;

#[cfg(test)]
mod tests;

/// Core trait for ROS 2 actions
pub trait ZAction: Send + Sync + 'static {
    type Goal: ZMessage + serde::Serialize + for<'de> serde::Deserialize<'de>;
    type Result: ZMessage + serde::Serialize + for<'de> serde::Deserialize<'de>;
    type Feedback: ZMessage + serde::Serialize + for<'de> serde::Deserialize<'de>;

    fn name() -> &'static str;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct GoalId([u8; 16]);

impl GoalId {
    pub fn new() -> Self {
        // Generate UUID v4
        let mut uuid = [0u8; 16];
        uuid.copy_from_slice(&uuid::Uuid::new_v4().as_bytes()[..]);
        Self(uuid)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(i8)]
pub enum GoalStatus {
    Unknown = 0,
    Accepted = 1,
    Executing = 2,
    Canceling = 3,
    Succeeded = 4,
    Canceled = 5,
    Aborted = 6,
}

impl GoalStatus {
    pub fn is_active(&self) -> bool {
        matches!(self, Self::Accepted | Self::Executing | Self::Canceling)
    }

    pub fn is_terminal(&self) -> bool {
        matches!(self, Self::Succeeded | Self::Canceled | Self::Aborted)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoalInfo {
    pub goal_id: GoalId,
    pub stamp: i64, // nanoseconds since epoch
}

impl GoalInfo {
    pub fn new(goal_id: GoalId) -> Self {
        let stamp = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as i64;
        Self { goal_id, stamp }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GoalEvent {
    Execute,
    CancelGoal,
    Succeed,
    Abort,
    Canceled,
}

// State machine transition logic (ROS 2 spec compliant)
pub fn transition_goal_state(current: GoalStatus, event: GoalEvent) -> GoalStatus {
    match (current, event) {
        // From ACCEPTED
        (GoalStatus::Accepted, GoalEvent::Execute) => GoalStatus::Executing,
        (GoalStatus::Accepted, GoalEvent::CancelGoal) => GoalStatus::Canceling,

        // From EXECUTING
        (GoalStatus::Executing, GoalEvent::CancelGoal) => GoalStatus::Canceling,
        (GoalStatus::Executing, GoalEvent::Succeed) => GoalStatus::Succeeded,
        (GoalStatus::Executing, GoalEvent::Abort) => GoalStatus::Aborted,

        // From CANCELING
        (GoalStatus::Canceling, GoalEvent::Canceled) => GoalStatus::Canceled,
        (GoalStatus::Canceling, GoalEvent::Succeed) => GoalStatus::Succeeded,
        (GoalStatus::Canceling, GoalEvent::Abort) => GoalStatus::Aborted,

        // Invalid transitions
        _ => GoalStatus::Unknown,
    }
}