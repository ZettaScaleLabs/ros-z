use crate::msg::ZMessage;
use serde::{Deserialize, Serialize};
use std::time::SystemTime;

pub mod client;
pub mod messages;
pub mod server;

#[cfg(test)]
mod tests;

/// Core trait for ROS 2 actions
pub trait ZAction: Send + Sync + 'static {
    type Goal: ZMessage + Clone + serde::Serialize + for<'de> serde::Deserialize<'de>;
    type Result: ZMessage + Clone + serde::Serialize + for<'de> serde::Deserialize<'de>;
    type Feedback: ZMessage + Clone + serde::Serialize + for<'de> serde::Deserialize<'de>;

    fn name() -> &'static str;
}

/// Unique identifier for action goals.
///
/// A `GoalId` is a UUID that uniquely identifies an action goal.
/// It is generated when a goal is sent and used to track the goal's
/// lifecycle, feedback, and results.
///
/// # Examples
///
/// ```
/// # use ros_z::action::GoalId;
/// let goal_id = GoalId::new();
/// assert!(goal_id.is_valid());
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct GoalId([u8; 16]);

impl GoalId {
    /// Creates a new random GoalId.
    ///
    /// Generates a UUID v4 and uses it as the goal identifier.
    ///
    /// # Returns
    ///
    /// A new `GoalId` with a randomly generated UUID.
    pub fn new() -> Self {
        // Generate UUID v4
        let mut uuid = [0u8; 16];
        uuid.copy_from_slice(&uuid::Uuid::new_v4().as_bytes()[..]);
        Self(uuid)
    }

    /// Creates a GoalId from raw bytes.
    ///
    /// # Arguments
    ///
    /// * `bytes` - A 16-byte array representing a UUID.
    ///
    /// # Returns
    ///
    /// A `GoalId` with the specified bytes.
    pub const fn from_bytes(bytes: [u8; 16]) -> Self {
        Self(bytes)
    }

    /// Checks if this GoalId is valid (not all zeros).
    ///
    /// # Returns
    ///
    /// `true` if the GoalId contains at least one non-zero byte, `false` otherwise.
    pub fn is_valid(&self) -> bool {
        self.0.iter().any(|&x| x != 0)
    }

    /// Returns the raw bytes of this GoalId.
    ///
    /// # Returns
    ///
    /// A reference to the 16-byte array containing the UUID.
    pub fn as_bytes(&self) -> &[u8; 16] {
        &self.0
    }
}

impl Default for GoalId {
    fn default() -> Self {
        Self::new()
    }
}

/// Status of an action goal.
///
/// The `GoalStatus` enum represents the current state of an action goal
/// in its lifecycle, from acceptance to completion or cancellation.
///
/// # Examples
///
/// ```
/// # use ros_z::action::GoalStatus;
/// let status = GoalStatus::Executing;
/// assert!(status.is_active());
/// assert!(!status.is_terminal());
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(i8)]
pub enum GoalStatus {
    /// Unknown status (initial state).
    Unknown = 0,
    /// Goal has been accepted by the server.
    Accepted = 1,
    /// Goal is currently being executed.
    Executing = 2,
    /// Goal is being canceled.
    Canceling = 3,
    /// Goal completed successfully.
    Succeeded = 4,
    /// Goal was canceled.
    Canceled = 5,
    /// Goal failed/aborted.
    Aborted = 6,
}

impl GoalStatus {
    /// Checks if the goal is in an active state.
    ///
    /// Active states are `Accepted`, `Executing`, and `Canceling`.
    ///
    /// # Returns
    ///
    /// `true` if the goal is active, `false` otherwise.
    pub fn is_active(&self) -> bool {
        matches!(self, Self::Accepted | Self::Executing | Self::Canceling)
    }

    /// Checks if the goal is in a terminal state.
    ///
    /// Terminal states are `Succeeded`, `Canceled`, and `Aborted`.
    ///
    /// # Returns
    ///
    /// `true` if the goal is terminal, `false` otherwise.
    pub fn is_terminal(&self) -> bool {
        matches!(self, Self::Succeeded | Self::Canceled | Self::Aborted)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
/// Information about an action goal including its ID and timestamp.
///
/// `GoalInfo` combines a `GoalId` with a timestamp to provide complete
/// information about when a goal was created or last updated.
///
/// # Examples
///
/// ```
/// # use ros_z::action::{GoalId, GoalInfo};
/// let goal_id = GoalId::new();
/// let goal_info = GoalInfo::new(goal_id);
/// ```
pub struct GoalInfo {
    /// The unique identifier of the goal.
    pub goal_id: GoalId,
    /// Timestamp in nanoseconds since Unix epoch.
    pub stamp: i64,
}

impl GoalInfo {
    /// Creates a new GoalInfo with the current timestamp.
    ///
    /// # Arguments
    ///
    /// * `goal_id` - The ID of the goal.
    ///
    /// # Returns
    ///
    /// A `GoalInfo` with the specified goal ID and current timestamp.
    pub fn new(goal_id: GoalId) -> Self {
        let stamp = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as i64;
        Self { goal_id, stamp }
    }
}

/// Events that can trigger goal state transitions.
///
/// `GoalEvent` represents the different events that can cause an action goal
/// to transition from one state to another in the ROS 2 action state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GoalEvent {
    /// Start executing an accepted goal.
    Execute,
    /// Request to cancel the goal.
    CancelGoal,
    /// Goal execution completed successfully.
    Succeed,
    /// Goal execution failed.
    Abort,
    /// Goal was successfully canceled.
    Canceled,
}

/// Transitions a goal status based on an event.
///
/// This function implements the ROS 2 action state machine transitions.
/// It takes the current goal status and an event, and returns the new status
/// according to the ROS 2 specification.
///
/// # Arguments
///
/// * `current` - The current status of the goal.
/// * `event` - The event that triggered the transition.
///
/// # Returns
///
/// The new goal status after applying the transition. Returns `GoalStatus::Unknown`
/// for invalid transitions.
///
/// # Examples
///
/// ```
/// # use ros_z::action::{GoalStatus, GoalEvent, transition_goal_state};
/// let new_status = transition_goal_state(GoalStatus::Accepted, GoalEvent::Execute);
/// assert_eq!(new_status, GoalStatus::Executing);
/// ```
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