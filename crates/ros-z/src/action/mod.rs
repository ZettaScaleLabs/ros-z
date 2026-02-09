use crate::msg::ZMessage;
use serde::{Deserialize, Serialize};
use std::time::SystemTime;

pub mod client;
pub mod driver;
pub mod macros;
pub mod messages;
pub mod server;
pub mod state;

// Re-export type-state markers for documentation and advanced usage
pub use server::{Accepted, Executing, Requested};

/// Core trait for ROS 2 actions
pub trait ZAction: Send + Sync + 'static {
    type Goal: ZMessage + Clone + Send + Sync + serde::Serialize + for<'de> serde::Deserialize<'de>;
    type Result: ZMessage
        + Clone
        + Send
        + Sync
        + serde::Serialize
        + for<'de> serde::Deserialize<'de>;
    type Feedback: ZMessage + Clone + serde::Serialize + for<'de> serde::Deserialize<'de>;

    fn name() -> &'static str;

    /// Returns type info for the SendGoal service.
    /// Default implementation returns zero hash for ros-z to ros-z communication only.
    /// Override this to provide proper type hashes for ROS 2 interop.
    fn send_goal_type_info() -> crate::entity::TypeInfo {
        crate::entity::TypeInfo::new(
            &format!("{}/_action/SendGoal", Self::name()),
            crate::entity::TypeHash::zero(),
        )
    }

    /// Returns type info for the GetResult service.
    /// Default implementation returns zero hash for ros-z to ros-z communication only.
    /// Override this to provide proper type hashes for ROS 2 interop.
    fn get_result_type_info() -> crate::entity::TypeInfo {
        crate::entity::TypeInfo::new(
            &format!("{}/_action/GetResult", Self::name()),
            crate::entity::TypeHash::zero(),
        )
    }

    /// Returns type info for the CancelGoal service.
    /// Default implementation returns zero hash for ros-z to ros-z communication only.
    /// Override this to provide proper type hashes for ROS 2 interop.
    fn cancel_goal_type_info() -> crate::entity::TypeInfo {
        crate::entity::TypeInfo::new(
            "action_msgs/srv/CancelGoal",
            crate::entity::TypeHash::zero(),
        )
    }

    /// Returns type info for the Feedback topic.
    /// Default implementation returns zero hash for ros-z to ros-z communication only.
    /// Override this to provide proper type hashes for ROS 2 interop.
    fn feedback_type_info() -> crate::entity::TypeInfo {
        crate::entity::TypeInfo::new(
            &format!("{}/_FeedbackMessage", Self::name()),
            crate::entity::TypeHash::zero(),
        )
    }

    /// Returns type info for the Status topic.
    /// Default implementation returns zero hash for ros-z to ros-z communication only.
    /// Override this to provide proper type hashes for ROS 2 interop.
    fn status_type_info() -> crate::entity::TypeInfo {
        crate::entity::TypeInfo::new(
            "action_msgs/msg/GoalStatusArray",
            crate::entity::TypeHash::zero(),
        )
    }
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
#[serde(try_from = "i8", into = "i8")]
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

// Conversion from i8 for serde deserialization (ROS2 uses i8 for status)
impl TryFrom<i8> for GoalStatus {
    type Error = String;

    fn try_from(value: i8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(GoalStatus::Unknown),
            1 => Ok(GoalStatus::Accepted),
            2 => Ok(GoalStatus::Executing),
            3 => Ok(GoalStatus::Canceling),
            4 => Ok(GoalStatus::Succeeded),
            5 => Ok(GoalStatus::Canceled),
            6 => Ok(GoalStatus::Aborted),
            _ => Err(format!("Invalid GoalStatus value: {}", value)),
        }
    }
}

// Conversion to i8 for serde serialization (ROS2 uses i8 for status)
impl From<GoalStatus> for i8 {
    fn from(status: GoalStatus) -> i8 {
        status as i8
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
/// Information about an action goal including its ID and timestamp.
///
/// `GoalInfo` combines a `GoalId` with a timestamp to provide complete
/// information about when a goal was created or last updated.
///
/// This matches the ROS2 `action_msgs/msg/GoalInfo` structure which uses
/// `builtin_interfaces/Time` for the stamp field.
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
    /// Timestamp using ROS2 Time structure (sec: i32, nanosec: u32)
    pub stamp: Time,
}

/// ROS2 Time structure from builtin_interfaces/msg/Time
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub struct Time {
    /// Seconds component of the timestamp
    pub sec: i32,
    /// Nanoseconds component of the timestamp
    pub nanosec: u32,
}

impl Time {
    /// Creates a Time from the current system time
    pub fn now() -> Self {
        let duration = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap();
        Self {
            sec: duration.as_secs() as i32,
            nanosec: duration.subsec_nanos(),
        }
    }

    /// Creates a zero timestamp
    pub fn zero() -> Self {
        Self { sec: 0, nanosec: 0 }
    }
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
        Self {
            goal_id,
            stamp: Time::now(),
        }
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
