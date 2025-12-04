use std::time::{Duration, SystemTime};

use ros_z::{
    Builder, Result,
    action::{GoalId, GoalInfo, GoalStatus, ZAction},
    context::ZContextBuilder,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestGoal {
    pub order: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestResult {
    pub value: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestFeedback {
    pub progress: i32,
}

pub struct TestAction;

impl ZAction for TestAction {
    type Goal = TestGoal;
    type Result = TestResult;
    type Feedback = TestFeedback;

    fn name() -> &'static str {
        "test_action"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_ros2_action_naming_conventions() -> Result<()> {
        // Test that our action naming follows ROS 2 conventions
        let ctx = ZContextBuilder::default().build()?;
        let node = ctx.create_node("test_compliance_node").build()?;

        let action_name = "fibonacci";

        // Test client naming
        let client_builder = node.create_action_client::<TestAction>(action_name);

        // Verify the naming conventions match ROS 2
        // Note: We can't directly access the internal names, but we can verify
        // the client builds successfully with the expected action name
        let client = client_builder.build()?;

        // Test server naming
        let server_builder = node.create_action_server::<TestAction>(action_name);
        let server = server_builder.build()?;

        // Basic validation that construction works
        assert!(client.send_goal(TestGoal { order: 1 }).await.is_ok());

        Ok(())
    }

    #[test]
    fn test_goal_id_format_compliance() -> Result<()> {
        // Test that GoalId format is compatible with ROS 2 expectations
        let id1 = GoalId::new();
        let id2 = GoalId::new();

        // IDs should be unique
        assert_ne!(id1, id2);

        // IDs should be valid (not all zeros)
        assert!(id1.is_valid());
        assert!(id2.is_valid());

        // Test conversion to bytes
        let bytes = id1.as_bytes();
        assert_eq!(bytes.len(), 16);

        // Test from_bytes
        let id3 = GoalId::from_bytes(*bytes);
        assert_eq!(id1, id3);

        Ok(())
    }

    #[test]
    fn test_goal_status_enum_values() -> Result<()> {
        // Test that GoalStatus values match ROS 2 specification
        // ROS 2 action goal status values:
        // UNKNOWN = 0, ACCEPTED = 1, EXECUTING = 2, CANCELING = 3,
        // SUCCEEDED = 4, CANCELED = 5, ABORTED = 6

        assert_eq!(GoalStatus::Unknown as i8, 0);
        assert_eq!(GoalStatus::Accepted as i8, 1);
        assert_eq!(GoalStatus::Executing as i8, 2);
        assert_eq!(GoalStatus::Canceling as i8, 3);
        assert_eq!(GoalStatus::Succeeded as i8, 4);
        assert_eq!(GoalStatus::Canceled as i8, 5);
        assert_eq!(GoalStatus::Aborted as i8, 6);

        // Test is_active and is_terminal methods
        assert!(!GoalStatus::Unknown.is_active());
        assert!(!GoalStatus::Unknown.is_terminal());

        assert!(GoalStatus::Accepted.is_active());
        assert!(!GoalStatus::Accepted.is_terminal());

        assert!(GoalStatus::Executing.is_active());
        assert!(!GoalStatus::Executing.is_terminal());

        assert!(GoalStatus::Canceling.is_active());
        assert!(!GoalStatus::Canceling.is_terminal());

        assert!(!GoalStatus::Succeeded.is_active());
        assert!(GoalStatus::Succeeded.is_terminal());

        assert!(!GoalStatus::Canceled.is_active());
        assert!(GoalStatus::Canceled.is_terminal());

        assert!(!GoalStatus::Aborted.is_active());
        assert!(GoalStatus::Aborted.is_terminal());

        Ok(())
    }

    #[test]
    fn test_goal_info_structure() -> Result<()> {
        // Test GoalInfo structure matches ROS 2 expectations
        let goal_id = GoalId::new();
        let goal_info = GoalInfo::new(goal_id);

        // Should have valid goal_id
        assert!(goal_info.goal_id.is_valid());

        // Should have reasonable timestamp (not zero and not in far future)
        let now = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as i64;

        assert!(goal_info.stamp > 0);
        assert!(goal_info.stamp <= now + 1000000); // Allow 1ms tolerance

        Ok(())
    }

    #[test]
    fn test_message_serialization_compatibility() -> Result<()> {
        // Test that our messages can be serialized/deserialized correctly
        // This validates that the serde implementations work properly

        // Test GoalId
        let goal_id = GoalId::new();
        let serialized = serde_json::to_string(&goal_id)?;
        let deserialized: GoalId = serde_json::from_str(&serialized)?;
        assert_eq!(goal_id, deserialized);

        // Test GoalInfo
        let goal_info = GoalInfo::new(goal_id);
        let serialized = serde_json::to_string(&goal_info)?;
        let deserialized: GoalInfo = serde_json::from_str(&serialized)?;
        assert_eq!(goal_info.goal_id, deserialized.goal_id);
        assert_eq!(goal_info.stamp, deserialized.stamp);

        // Test GoalStatus
        let status = GoalStatus::Executing;
        let serialized = serde_json::to_string(&status)?;
        let deserialized: GoalStatus = serde_json::from_str(&serialized)?;
        assert_eq!(status, deserialized);

        Ok(())
    }

    #[test]
    fn test_action_message_structure() -> Result<()> {
        // Test that action messages have the expected structure
        let goal = TestGoal { order: 42 };
        let result = TestResult { value: 84 };
        let feedback = TestFeedback { progress: 50 };

        // Test serialization round-trip
        let goal_json = serde_json::to_string(&goal)?;
        let goal_back: TestGoal = serde_json::from_str(&goal_json)?;
        assert_eq!(goal.order, goal_back.order);

        let result_json = serde_json::to_string(&result)?;
        let result_back: TestResult = serde_json::from_str(&result_json)?;
        assert_eq!(result.value, result_back.value);

        let feedback_json = serde_json::to_string(&feedback)?;
        let feedback_back: TestFeedback = serde_json::from_str(&feedback_json)?;
        assert_eq!(feedback.progress, feedback_back.progress);

        Ok(())
    }

    #[test]
    fn test_action_state_machine_compliance() -> Result<()> {
        // Test that our goal state transitions follow ROS 2 specification
        use ros_z::action::transition_goal_state;

        // Valid transitions from ACCEPTED
        assert_eq!(
            transition_goal_state(GoalStatus::Accepted, ros_z::action::GoalEvent::Execute),
            GoalStatus::Executing
        );
        assert_eq!(
            transition_goal_state(GoalStatus::Accepted, ros_z::action::GoalEvent::CancelGoal),
            GoalStatus::Canceling
        );

        // Valid transitions from EXECUTING
        assert_eq!(
            transition_goal_state(GoalStatus::Executing, ros_z::action::GoalEvent::CancelGoal),
            GoalStatus::Canceling
        );
        assert_eq!(
            transition_goal_state(GoalStatus::Executing, ros_z::action::GoalEvent::Succeed),
            GoalStatus::Succeeded
        );
        assert_eq!(
            transition_goal_state(GoalStatus::Executing, ros_z::action::GoalEvent::Abort),
            GoalStatus::Aborted
        );

        // Valid transitions from CANCELING
        assert_eq!(
            transition_goal_state(GoalStatus::Canceling, ros_z::action::GoalEvent::Canceled),
            GoalStatus::Canceled
        );
        assert_eq!(
            transition_goal_state(GoalStatus::Canceling, ros_z::action::GoalEvent::Succeed),
            GoalStatus::Succeeded
        );
        assert_eq!(
            transition_goal_state(GoalStatus::Canceling, ros_z::action::GoalEvent::Abort),
            GoalStatus::Aborted
        );

        // Invalid transitions should return Unknown
        assert_eq!(
            transition_goal_state(GoalStatus::Succeeded, ros_z::action::GoalEvent::Execute),
            GoalStatus::Unknown
        );
        assert_eq!(
            transition_goal_state(GoalStatus::Unknown, ros_z::action::GoalEvent::Succeed),
            GoalStatus::Unknown
        );

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_action_protocol_compliance() -> Result<()> {
        // Test that our action protocol follows ROS 2 action specification
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_protocol_client").build()?;
        let server_node = ctx.create_node("test_protocol_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_protocol_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_protocol_action")
            .build()?;

        // Test basic action protocol: send goal -> accept -> execute -> succeed
        let goal = TestGoal { order: 10 };
        let goal_handle = client.send_goal(goal).await?;
        assert!(goal_handle.id().is_valid());

        // Server should receive and accept the goal
        let server_clone = server.clone();
        let processing_task = tokio::spawn(async move {
            if let Ok(requested) = server_clone.recv_goal().await {
                let accepted = requested.accept();
                let goal_info = accepted.goal.clone();
                let executing = accepted.execute();

                // Simulate processing
                tokio::time::sleep(Duration::from_millis(10)).await;

                let result = TestResult {
                    value: goal_info.order * 2,
                };
                let _ = executing.succeed(result);
            }
        });

        // Client should get result
        let mut handle = goal_handle;
        let result = handle.result().await?;
        assert_eq!(result.value, 20); // 10 * 2

        processing_task.await?;

        Ok(())
    }

    #[test]
    fn test_cancel_message_format() -> Result<()> {
        // Test that cancel messages follow ROS 2 format
        let goal_id = GoalId::new();
        let goal_info = GoalInfo::new(goal_id);

        let cancel_request = ros_z::action::messages::CancelGoalRequest {
            goal_info: goal_info.clone(),
        };
        let cancel_response = ros_z::action::messages::CancelGoalResponse {
            return_code: 0,
            goals_canceling: vec![goal_info],
        };

        // Test serialization
        let req_json = serde_json::to_string(&cancel_request)?;
        let resp_json = serde_json::to_string(&cancel_response)?;

        // Should be valid JSON
        assert!(req_json.contains("goal_info"));
        assert!(resp_json.contains("return_code"));
        assert!(resp_json.contains("goals_canceling"));

        Ok(())
    }
}
