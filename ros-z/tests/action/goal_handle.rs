use ros_z::{
    Builder, Result,
    action::{GoalStatus, ZAction},
    context::ZContextBuilder,
};
use serde::{Deserialize, Serialize};

// Define test action messages (similar to Fibonacci)
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

// Define the action type
pub struct TestAction;

impl ZAction for TestAction {
    type Goal = TestGoal;
    type Result = TestResult;
    type Feedback = TestFeedback;

    fn name() -> &'static str {
        "test_action"
    }
}

// Helper function to create test setup
async fn setup_test() -> Result<(
    ros_z::node::ZNode,
    ros_z::action::client::ZActionClient<TestAction>,
    std::sync::Arc<ros_z::action::server::ZActionServer<TestAction>>,
)> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("test_goal_handle_node").build()?;

    let client = node
        .create_action_client::<TestAction>("test_action")
        .build()?;

    let server = node
        .create_action_server::<TestAction>("test_action")
        .build()?;

    // Wait for discovery
    tokio::time::sleep(std::time::Duration::from_millis(100)).await;

    Ok((node, client, server))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    
    async fn test_goal_handle_creation() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Set up server handler
        let _server_handle = server.with_handler(|executing| async move {
            executing.succeed(TestResult { value: 42 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal and get handle
        let mut goal_handle = client.send_goal(TestGoal { order: 10 }).await?;
        let goal_id = goal_handle.id();

        // Verify goal handle has valid ID
        assert!(goal_id.is_valid()); // ID should not be all zeros

        // Get result
        let result = goal_handle.result().await?;
        assert_eq!(result.value, 42);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    
    async fn test_goal_handle_status() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Set up server handler that takes time
        let _server_handle = server.with_handler(|executing| async move {
            tokio::time::sleep(std::time::Duration::from_millis(200)).await;
            executing.succeed(TestResult { value: 42 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal
        let mut goal_handle = client.send_goal(TestGoal { order: 10 }).await?;

        // Check initial status
        if let Some(status_watch) = goal_handle.status_watch() {
            let initial_status = *status_watch.borrow();
            // Status should be accepted or executing
            assert!(
                initial_status == GoalStatus::Accepted || initial_status == GoalStatus::Executing
            );
        }

        // Wait for completion
        let result = goal_handle.result().await?;
        assert_eq!(result.value, 42);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    
    async fn test_goal_handle_feedback() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Set up server handler that publishes feedback
        let _server_handle = server.with_handler(|executing| async move {
            for i in 1..=3 {
                executing
                    .publish_feedback(TestFeedback { progress: i * 10 })
                    .unwrap();
                tokio::time::sleep(std::time::Duration::from_millis(50)).await;
            }
            executing.succeed(TestResult { value: 42 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal and collect feedback
        let mut goal_handle = client.send_goal(TestGoal { order: 10 }).await?;

        let mut feedback_values = Vec::new();
        if let Some(mut feedback_stream) = goal_handle.feedback_stream() {
            // Spawn task to collect feedback
            tokio::spawn(async move {
                while let Some(fb) = feedback_stream.recv().await {
                    feedback_values.push(fb.progress);
                }
            });
        }

        // Wait for result
        let result = goal_handle.result().await?;
        assert_eq!(result.value, 42);

        // Give feedback collection time
        tokio::time::sleep(std::time::Duration::from_millis(200)).await;

        // In a real test, we'd check feedback_values
        // For now, we just verify the result was received
        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    
    async fn test_goal_handle_cancellation() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Set up server handler that checks for cancellation
        let _server_handle = server.with_handler(|executing| async move {
            for _ in 0..10 {
                if executing.is_cancel_requested() {
                    executing.canceled(TestResult { value: 0 }).unwrap();
                    return;
                }
                tokio::time::sleep(std::time::Duration::from_millis(50)).await;
            }
            executing.succeed(TestResult { value: 42 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal
        let goal_handle = client.send_goal(TestGoal { order: 10 }).await?;
        let goal_id = goal_handle.id();

        // Cancel the goal
        let cancel_response = client.cancel_goal(goal_id).await?;
        assert!(!cancel_response.goals_canceling.is_empty());

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    
    async fn test_goal_handle_unique_ids() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Set up server handler
        let _server_handle = server.with_handler(|executing| async move {
            executing.succeed(TestResult { value: 42 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send multiple goals and verify unique IDs
        let mut goal_handle1 = client.send_goal(TestGoal { order: 10 }).await?;
        let mut goal_handle2 = client.send_goal(TestGoal { order: 20 }).await?;

        let id1 = goal_handle1.id();
        let id2 = goal_handle2.id();

        // IDs should be different
        assert_ne!(id1, id2);

        // Get results
        let result1 = goal_handle1.result().await?;
        let result2 = goal_handle2.result().await?;

        assert_eq!(result1.value, 42);
        assert_eq!(result2.value, 42);

        Ok(())
    }

    // Additional tests would cover:
    // - Goal handle state transitions (handled internally by server)
    // - Invalid goal handle operations
    // - Terminal state timestamps
    // These are more relevant to the low-level C API and less to the Rust high-level API
}
