use ros_z::{Builder, Result, action::ZAction, context::ZContextBuilder};
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
    let node = ctx.create_node("test_action_node").build()?;

    let client = node
        .create_action_client::<TestAction>("test_action")
        .build()?;

    let server = node
        .create_action_server::<TestAction>("test_action")
        .build()?;

    // Wait for discovery (similar to C++ test)
    tokio::time::sleep(std::time::Duration::from_millis(100)).await;

    Ok((node, client, server))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_simple_goal_execution() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Set up server handler
        let _server_handle = server.with_handler(|executing| async move {
            executing.succeed(TestResult { value: 42 }).unwrap();
        });

        // Give the server handler time to start
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal from client
        let mut goal_handle = client.send_goal(TestGoal { order: 10 }).await?;
        let result = goal_handle.result().await?;

        assert_eq!(result.value, 42);
        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_goal_rejection() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Set up server handler that rejects goals with order < 0
        let _server_handle = server.clone();
        tokio::spawn(async move {
            loop {
                if let Ok(requested) = _server_handle.recv_goal().await {
                    if requested.goal.order < 0 {
                        let _ = requested.reject();
                    } else {
                        let accepted = requested.accept();
                        let executing = accepted.execute();
                        executing.succeed(TestResult { value: 42 }).unwrap();
                    }
                }
            }
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal with negative order (should be rejected)
        let result = client.send_goal(TestGoal { order: -1 }).await;
        assert!(result.is_err(), "Goal should have been rejected");

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_feedback_streaming() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Set up server handler that publishes feedback
        let _server_handle = server.with_handler(|executing| async move {
            for i in 0..5 {
                executing
                    .publish_feedback(TestFeedback { progress: i * 20 })
                    .unwrap();
                tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            }
            executing.succeed(TestResult { value: 100 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal and collect feedback
        let mut goal_handle = client.send_goal(TestGoal { order: 10 }).await?;

        let mut feedback_values = Vec::new();
        if let Some(mut feedback_stream) = goal_handle.feedback_stream() {
            tokio::spawn(async move {
                while let Some(feedback) = feedback_stream.recv().await {
                    feedback_values.push(feedback.progress);
                }
                feedback_values
            });
        }

        let result = goal_handle.result().await?;
        assert_eq!(result.value, 100);

        // Give feedback collection time to complete
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_goal_cancellation() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Set up server handler that checks for cancellation
        let _server_handle = server.with_handler(|executing| async move {
            for i in 0..10 {
                if executing.is_cancel_requested() {
                    executing.canceled(TestResult { value: i }).unwrap();
                    return;
                }
                tokio::time::sleep(std::time::Duration::from_millis(50)).await;
            }
            executing.succeed(TestResult { value: 100 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal
        let goal_handle = client.send_goal(TestGoal { order: 10 }).await?;
        let goal_id = goal_handle.id();

        // Wait a bit then cancel
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        let cancel_response = client.cancel_goal(goal_id).await?;

        // Verify cancellation was accepted
        assert!(cancel_response.goals_canceling.len() > 0);

        Ok(())
    }
}
