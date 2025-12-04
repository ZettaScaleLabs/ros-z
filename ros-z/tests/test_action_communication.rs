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

    // ============================================================================
    // Low-level communication tests (ported from C++ test_action_communication.cpp)
    // ============================================================================

    /// Test basic goal request/response communication
    /// Ported from test_valid_goal_comm in C++
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_valid_goal_comm() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Spawn server task to handle goal request manually
        tokio::spawn({
            let server = server.clone();
            async move {
                // Wait for goal request
                if let Ok(requested) = server.recv_goal().await {
                    // Check the goal was received correctly
                    assert_eq!(requested.goal.order, 10);

                    // Accept the goal
                    let _accepted = requested.accept();
                }
            }
        });

        // Give server time to start listening
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal request from client
        let mut goal_handle = client.send_goal(TestGoal { order: 10 }).await?;

        // Verify goal was accepted
        if let Some(status) = goal_handle.status_watch() {
            let current_status = status.borrow().clone();
            assert!(
                current_status == GoalStatus::Accepted || current_status == GoalStatus::Executing
            );
        }

        // Give server time to process
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        Ok(())
    }

    /// Test basic result request/response communication
    /// Ported from test_valid_result_comm in C++
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_valid_result_comm() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Set up server handler that completes the goal
        let _server_handle = server.with_handler(|executing| async move {
            executing.succeed(TestResult { value: 42 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal and wait for result
        let mut goal_handle = client.send_goal(TestGoal { order: 10 }).await?;
        let result = goal_handle.result().await?;

        // Verify result was received correctly
        assert_eq!(result.value, 42);

        Ok(())
    }

    /// Test basic cancel request/response communication
    /// Ported from test_valid_cancel_comm in C++
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_valid_cancel_comm() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Set up server handler that runs for a while
        let _server_handle = server.with_handler(|executing| async move {
            for _ in 0..100 {
                if executing.is_cancel_requested() {
                    executing.canceled(TestResult { value: 0 }).unwrap();
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

        // Send cancel request
        let cancel_response = client.cancel_goal(goal_id).await?;

        // Verify cancel response contains the goal
        assert!(!cancel_response.goals_canceling.is_empty());
        assert_eq!(cancel_response.goals_canceling[0].goal_id, goal_id);

        Ok(())
    }

    /// Test basic feedback publishing/subscription
    /// Ported from test_valid_feedback_comm in C++
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_valid_feedback_comm() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Set up server handler that publishes feedback
        let _server_handle = server.with_handler(|executing| async move {
            for i in 0..3 {
                executing
                    .publish_feedback(TestFeedback { progress: i })
                    .unwrap();
                tokio::time::sleep(std::time::Duration::from_millis(50)).await;
            }
            executing.succeed(TestResult { value: 100 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal and collect feedback
        let mut goal_handle = client.send_goal(TestGoal { order: 10 }).await?;

        let mut feedback_values = Vec::new();
        if let Some(mut feedback_stream) = goal_handle.feedback_stream() {
            let feedback_task = tokio::spawn(async move {
                let mut values = Vec::new();
                while let Some(feedback) = feedback_stream.recv().await {
                    values.push(feedback.progress);
                }
                values
            });

            // Wait for goal to complete
            let _result = goal_handle.result().await?;

            // Give feedback collection time to finish
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;

            // Get collected feedback values
            feedback_values = feedback_task.await?;
        }

        // Verify we received feedback
        assert!(!feedback_values.is_empty());
        assert!(feedback_values.contains(&0));

        Ok(())
    }

    /// Test basic status publishing/subscription
    /// Ported from test_valid_status_comm in C++
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_valid_status_comm() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Set up server handler
        let _server_handle = server.with_handler(|executing| async move {
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            executing.succeed(TestResult { value: 42 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal
        let mut goal_handle = client.send_goal(TestGoal { order: 10 }).await?;

        // Watch status changes
        let status_task = if let Some(mut status_rx) = goal_handle.status_watch() {
            Some(tokio::spawn(async move {
                let mut statuses = Vec::new();
                loop {
                    status_rx.changed().await.ok();
                    let status = status_rx.borrow().clone();
                    statuses.push(status);
                    if status == GoalStatus::Succeeded {
                        break;
                    }
                }
                statuses
            }))
        } else {
            None
        };

        // Wait for result
        let _result = goal_handle.result().await?;

        // Get collected statuses
        if let Some(task) = status_task {
            let statuses = task.await?;

            // Verify we saw status transitions
            assert!(!statuses.is_empty());
            assert!(statuses.contains(&GoalStatus::Succeeded));
        }

        Ok(())
    }

    /// Test handling multiple concurrent goals
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_multiple_goals() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Set up server handler
        let _server_handle = server.with_handler(|executing| async move {
            let order = executing.goal.order;
            tokio::time::sleep(std::time::Duration::from_millis(50)).await;
            executing.succeed(TestResult { value: order * 2 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send multiple goals
        let mut goal1 = client.send_goal(TestGoal { order: 10 }).await?;
        let mut goal2 = client.send_goal(TestGoal { order: 20 }).await?;
        let mut goal3 = client.send_goal(TestGoal { order: 30 }).await?;

        // Get results
        let result1 = goal1.result().await?;
        let result2 = goal2.result().await?;
        let result3 = goal3.result().await?;

        // Verify results
        assert_eq!(result1.value, 20);
        assert_eq!(result2.value, 40);
        assert_eq!(result3.value, 60);

        Ok(())
    }
}
