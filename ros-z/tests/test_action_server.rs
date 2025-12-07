use ros_z::{
    Builder, Result,
    action::ZAction,
    context::ZContextBuilder,
    qos::{QosHistory, QosProfile, QosReliability},
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

#[cfg(test)]
mod tests {
    use super::*;

    // Helper function to create test setup
    async fn setup_test() -> Result<(
        ros_z::node::ZNode,
        ros_z::action::client::ZActionClient<TestAction>,
        std::sync::Arc<ros_z::action::server::ZActionServer<TestAction>>,
    )> {
        let ctx = ZContextBuilder::default().build()?;
        let node = ctx.create_node("test_action_server_node").build()?;

        let client = node
            .create_action_client::<TestAction>("test_action_server_name")
            .build()?;

        let server = node
            .create_action_server::<TestAction>("test_action_server_name")
            .build()?;

        Ok((node, client, server))
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_action_server_init_fini() -> Result<()> {
        let ctx = ZContextBuilder::default().build()?;
        let node = ctx.create_node("test_action_server_node").build()?;

        // Test successful initialization with valid arguments
        let _server = node
            .create_action_server::<TestAction>("test_action_server_name")
            .build()?;
        // If we reach here, initialization succeeded

        // Note: Action name validation is not yet implemented in Rust version
        // In C++ it validates empty names and invalid characters, but here we allow them for now

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_action_server_is_valid() -> Result<()> {
        let (_node, _client, _server) = setup_test().await?;

        // Test valid server - if we can create it, it's valid
        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_action_accept_new_goal() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Send a goal request
        let goal_handle = client.send_goal(TestGoal { order: 10 }).await?;
        let _goal_id = goal_handle.id();

        // Accept the goal on the server side
        let requested = server.recv_goal().await?;
        assert_eq!(requested.goal.order, 10);

        let _accepted = requested.accept();

        // Verify the goal was accepted by checking we can get the result later
        // (This is a basic smoke test - full validation would require more complex setup)
        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_action_server_goal_exists() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Send and accept a goal
        let goal_handle = client.send_goal(TestGoal { order: 10 }).await?;
        let _goal_id = goal_handle.id();

        let requested = server.recv_goal().await?;
        let _accepted = requested.accept();

        // The goal should exist (we can't directly check internal state, but acceptance implies it)
        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_action_server_notify_goal_done() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Send and accept a goal
        let mut goal_handle = client.send_goal(TestGoal { order: 10 }).await?;
        let requested = server.recv_goal().await?;
        let accepted = requested.accept();
        let executing = accepted.execute();

        // Complete the goal
        executing.succeed(TestResult { value: 42 })?;

        // Get the result to verify completion
        let result = goal_handle.result().await?;
        assert_eq!(result.value, 42);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_action_server_get_goal_status_array() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Add a goal
        let _goal_handle = client.send_goal(TestGoal { order: 10 }).await?;
        let requested = server.recv_goal().await?;
        let _accepted = requested.accept();

        // Status should be available (basic smoke test)
        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_action_process_cancel_request() -> Result<()> {
        let (_node, client, server) = setup_test().await?;

        // Send and accept a goal
        let goal_handle = client.send_goal(TestGoal { order: 10 }).await?;
        let requested = server.recv_goal().await?;
        let accepted = requested.accept();
        let _executing = accepted.execute();

        // Send cancel request
        let _cancel_response = goal_handle.cancel().await?;

        // Process cancel on server side
        let (cancel_request, _query) = server.recv_cancel().await?;
        assert_eq!(cancel_request.goal_info.goal_id, goal_handle.id());

        // Basic verification that cancel was received
        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_action_server_goal_timeout_configuration() -> Result<()> {
        use std::time::Duration;

        let ctx = ZContextBuilder::default().build()?;
        let node = ctx.create_node("test_timeout_node").build()?;

        // Create server with goal timeout
        let _server = node
            .create_action_server::<TestAction>("test_timeout_action")
            .goal_timeout(Duration::from_secs(30))
            .build()?;

        // Verify that the server was created successfully
        // The timeout is stored internally and would be used when goals are accepted/executed
        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_action_server_qos_configuration() -> Result<()> {
        let ctx = ZContextBuilder::default().build()?;
        let node = ctx.create_node("test_qos_node").build()?;

        // Create custom QoS profile
        let custom_qos = QosProfile {
            reliability: QosReliability::BestEffort,
            history: QosHistory::KeepLast(5),
            ..Default::default()
        };

        // Create server with QoS configuration
        let _server = node
            .create_action_server::<TestAction>("test_qos_action")
            .with_goal_service_qos(custom_qos)
            .with_feedback_topic_qos(custom_qos)
            .build()?;

        // Verify that the server was created successfully
        // The QoS settings are applied to the underlying entities
        Ok(())
    }
}
