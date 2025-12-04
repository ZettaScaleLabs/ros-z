use ros_z::{Builder, Result, action::ZAction, context::ZContextBuilder};
use serde::{Deserialize, Serialize};

// Define test action messages
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
async fn setup_test_with_action_name(
    action_name: &str,
) -> Result<(
    ros_z::node::ZNode,
    ros_z::action::client::ZActionClient<TestAction>,
    std::sync::Arc<ros_z::action::server::ZActionServer<TestAction>>,
)> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("test_names_node").build()?;

    let client = node
        .create_action_client::<TestAction>(action_name)
        .build()?;

    let server = node
        .create_action_server::<TestAction>(action_name)
        .build()?;

    // Wait for discovery
    tokio::time::sleep(std::time::Duration::from_millis(100)).await;

    Ok((node, client, server))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_action_name_communication() -> Result<()> {
        let (_node, client, server) = setup_test_with_action_name("test_action_names").await?;

        // Set up server handler
        let _server_handle = server.with_handler(|executing| async move {
            executing.succeed(TestResult { value: 42 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal and verify communication works with the action name
        let mut goal_handle = client.send_goal(TestGoal { order: 10 }).await?;
        let result = goal_handle.result().await?;

        assert_eq!(result.value, 42);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_action_name_with_underscores() -> Result<()> {
        let (_node, client, server) =
            setup_test_with_action_name("test_action_with_underscores").await?;

        // Set up server handler
        let _server_handle = server.with_handler(|executing| async move {
            executing.succeed(TestResult { value: 123 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal and verify communication works
        let mut goal_handle = client.send_goal(TestGoal { order: 20 }).await?;
        let result = goal_handle.result().await?;

        assert_eq!(result.value, 123);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_action_name_with_namespace() -> Result<()> {
        let (_node, client, server) = setup_test_with_action_name("/namespace/test_action").await?;

        // Set up server handler
        let _server_handle = server.with_handler(|executing| async move {
            executing.succeed(TestResult { value: 456 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Send goal and verify communication works with namespaced action
        let mut goal_handle = client.send_goal(TestGoal { order: 30 }).await?;
        let result = goal_handle.result().await?;

        assert_eq!(result.value, 456);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_multiple_actions_different_names() -> Result<()> {
        let ctx = ZContextBuilder::default().build()?;
        let node = ctx.create_node("test_multiple_names_node").build()?;

        // Create two different actions with different names
        let client1 = node
            .create_action_client::<TestAction>("action_one")
            .build()?;

        let server1 = node
            .create_action_server::<TestAction>("action_one")
            .build()?;

        let client2 = node
            .create_action_client::<TestAction>("action_two")
            .build()?;

        let server2 = node
            .create_action_server::<TestAction>("action_two")
            .build()?;

        // Set up different handlers for each server
        let _server_handle1 = server1.with_handler(|executing| async move {
            executing.succeed(TestResult { value: 111 }).unwrap();
        });

        let _server_handle2 = server2.with_handler(|executing| async move {
            executing.succeed(TestResult { value: 222 }).unwrap();
        });

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Test communication with both actions
        let mut goal_handle1 = client1.send_goal(TestGoal { order: 1 }).await?;
        let result1 = goal_handle1.result().await?;
        assert_eq!(result1.value, 111);

        let mut goal_handle2 = client2.send_goal(TestGoal { order: 2 }).await?;
        let result2 = goal_handle2.result().await?;
        assert_eq!(result2.value, 222);

        Ok(())
    }

    // Additional tests would cover:
    // - Invalid action names (empty, starting with numbers, etc.) - handled by builder
    // - Action name expansion and qualification - handled internally
    // - Service and topic name derivations - handled internally
    // The Rust API abstracts these details, so we test the high-level functionality
}
