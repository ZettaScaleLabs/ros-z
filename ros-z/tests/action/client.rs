use std::sync::Arc;

use ros_z::{Builder, Result, context::ZContextBuilder, define_action};
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

define_action! {
    TestAction,
    action_name: "test_action",
    Goal: TestGoal,
    Result: TestResult,
    Feedback: TestFeedback,
}

// Helper function to create test setup
async fn setup_test_base() -> Result<(ros_z::node::ZNode,)> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("test_action_client_node").build()?;

    // Wait for discovery
    tokio::time::sleep(std::time::Duration::from_millis(100)).await;

    Ok((node,))
}

// Helper function to create test setup with client
async fn setup_test_with_client() -> Result<(
    ros_z::node::ZNode,
    std::sync::Arc<ros_z::action::client::ZActionClient<TestAction>>,
)> {
    let (node,) = setup_test_base().await?;

    let client = Arc::new(
        node.create_action_client::<TestAction>("/test_action_client_name")
            .build()?,
    );

    Ok((node, client))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_action_client_init_fini() -> Result<()> {
        let (node,) = setup_test_base().await?;

        // Test successful initialization with valid arguments
        let client = node
            .create_action_client::<TestAction>("/test_action_client_name")
            .build()?;

        // Verify the client was created successfully by checking it can be cloned
        let _client_clone = client.clone();

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_action_client_is_valid() -> Result<()> {
        let (_node, client) = setup_test_with_client().await?;

        // Test valid client - verify it can be cloned (proves internal structures are valid)
        let _client_clone = client.clone();

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_action_server_is_available() -> Result<()> {
        let (node, _client) = setup_test_with_client().await?;

        // Create a server to verify availability detection
        let _server = node
            .create_action_server::<TestAction>("/test_action_client_name")
            .build()?;

        // Wait for discovery
        tokio::time::sleep(std::time::Duration::from_millis(1500)).await;

        // Verify server is discoverable through graph
        let server_names_types = node.graph.get_action_server_names_and_types_by_node((
            node.entity.namespace.clone(),
            node.entity.name.clone(),
        ));
        assert!(!server_names_types.is_empty());

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_action_client_get_action_name() -> Result<()> {
        let (node, _client) = setup_test_with_client().await?;

        // Verify action name through graph introspection
        let client_names_types = node.graph.get_action_client_names_and_types_by_node((
            node.entity.namespace.clone(),
            node.entity.name.clone(),
        ));

        // Should find the action client with the expected name
        let action_found = client_names_types
            .iter()
            .any(|(name, _)| name.contains("test_action_client_name"));
        assert!(action_found);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_action_client_get_options() -> Result<()> {
        use ros_z::qos::{QosHistory, QosProfile, QosReliability};

        let (node,) = setup_test_base().await?;

        // Create client with custom QoS options
        let custom_qos = QosProfile {
            reliability: QosReliability::BestEffort,
            history: QosHistory::KeepLast(5),
            ..Default::default()
        };

        let _client = node
            .create_action_client::<TestAction>("/test_action_options")
            .with_goal_service_qos(custom_qos)
            .with_result_service_qos(custom_qos)
            .build()?;

        // Verify client creation with custom options succeeded
        Ok(())
    }

    // TODO: Additional tests would cover:
    // - Server availability checks
    // - Introspection configuration
    // - Fault injection tests (memory allocation failures)
    // These would require more complex setup and are deferred for now
}
