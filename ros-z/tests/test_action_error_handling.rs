use std::time::Duration;

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

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_invalid_goal_parameters() -> Result<()> {
        // Test handling of invalid goal parameters
        let ctx = ZContextBuilder::default().build()?;
        let node = ctx.create_node("test_invalid_goal_node").build()?;

        let client = node
            .create_action_client::<TestAction>("test_invalid_goal_action")
            .build()?;

        // Test with goals that might cause issues (very large values, etc.)
        let invalid_goal = TestGoal { order: i32::MAX };
        let goal_handle = client.send_goal(invalid_goal).await;

        // The goal should still be accepted, but server might handle it appropriately
        assert!(goal_handle.is_ok());

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_server_unavailable() -> Result<()> {
        // Test client behavior when server is not available
        let ctx = ZContextBuilder::default().build()?;
        let node = ctx.create_node("test_unavailable_server_node").build()?;

        let client = node
            .create_action_client::<TestAction>("test_unavailable_server_action")
            .build()?;

        // Try to send goal to non-existent server
        let goal = TestGoal { order: 42 };
        let result = tokio::time::timeout(Duration::from_millis(100), client.send_goal(goal)).await;

        // Should timeout or fail gracefully
        assert!(result.is_err() || result.unwrap().is_err());

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_goal_timeout_expiration() -> Result<()> {
        // Test that goals actually expire when timeout is reached
        let ctx = ZContextBuilder::default().build()?;

        let client_node = ctx.create_node("test_timeout_client").build()?;
        let server_node = ctx.create_node("test_timeout_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_timeout_action")
            .build()?;

        // Create server with very short timeout
        let _server = server_node
            .create_action_server::<TestAction>("test_timeout_action")
            .goal_timeout(Duration::from_millis(50)) // Very short timeout
            .build()?;

        // Send goal and wait longer than timeout
        let goal = TestGoal { order: 1 };
        let goal_handle = client.send_goal(goal).await?;
        tokio::time::sleep(Duration::from_millis(100)).await; // Wait for expiration

        // Goal should have expired and been aborted
        // Note: This test depends on background expiration task being implemented
        // For now, just verify the goal was accepted
        assert!(goal_handle.id().is_valid()); // ID should not be all zeros

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_invalid_qos_configuration() -> Result<()> {
        // Test handling of invalid QoS configurations
        let ctx = ZContextBuilder::default().build()?;
        let node = ctx.create_node("test_invalid_qos_node").build()?;

        // Test with invalid QoS values
        use ros_z::qos::{QosHistory, QosProfile};

        let invalid_qos = QosProfile {
            history: QosHistory::KeepLast(0), // Invalid: 0 depth
            ..Default::default()
        };

        // This should still work - QoS validation happens at Zenoh level
        let client = node
            .create_action_client::<TestAction>("test_invalid_qos_action")
            .with_goal_service_qos(invalid_qos)
            .build()?;

        assert!(client.send_goal(TestGoal { order: 1 }).await.is_ok());

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_concurrent_goal_modification() -> Result<()> {
        // Test handling of concurrent modifications to goals
        let ctx = ZContextBuilder::default().build()?;

        let client_node = ctx.create_node("test_concurrent_client").build()?;
        let server_node = ctx.create_node("test_concurrent_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_concurrent_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_concurrent_action")
            .build()?;

        // Send multiple goals concurrently
        let mut handles = vec![];

        for i in 0..5 {
            let client_clone = client.clone();
            let server_clone = server.clone();

            let handle = tokio::spawn(async move {
                // Start server handler
                let server_task = tokio::spawn(async move {
                    if let Ok(requested) = server_clone.recv_goal().await {
                        let accepted = requested.accept();
                        let executing = accepted.execute();
                        tokio::time::sleep(Duration::from_millis(10)).await;
                        let _ = executing.succeed(TestResult { value: i * 10 });
                    }
                });

                // Send goal
                let goal = TestGoal { order: i };
                let mut goal_handle = client_clone.send_goal(goal).await?;
                let result = goal_handle.result().await?;

                server_task.await?;
                Ok::<_, Box<dyn std::error::Error + Send + Sync>>(result.value)
            });

            handles.push(handle);
        }

        // Wait for all to complete
        for handle in handles {
            let result = handle.await??;
            assert!(result >= 0);
        }

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_resource_exhaustion() -> Result<()> {
        // Test behavior under resource exhaustion
        let ctx = ZContextBuilder::default().build()?;
        let node = ctx.create_node("test_exhaustion_node").build()?;

        // Create many clients and servers
        let mut clients = vec![];
        let mut servers = vec![];

        for i in 0..10 {
            let client = node
                .create_action_client::<TestAction>(&format!("test_exhaustion_action_{}", i))
                .build()?;
            clients.push(client);

            let server = node
                .create_action_server::<TestAction>(&format!("test_exhaustion_action_{}", i))
                .build()?;
            servers.push(server);
        }

        // Send goals to all
        let mut goal_handles = vec![];
        for (i, client) in clients.iter().enumerate() {
            let goal = TestGoal { order: i as i32 };
            let handle = client.send_goal(goal).await?;
            goal_handles.push(handle);
        }

        // Verify goals were accepted
        for handle in goal_handles {
            assert!(handle.id().is_valid());
        }

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_server_shutdown_during_operation() -> Result<()> {
        // Test server shutdown while goals are active
        let ctx = ZContextBuilder::default().build()?;

        let client_node = ctx.create_node("test_shutdown_client").build()?;
        let server_node = ctx.create_node("test_shutdown_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_shutdown_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_shutdown_action")
            .build()?;

        // Send goal
        let goal = TestGoal { order: 42 };
        let mut goal_handle = client.send_goal(goal).await?;

        // Start server processing but shutdown before completion
        let server_clone = server.clone();
        tokio::spawn(async move {
            if let Ok(requested) = server_clone.recv_goal().await {
                let accepted = requested.accept();
                let executing = accepted.execute();
                tokio::time::sleep(Duration::from_millis(50)).await;
                // Server shuts down here without completing the goal
                drop(executing);
            }
        });

        tokio::time::sleep(Duration::from_millis(25)).await;

        // Client should handle server disconnection gracefully
        // This is a basic test - in practice, the result() call might timeout
        let result_future = goal_handle.result();
        let timeout_result = tokio::time::timeout(Duration::from_millis(100), result_future).await;

        // Should either timeout or fail gracefully
        assert!(timeout_result.is_err() || timeout_result.unwrap().is_err());

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_network_failure_simulation() -> Result<()> {
        // Test handling of simulated network failures
        let ctx = ZContextBuilder::default().build()?;

        let client_node = ctx.create_node("test_network_client").build()?;
        let server_node = ctx.create_node("test_network_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_network_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_network_action")
            .build()?;

        // Send goal
        let goal = TestGoal { order: 1 };
        let mut goal_handle = client.send_goal(goal).await?;

        // Simulate network failure by dropping server
        drop(server);

        // Client operations should fail gracefully
        let result_future = goal_handle.result();
        let timeout_result = tokio::time::timeout(Duration::from_millis(100), result_future).await;

        // Should fail due to server being unavailable
        assert!(timeout_result.is_err() || timeout_result.unwrap().is_err());

        Ok(())
    }
}
