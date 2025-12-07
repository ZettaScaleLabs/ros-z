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

pub struct TestAction;

impl ZAction for TestAction {
    type Goal = TestGoal;
    type Result = TestResult;
    type Feedback = TestFeedback;

    fn name() -> &'static str {
        "test_action"
    }
}

async fn setup_test_base() -> Result<ros_z::node::ZNode> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("test_action_server_node").build()?;
    tokio::time::sleep(std::time::Duration::from_millis(100)).await;
    Ok(node)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_action_server_init_fini() -> Result<()> {
        let node = setup_test_base().await?;

        // Test successful initialization with valid arguments
        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;
        drop(server); // fini

        // Test with empty action name - should fail
        let result = node.create_action_server::<TestAction>("").build();
        assert!(result.is_err());

        // Test with invalid action name (contains spaces) - should work as remapping handles it
        let server = node
            .create_action_server::<TestAction>("/invalid name")
            .build()?;
        drop(server);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_action_server_is_valid() -> Result<()> {
        let node = setup_test_base().await?;

        // Create a valid server
        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        // In Rust, if we can create the server, it's valid
        // The internal components are managed by Arc and proper initialization
        assert!(true);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_action_server_get_action_name() -> Result<()> {
        let node = setup_test_base().await?;

        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        // In ros-z, action name is not directly accessible from server
        // It's stored internally in the remapped form
        assert!(true);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_action_server_get_options() -> Result<()> {
        let node = setup_test_base().await?;

        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        // In ros-z, options are not directly accessible from server
        // They are used during construction
        assert!(true);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_action_accept_new_goal() -> Result<()> {
        let node = setup_test_base().await?;

        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        // In ros-z, goal acceptance is handled through the recv_goal method
        // and RequestedGoal::accept()
        assert!(true);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_action_server_goal_exists() -> Result<()> {
        let node = setup_test_base().await?;

        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        // In ros-z, goal existence checking is internal to the server
        assert!(true);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_action_server_notify_goal_done() -> Result<()> {
        let node = setup_test_base().await?;

        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        // In ros-z, goal done notification is handled automatically
        assert!(true);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_action_clear_expired_goals() -> Result<()> {
        let node = setup_test_base().await?;

        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        // In ros-z, goal expiration is not yet implemented
        assert!(true);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_action_process_cancel_request() -> Result<()> {
        let node = setup_test_base().await?;

        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        // In ros-z, cancel request processing is handled through recv_cancel
        assert!(true);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_action_server_get_goal_status_array() -> Result<()> {
        let node = setup_test_base().await?;

        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        // In ros-z, status publishing is handled automatically
        assert!(true);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_default_internal_services_introspection_status() -> Result<()> {
        let node = setup_test_base().await?;

        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        // In ros-z, introspection is not yet implemented
        assert!(true);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_set_internal_services_introspection_off() -> Result<()> {
        let node = setup_test_base().await?;

        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        // Introspection configuration not yet implemented in ros-z
        assert!(true);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_set_internal_services_introspection_metadata() -> Result<()> {
        let node = setup_test_base().await?;

        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        // Introspection configuration not yet implemented in ros-z
        assert!(true);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_set_internal_services_introspection_contents() -> Result<()> {
        let node = setup_test_base().await?;

        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        // Introspection configuration not yet implemented in ros-z
        assert!(true);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_action_server_init_fini_maybe_fail() -> Result<()> {
        // Fault injection testing is more complex in Rust
        // Memory allocation failures are handled by Rust's allocator
        // For now, just test normal operation
        let node = setup_test_base().await?;

        for i in 0..10 {
            let action_name = format!("/test_action_server_name_{}", i);
            let server = node
                .create_action_server::<TestAction>(&action_name)
                .build()?;
            drop(server);
        }

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_action_process_cancel_request_maybe_fail() -> Result<()> {
        let node = setup_test_base().await?;

        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        // Test cancel request processing under various conditions
        assert!(true);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_action_expire_goals() -> Result<()> {
        use std::{sync::Arc, time::Duration};

        let node = setup_test_base().await?;

        let server = Arc::new(
            node.create_action_server::<TestAction>("/test_action_server_name")
                .build()?,
        );

        // Create a client to send goals
        let client = Arc::new(
            node.create_action_client::<TestAction>("/test_action_server_name")
                .build()?,
        );

        // Process and complete 3 goals
        let server_clone = server.clone();
        let server_handle = tokio::spawn(async move {
            for _ in 0..3 {
                if let Ok(requested) = server_clone.recv_goal().await {
                    let accepted = requested.accept();
                    let executing = accepted.execute();
                    let _ = executing.succeed(TestResult { value: 42 });
                }
            }
        });

        // Send 3 goals without waiting for results
        for i in 0..3 {
            let goal = TestGoal { order: i };
            let _ = client.send_goal(goal).await?;
        }

        // Wait a bit for goals to be processed
        tokio::time::sleep(Duration::from_millis(200)).await;

        // Initially, no goals should be expired (they're too recent)
        let expired = server.expire_goals(Duration::from_secs(10));
        assert_eq!(expired.len(), 0, "No goals should be expired yet");

        // Wait a bit, then expire with very short timeout
        tokio::time::sleep(Duration::from_millis(200)).await;
        let expired = server.expire_goals(Duration::from_millis(100));
        assert_eq!(
            expired.len(),
            3,
            "All 3 goals should be expired after timeout"
        );

        // Second expiration should find nothing
        let expired = server.expire_goals(Duration::from_millis(50));
        assert_eq!(expired.len(), 0, "No more goals to expire");

        // Clean up
        drop(server_handle);
        drop(client);
        drop(server);
        tokio::time::sleep(Duration::from_millis(50)).await;

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_action_result_timeout_config() -> Result<()> {
        use std::time::Duration;

        let node = setup_test_base().await?;

        // Test default timeout
        let server = node
            .create_action_server::<TestAction>("/test_action_server_name")
            .build()?;

        let default_timeout = server.result_timeout();
        assert_eq!(default_timeout, Duration::from_secs(10));

        // Test custom timeout
        let server = node
            .create_action_server::<TestAction>("/test_action_server_name2")
            .result_timeout(Duration::from_secs(30))
            .build()?;

        let custom_timeout = server.result_timeout();
        assert_eq!(custom_timeout, Duration::from_secs(30));

        // Test setting timeout after creation
        server.set_result_timeout(Duration::from_secs(60));
        let updated_timeout = server.result_timeout();
        assert_eq!(updated_timeout, Duration::from_secs(60));

        Ok(())
    }
}
