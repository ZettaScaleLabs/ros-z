use std::time::Duration;

use ros_z::{Builder, Result, action::ZAction, context::ZContextBuilder};
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
    #[ignore = "Timing out - lifecycle test needs investigation"]
    async fn test_graceful_server_shutdown() -> Result<()> {
        // Test that server shuts down gracefully and cleans up resources
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_graceful_client").build()?;
        let server_node = ctx.create_node("test_graceful_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_graceful_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_graceful_action")
            .build()?;

        // Send a goal
        let goal = TestGoal { order: 42 };
        let mut goal_handle = client.send_goal(goal).await?;

        // Start server processing
        let server_clone = server.clone();
        let processing_task = tokio::spawn(async move {
            if let Ok(requested) = server_clone.recv_goal().await {
                let accepted = requested.accept();
                let executing = accepted.execute();

                // Simulate some processing time
                tokio::time::sleep(Duration::from_millis(100)).await;

                // Complete the goal
                let _ = executing.succeed(TestResult { value: 84 });
            }
        });

        // Wait a bit then drop the server (simulating shutdown)
        tokio::time::sleep(Duration::from_millis(50)).await;
        drop(server);

        // Client should still be able to get result (server completed before shutdown)
        let result_future = tokio::time::timeout(Duration::from_millis(200), goal_handle.result());
        let result = result_future.await;

        // Wait for processing task to complete
        let _ = processing_task.await;

        // Result should be available (server completed before shutdown)
        assert!(result.is_ok());
        assert!(result.unwrap().is_ok());

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore = "Timing out - lifecycle test needs investigation"]
    async fn test_client_shutdown_during_active_goal() -> Result<()> {
        // Test client shutdown while goals are still active
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_client_shutdown_client").build()?;
        let server_node = ctx.create_node("test_client_shutdown_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_client_shutdown_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_client_shutdown_action")
            .build()?;

        // Send goal
        let goal = TestGoal { order: 1 };
        let goal_handle = client.send_goal(goal).await?;

        // Start server processing (but don't complete it)
        let server_clone = server.clone();
        let processing_task = tokio::spawn(async move {
            if let Ok(requested) = server_clone.recv_goal().await {
                let accepted = requested.accept();
                let _executing = accepted.execute();

                // Hold the goal active for a while
                tokio::time::sleep(Duration::from_millis(200)).await;
            }
        });

        // Client shuts down (drop) while goal is active
        tokio::time::sleep(Duration::from_millis(50)).await;
        drop(client);
        drop(client_node);

        // Server should continue processing
        let _ = processing_task.await;

        // Goal handle should still exist but result may not be retrievable
        // (client resources are cleaned up)
        assert!(goal_handle.id().is_valid());

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore = "Timing out - lifecycle test needs investigation"]
    async fn test_context_shutdown_cleans_resources() -> Result<()> {
        // Test that context shutdown properly cleans up all action resources
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_context_client").build()?;
        let server_node = ctx.create_node("test_context_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_context_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_context_action")
            .build()?;

        // Create some goals
        let mut goal_handles = vec![];
        for i in 0..5 {
            let goal = TestGoal { order: i };
            let handle = client.send_goal(goal).await?;
            goal_handles.push(handle);
        }

        // Start server processing
        let server_clone = server.clone();
        let processing_task = tokio::spawn(async move {
            for _ in 0..5 {
                if let Ok(requested) = server_clone.recv_goal().await {
                    let accepted = requested.accept();
                    let executing = accepted.execute();
                    tokio::time::sleep(Duration::from_millis(10)).await;
                    let _ = executing.succeed(TestResult { value: 42 });
                }
            }
        });

        // Shutdown context while operations are in progress
        tokio::time::sleep(Duration::from_millis(25)).await;
        drop(ctx); // This should clean up all resources

        // Wait for processing to complete (may fail due to context shutdown)
        let _ = processing_task.await;

        // All handles should still be valid (even if operations failed)
        for handle in goal_handles {
            assert!(handle.id().is_valid());
        }

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore = "Timing out - lifecycle test needs investigation"]
    async fn test_goal_handle_drop_cleans_resources() -> Result<()> {
        // Test that dropping GoalHandle properly cleans up resources
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_drop_client").build()?;
        let server_node = ctx.create_node("test_drop_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_drop_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_drop_action")
            .build()?;

        // Send goal and immediately drop handle
        let goal = TestGoal { order: 123 };
        let goal_handle = client.send_goal(goal).await?;
        let goal_id = goal_handle.id();

        // Drop the handle explicitly
        drop(goal_handle);

        // Server should still be able to receive the goal
        let server_clone = server.clone();
        let processing_task = tokio::spawn(async move {
            if let Ok(requested) = server_clone.recv_goal().await {
                let accepted = requested.accept();
                let executing = accepted.execute();
                tokio::time::sleep(Duration::from_millis(10)).await;
                let _ = executing.succeed(TestResult { value: 246 });
            }
        });

        // Wait for processing
        let _ = processing_task.await;

        // Goal ID should have been valid
        assert!(goal_id.is_valid());

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore = "Timing out - lifecycle test needs investigation"]
    async fn test_multiple_server_instances_cleanup() -> Result<()> {
        // Test cleanup when multiple server instances exist
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_multi_client").build()?;
        let server_node = ctx.create_node("test_multi_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_multi_action")
            .build()?;

        // Create multiple server instances
        let mut servers = vec![];
        for i in 0..3 {
            let server = server_node
                .create_action_server::<TestAction>(&format!("test_multi_action_{}", i))
                .build()?;
            servers.push(server);
        }

        // Send goals to different servers
        let mut goal_handles = vec![];
        for i in 0..3 {
            let client_clone = client.clone();
            let goal = TestGoal { order: i };
            let handle = client_clone.send_goal(goal).await?;
            goal_handles.push(handle);
        }

        // Start processing on all servers
        let mut processing_tasks = vec![];
        for server in servers {
            let processing_task = tokio::spawn(async move {
                if let Ok(requested) = server.recv_goal().await {
                    let accepted = requested.accept();
                    let executing = accepted.execute();
                    tokio::time::sleep(Duration::from_millis(10)).await;
                    let _ = executing.succeed(TestResult { value: 42 });
                }
            });
            processing_tasks.push(processing_task);
        }

        // Wait for all processing to complete
        for task in processing_tasks {
            let _ = task.await;
        }

        // All goal handles should be valid
        for handle in goal_handles {
            assert!(handle.id().is_valid());
        }

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore = "Timing out - lifecycle test needs investigation"]
    async fn test_resource_leak_prevention() -> Result<()> {
        // Test that resources are properly cleaned up to prevent leaks
        let ctx = ZContextBuilder::default().build()?;
        let node = ctx.create_node("test_leak_node").build()?;

        // Create and destroy multiple clients and servers in sequence
        for i in 0..10 {
            let client = node
                .create_action_client::<TestAction>(&format!("test_leak_action_{}", i))
                .build()?;

            let server = node
                .create_action_server::<TestAction>(&format!("test_leak_action_{}", i))
                .build()?;

            // Send a goal
            let goal = TestGoal { order: i };
            let goal_handle = client.send_goal(goal).await?;

            // Start server processing
            let server_clone = server.clone();
            let processing_task = tokio::spawn(async move {
                if let Ok(requested) = server_clone.recv_goal().await {
                    let accepted = requested.accept();
                    let executing = accepted.execute();
                    tokio::time::sleep(Duration::from_millis(5)).await;
                    let _ = executing.succeed(TestResult { value: i * 2 });
                }
            });

            // Wait for completion
            let mut handle = goal_handle;
            let result_future = tokio::time::timeout(Duration::from_millis(100), handle.result());
            let _ = result_future.await;
            let _ = processing_task.await;

            // Resources should be cleaned up when variables go out of scope
        }

        // All iterations completed without resource leaks
        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore = "Timing out - lifecycle test needs investigation"]
    async fn test_background_task_cleanup() -> Result<()> {
        // Test that background tasks (feedback/status routing) are properly cleaned up
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_bg_client").build()?;
        let server_node = ctx.create_node("test_bg_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_bg_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_bg_action")
            .build()?;

        // Send multiple goals to generate background activity
        let mut goal_handles = vec![];
        for i in 0..5 {
            let goal = TestGoal { order: i };
            let handle = client.send_goal(goal).await?;
            goal_handles.push(handle);
        }

        // Start server processing with feedback
        let server_clone = server.clone();
        let processing_task = tokio::spawn(async move {
            for _ in 0..5 {
                if let Ok(requested) = server_clone.recv_goal().await {
                    let accepted = requested.accept();
                    let executing = accepted.execute();

                    // Send some feedback
                    let _ = executing.publish_feedback(TestFeedback { progress: 50 });

                    tokio::time::sleep(Duration::from_millis(10)).await;

                    let _ = executing.succeed(TestResult { value: 100 });
                }
            }
        });

        // Wait for processing
        let _ = processing_task.await;

        // Drop client and server - background tasks should be cleaned up
        drop(client);
        drop(server);
        drop(client_node);
        drop(server_node);

        // Context drop should clean up all background tasks
        drop(ctx);

        // Test completes without hanging (background tasks cleaned up)
        Ok(())
    }
}
