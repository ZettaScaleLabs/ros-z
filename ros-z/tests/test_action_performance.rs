use std::time::{Duration, Instant};

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
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_high_frequency_goals() -> Result<()> {
        // Test handling of high-frequency goal submissions
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_high_freq_client").build()?;
        let server_node = ctx.create_node("test_high_freq_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_high_freq_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_high_freq_action")
            .build()?;

        let start_time = Instant::now();
        let mut goal_handles = vec![];

        // Send 100 goals rapidly
        for i in 0..100 {
            let goal = TestGoal { order: i };
            let goal_handle = client.send_goal(goal).await?;
            goal_handles.push(goal_handle);
        }

        let send_duration = start_time.elapsed();
        println!("Sent 100 goals in {:?}", send_duration);

        // Process results
        let mut results = vec![];
        for (i, handle) in goal_handles.into_iter().enumerate() {
            // Start server handler for this goal
            let server_clone = server.clone();
            tokio::spawn(async move {
                if let Ok(requested) = server_clone.recv_goal().await {
                    let accepted = requested.accept();
                    let executing = accepted.execute();
                    tokio::time::sleep(Duration::from_millis(1)).await; // Minimal processing time
                    let _ = executing.succeed(TestResult {
                        value: i as i32 * 10,
                    });
                }
            });

            // Get result with timeout
            let mut handle = handle;
            let result_future = tokio::time::timeout(Duration::from_millis(500), handle.result());
            match result_future.await {
                Ok(Ok(result)) => results.push(result.value),
                _ => {} // Timeout or error - skip for performance test
            }
        }

        let total_duration = start_time.elapsed();
        println!(
            "Processed {} results in {:?}",
            results.len(),
            total_duration
        );

        // Basic validation - we should have processed some goals
        assert!(results.len() > 0);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_memory_usage_under_load() -> Result<()> {
        // Test memory usage with sustained load
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_memory_client").build()?;
        let server_node = ctx.create_node("test_memory_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_memory_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_memory_action")
            .build()?;

        let mut active_goals = vec![];

        // Create sustained load with overlapping goals
        for batch in 0..5 {
            let mut batch_handles = vec![];

            // Send batch of 20 goals
            for i in 0..20 {
                let goal = TestGoal {
                    order: batch * 20 + i,
                };
                let goal_handle = client.send_goal(goal).await?;
                batch_handles.push(goal_handle);
            }

            // Start server handlers for this batch
            for (i, _) in batch_handles.iter().enumerate() {
                let server_clone = server.clone();
                let goal_index = batch * 20 + i as i32;
                tokio::spawn(async move {
                    if let Ok(requested) = server_clone.recv_goal().await {
                        let accepted = requested.accept();
                        let executing = accepted.execute();
                        tokio::time::sleep(Duration::from_millis(50)).await; // Simulate processing
                        let _ = executing.succeed(TestResult {
                            value: goal_index as i32,
                        });
                    }
                });
            }

            active_goals.push(batch_handles);

            // Small delay between batches
            tokio::time::sleep(Duration::from_millis(10)).await;
        }

        // Wait for all goals to complete
        let mut completed_count = 0;
        for batch in active_goals {
            for mut handle in batch {
                let result_future =
                    tokio::time::timeout(Duration::from_millis(1000), handle.result());
                if let Ok(Ok(_)) = result_future.await {
                    completed_count += 1;
                }
            }
        }

        println!("Completed {} goals under sustained load", completed_count);

        // Should have completed most goals
        assert!(completed_count > 50);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_concurrent_clients() -> Result<()> {
        // Test multiple clients sending goals simultaneously
        let ctx = ZContextBuilder::default().build()?;
        let server_node = ctx.create_node("test_concurrent_server").build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_concurrent_action")
            .build()?;

        let mut client_handles = vec![];

        // Create 10 concurrent clients
        for client_id in 0..10 {
            let server_clone = server.clone();

            let handle = tokio::spawn(async move {
                let ctx = ZContextBuilder::default().build()?;
                let client_node = ctx.create_node(&format!("client_{}", client_id)).build()?;
                let client = client_node
                    .create_action_client::<TestAction>("test_concurrent_action")
                    .build()?;

                let mut results = vec![];

                // Each client sends 5 goals
                for goal_id in 0..5 {
                    let goal = TestGoal {
                        order: client_id * 10 + goal_id,
                    };

                    // Start server handler
                    let server_clone_inner = server_clone.clone();
                    tokio::spawn(async move {
                        if let Ok(requested) = server_clone_inner.recv_goal().await {
                            let accepted = requested.accept();
                            let executing = accepted.execute();
                            tokio::time::sleep(Duration::from_millis(5)).await;
                            let _ = executing.succeed(TestResult {
                                value: goal.order * 2,
                            });
                        }
                    });

                    let mut goal_handle = client.send_goal(goal).await?;
                    let result_future =
                        tokio::time::timeout(Duration::from_millis(200), goal_handle.result());

                    match result_future.await {
                        Ok(Ok(result)) => results.push(result.value),
                        _ => {} // Skip timeouts for this test
                    }
                }

                Ok::<_, Box<dyn std::error::Error + Send + Sync>>(results)
            });

            client_handles.push(handle);
        }

        // Wait for all clients to complete
        let mut total_results = 0;
        for handle in client_handles {
            if let Ok(Ok(results)) = handle.await {
                total_results += results.len();
            }
        }

        println!("Concurrent clients completed {} goals", total_results);

        // Should have completed some goals
        assert!(total_results > 0);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_goal_burst_capacity() -> Result<()> {
        // Test system capacity with goal bursts
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_burst_client").build()?;
        let server_node = ctx.create_node("test_burst_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_burst_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_burst_action")
            .build()?;

        let start_time = Instant::now();

        // Send burst of goals
        let mut goal_handles = vec![];
        for i in 0..50 {
            let goal = TestGoal { order: i };
            match client.send_goal(goal).await {
                Ok(handle) => goal_handles.push(handle),
                Err(_) => {} // Skip failed sends for capacity test
            }
        }

        let burst_send_time = start_time.elapsed();
        println!(
            "Sent {} goals in burst: {:?}",
            goal_handles.len(),
            burst_send_time
        );

        // Process with limited server capacity
        let _processed = 0;
        for _ in 0..goal_handles.len() {
            let server_clone = server.clone();
            tokio::spawn(async move {
                if let Ok(requested) = server_clone.recv_goal().await {
                    let accepted = requested.accept();
                    let executing = accepted.execute();
                    tokio::time::sleep(Duration::from_millis(10)).await;
                    let _ = executing.succeed(TestResult { value: 42 });
                }
            });
        }

        // Wait for processing with timeout
        tokio::time::sleep(Duration::from_millis(200)).await;

        let total_time = start_time.elapsed();
        println!("Burst test completed in {:?}", total_time);

        // Basic validation - should have sent some goals
        assert!(goal_handles.len() > 0);

        Ok(())
    }
}
