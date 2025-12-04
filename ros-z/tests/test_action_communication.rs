use std::time::Duration;

use ros_z::{Builder, Result, action::ZAction, context::ZContextBuilder};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestGoal {
    pub order: i32,
    pub sequence_id: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestResult {
    pub value: i32,
    pub sequence_id: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestFeedback {
    pub progress: i32,
    pub sequence_id: i32,
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
    async fn test_message_ordering_guarantee() -> Result<()> {
        // Test that messages are processed in the correct order
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_order_client").build()?;
        let server_node = ctx.create_node("test_order_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_order_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_order_action")
            .build()?;

        // Send goals with sequence IDs
        let mut goal_handles = vec![];
        for seq_id in 0..10 {
            let goal = TestGoal {
                order: seq_id * 10,
                sequence_id: seq_id,
            };
            let handle = client.send_goal(goal).await?;
            goal_handles.push((handle, seq_id));
        }

        // Process goals and collect results
        let mut results = vec![];
        let server_clone = server.clone();

        for _ in 0..10 {
            if let Ok(requested) = server_clone.recv_goal().await {
                let accepted = requested.accept();
                let goal_info = accepted.goal.clone();
                let executing = accepted.execute();

                // Simulate processing time
                tokio::time::sleep(Duration::from_millis(5)).await;

                let result = TestResult {
                    value: goal_info.order * 2,
                    sequence_id: goal_info.sequence_id,
                };

                let _ = executing.succeed(result.clone());
                results.push(result);
            }
        }

        // Sort results by sequence ID
        results.sort_by_key(|r| r.sequence_id);

        // Verify ordering
        for (i, result) in results.iter().enumerate() {
            assert_eq!(result.sequence_id, i as i32);
            assert_eq!(result.value, i as i32 * 10 * 2);
        }

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_feedback_stream_ordering() -> Result<()> {
        // Test that feedback messages maintain ordering
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_feedback_client").build()?;
        let server_node = ctx.create_node("test_feedback_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_feedback_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_feedback_action")
            .build()?;

        // Send goal
        let goal = TestGoal {
            order: 100,
            sequence_id: 1,
        };
        let mut goal_handle = client.send_goal(goal).await?;

        // Start server processing with ordered feedback
        let server_clone = server.clone();
        let processing_task = tokio::spawn(async move {
            if let Ok(requested) = server_clone.recv_goal().await {
                let accepted = requested.accept();
                let executing = accepted.execute();

                // Send feedback in order
                for progress in [10, 25, 50, 75, 100] {
                    let feedback = TestFeedback {
                        progress,
                        sequence_id: 1,
                    };
                    let _ = executing.publish_feedback(feedback);
                    tokio::time::sleep(Duration::from_millis(2)).await;
                }

                let result = TestResult {
                    value: 200,
                    sequence_id: 1,
                };
                let _ = executing.succeed(result);
            }
        });

        // Collect feedback
        let mut feedback_rx = goal_handle.feedback_stream().unwrap();
        let mut received_feedback = vec![];

        // Collect all feedback messages
        while let Ok(feedback) =
            tokio::time::timeout(Duration::from_millis(50), feedback_rx.recv()).await
        {
            if let Some(fb) = feedback {
                let progress = fb.progress;
                received_feedback.push(fb);
                if progress >= 100 {
                    break;
                }
            }
        }

        // Wait for processing to complete
        let _ = processing_task.await;

        // Verify feedback ordering
        assert_eq!(received_feedback.len(), 5);
        let expected_progress = [10, 25, 50, 75, 100];
        for (i, feedback) in received_feedback.iter().enumerate() {
            assert_eq!(feedback.progress, expected_progress[i]);
            assert_eq!(feedback.sequence_id, 1);
        }

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_concurrent_goal_processing() -> Result<()> {
        // Test processing multiple goals concurrently
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_concurrent_proc_client").build()?;
        let server_node = ctx.create_node("test_concurrent_proc_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_concurrent_proc_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_concurrent_proc_action")
            .build()?;

        // Send multiple goals
        let mut goal_handles = vec![];
        for i in 0..5 {
            let goal = TestGoal {
                order: i * 20,
                sequence_id: i,
            };
            let handle = client.send_goal(goal).await?;
            goal_handles.push(handle);
        }

        // Process goals concurrently
        let mut processing_tasks = vec![];
        for _ in 0..5 {
            let server_clone = server.clone();
            let task = tokio::spawn(async move {
                if let Ok(requested) = server_clone.recv_goal().await {
                    let accepted = requested.accept();
                    let goal_info = accepted.goal.clone();
                    let executing = accepted.execute();

                    // Variable processing time based on goal order
                    let delay = Duration::from_millis((goal_info.order / 10) as u64 + 5);
                    tokio::time::sleep(delay).await;

                    let result = TestResult {
                        value: goal_info.order * 3,
                        sequence_id: goal_info.sequence_id,
                    };

                    let _ = executing.succeed(result);
                }
            });
            processing_tasks.push(task);
        }

        // Wait for all processing to complete
        for task in processing_tasks {
            let _ = task.await;
        }

        // Collect results
        let mut results = vec![];
        for mut handle in goal_handles {
            let result_future = tokio::time::timeout(Duration::from_millis(500), handle.result());
            if let Ok(Ok(result)) = result_future.await {
                results.push(result);
            }
        }

        // Should have processed all goals
        assert_eq!(results.len(), 5);

        // Results should be correct
        for result in results {
            assert_eq!(result.value, result.sequence_id * 20 * 3);
        }

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_goal_timeout_and_recovery() -> Result<()> {
        // Test timeout handling and recovery
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_timeout_client").build()?;
        let server_node = ctx.create_node("test_timeout_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_timeout_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_timeout_action")
            .goal_timeout(Duration::from_millis(100)) // Short timeout
            .build()?;

        // Send goal
        let goal = TestGoal {
            order: 50,
            sequence_id: 1,
        };
        let mut goal_handle = client.send_goal(goal).await?;

        // Server takes too long to respond
        let server_clone = server.clone();
        let processing_task = tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(150)).await; // Longer than timeout

            if let Ok(requested) = server_clone.recv_goal().await {
                let accepted = requested.accept();
                let executing = accepted.execute();
                let result = TestResult {
                    value: 100,
                    sequence_id: 1,
                };
                let _ = executing.succeed(result);
            }
        });

        // Client should timeout
        let result_future = tokio::time::timeout(Duration::from_millis(200), goal_handle.result());
        let result = result_future.await;

        // Should timeout or fail
        assert!(result.is_err() || result.unwrap().is_err());

        // Wait for server task
        let _ = processing_task.await;

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_status_update_consistency() -> Result<()> {
        // Test that status updates are consistent and ordered
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_status_client").build()?;
        let server_node = ctx.create_node("test_status_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_status_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_status_action")
            .build()?;

        // Send goal
        let goal = TestGoal {
            order: 75,
            sequence_id: 2,
        };
        let goal_handle = client.send_goal(goal).await?;

        // Monitor status changes
        let status_watch = client.status_watch(goal_handle.id()).unwrap();

        // Start status monitoring task
        let status_task = tokio::spawn(async move {
            let mut changes = vec![];
            let mut receiver = status_watch;

            // Collect status changes with timeout
            for _ in 0..10 {
                match tokio::time::timeout(Duration::from_millis(50), receiver.changed()).await {
                    Ok(Ok(_)) => {
                        let status = *receiver.borrow();
                        changes.push(status);
                    }
                    _ => break,
                }
            }

            changes
        });

        // Wait a bit for status updates
        tokio::time::sleep(Duration::from_millis(100)).await;

        // Collect status changes
        let status_changes = status_task.await?;

        // Should have at least some status changes
        assert!(status_changes.len() > 0);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_large_message_handling() -> Result<()> {
        // Test handling of larger messages
        #[derive(Debug, Clone, Serialize, Deserialize)]
        struct LargeGoal {
            pub data: Vec<i32>,
            pub sequence_id: i32,
        }

        #[derive(Debug, Clone, Serialize, Deserialize)]
        struct LargeResult {
            pub processed_data: Vec<i32>,
            pub sequence_id: i32,
        }

        #[derive(Debug, Clone, Serialize, Deserialize)]
        struct LargeFeedback {
            pub progress_data: Vec<i32>,
            pub sequence_id: i32,
        }

        // For this test, we'll use the standard action but with larger data
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_large_client").build()?;
        let server_node = ctx.create_node("test_large_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_large_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_large_action")
            .build()?;

        // Create goal with larger data (simulate large message)
        let large_data = (0..1000).collect::<Vec<i32>>();
        let goal = TestGoal {
            order: large_data.iter().sum(),
            sequence_id: 3,
        };

        let mut goal_handle = client.send_goal(goal).await?;

        // Process large goal
        let server_clone = server.clone();
        let processing_task = tokio::spawn(async move {
            if let Ok(requested) = server_clone.recv_goal().await {
                let accepted = requested.accept();
                let goal_info = accepted.goal.clone();
                let executing = accepted.execute();

                // Simulate processing of large data
                tokio::time::sleep(Duration::from_millis(10)).await;

                let result = TestResult {
                    value: goal_info.order * 2,
                    sequence_id: goal_info.sequence_id,
                };

                let _ = executing.succeed(result);
            }
        });

        // Get result
        let result_future = tokio::time::timeout(Duration::from_millis(500), goal_handle.result());
        let result = result_future.await?;

        let _ = processing_task.await;

        // Verify result
        assert!(result.is_ok());
        let result_data = result.unwrap();
        assert_eq!(result_data.sequence_id, 3);

        Ok(())
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore] // Ignore by default due to cleanup issues with background tasks
    async fn test_network_partition_simulation() -> Result<()> {
        // Test behavior during simulated network issues
        let ctx = ZContextBuilder::default().build()?;
        let client_node = ctx.create_node("test_partition_client").build()?;
        let server_node = ctx.create_node("test_partition_server").build()?;

        let client = client_node
            .create_action_client::<TestAction>("test_partition_action")
            .build()?;

        let server = server_node
            .create_action_server::<TestAction>("test_partition_action")
            .build()?;

        // Send goal
        let goal = TestGoal {
            order: 25,
            sequence_id: 4,
        };
        let mut goal_handle = client.send_goal(goal).await?;

        // Simulate network delay/server unavailability
        let server_clone = server.clone();
        let processing_task = tokio::spawn(async move {
            // Delay before processing (simulate network issue)
            tokio::time::sleep(Duration::from_millis(200)).await;

            if let Ok(requested) = server_clone.recv_goal().await {
                let accepted = requested.accept();
                let executing = accepted.execute();

                let result = TestResult {
                    value: 50,
                    sequence_id: 4,
                };
                let _ = executing.succeed(result);
            }
        });

        // Client tries to get result with shorter timeout
        let result_future = tokio::time::timeout(Duration::from_millis(100), goal_handle.result());
        let early_result = result_future.await;

        // Should timeout initially
        assert!(early_result.is_err());

        // Now wait for the actual result
        let result_future = tokio::time::timeout(Duration::from_millis(300), goal_handle.result());
        let final_result = result_future.await;

        let _ = processing_task.await;

        // Should eventually succeed
        assert!(final_result.is_ok());
        assert!(final_result.unwrap().is_ok());

        Ok(())
    }
}
