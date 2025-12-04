// ros-z/examples/action_concurrent_goals.rs
use std::time::Duration;

use ros_z::{
    Builder, Result,
    action::{ZAction, server::ExecutingGoal},
    context::ZContextBuilder,
};
use serde::{Deserialize, Serialize};

// Define ConcurrentGoals action messages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConcurrentGoal {
    pub task_id: i32,
    pub duration_ms: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConcurrentResult {
    pub task_id: i32,
    pub completed_at: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConcurrentFeedback {
    pub task_id: i32,
    pub progress: f32,
}

// Define the action type
pub struct ConcurrentAction;

impl ZAction for ConcurrentAction {
    type Goal = ConcurrentGoal;
    type Result = ConcurrentResult;
    type Feedback = ConcurrentFeedback;

    fn name() -> &'static str {
        "concurrent_action"
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();

    match args.get(1).map(String::as_str) {
        Some("server") => run_server().await,
        Some("client") => run_client().await,
        _ => {
            eprintln!("Usage: {} [server|client]", args[0]);
            std::process::exit(1);
        }
    }
}

async fn run_server() -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("concurrent_server").build()?;

    let _server = node
        .create_action_server::<ConcurrentAction>("concurrent_action")
        .build()?
        .with_handler(|executing: ExecutingGoal<ConcurrentAction>| async move {
            let task_id = executing.goal.task_id;
            let duration_ms = executing.goal.duration_ms;
            let start_time = std::time::Instant::now();

            println!("Starting task {} for {} ms", task_id, duration_ms);

            let total_steps = 10;
            for step in 0..=total_steps {
                if executing.is_cancel_requested() {
                    println!("Task {} canceled", task_id);
                    executing
                        .canceled(ConcurrentResult {
                            task_id,
                            completed_at: start_time.elapsed().as_millis() as i64,
                        })
                        .unwrap();
                    return;
                }

                // Publish feedback
                executing
                    .publish_feedback(ConcurrentFeedback {
                        task_id,
                        progress: step as f32 / total_steps as f32,
                    })
                    .expect("Failed to publish feedback");

                tokio::time::sleep(Duration::from_millis(
                    duration_ms as u64 / total_steps as u64,
                ))
                .await;
            }

            let completed_at = start_time.elapsed().as_millis() as i64;
            println!("Task {} completed in {} ms", task_id, completed_at);
            executing
                .succeed(ConcurrentResult {
                    task_id,
                    completed_at,
                })
                .unwrap();
        });

    println!("Concurrent action server started");
    tokio::signal::ctrl_c().await?;
    Ok(())
}

async fn run_client() -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("concurrent_client").build()?;

    let client = node
        .create_action_client::<ConcurrentAction>("concurrent_action")
        .build()?;

    // Send multiple goals concurrently
    let goals = vec![
        ConcurrentGoal {
            task_id: 1,
            duration_ms: 2000,
        },
        ConcurrentGoal {
            task_id: 2,
            duration_ms: 3000,
        },
        ConcurrentGoal {
            task_id: 3,
            duration_ms: 1500,
        },
    ];

    let mut handles = Vec::new();

    for goal in goals {
        let client_clone = client.clone();
        let handle = tokio::spawn(async move {
            println!(
                "Sending goal: task_id={}, duration_ms={}",
                goal.task_id, goal.duration_ms
            );
            let mut goal_handle = client_clone.send_goal(goal.clone()).await.unwrap();

            // Monitor feedback
            if let Some(mut feedback_stream) = goal_handle.feedback_stream() {
                tokio::spawn(async move {
                    while let Some(fb) = feedback_stream.recv().await {
                        println!("Task {} progress: {:.1}%", fb.task_id, fb.progress * 100.0);
                    }
                });
            }

            let result = goal_handle.result().await.unwrap();
            println!(
                "Task {} result: completed_at={} ms",
                result.task_id, result.completed_at
            );
        });
        handles.push(handle);
    }

    // Wait for all goals to complete
    for handle in handles {
        let _ = handle.await;
    }

    Ok(())
}
