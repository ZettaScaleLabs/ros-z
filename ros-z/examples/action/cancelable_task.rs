// ros-z/examples/action_cancelable_task.rs
use std::time::Duration;

use ros_z::{
    Builder, Result,
    action::{ZAction, server::ExecutingGoal},
    context::ZContextBuilder,
};
use serde::{Deserialize, Serialize};

// Define CancelableTask action messages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CancelableTaskGoal {
    pub target_count: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CancelableTaskResult {
    pub final_count: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CancelableTaskFeedback {
    pub current_count: i32,
}

// Define the action type
pub struct CancelableTask;

impl ZAction for CancelableTask {
    type Goal = CancelableTaskGoal;
    type Result = CancelableTaskResult;
    type Feedback = CancelableTaskFeedback;

    fn name() -> &'static str {
        "cancelable_task"
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();

    match args.get(1).map(String::as_str) {
        Some("server") => run_server().await,
        Some("client") => {
            let target = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(20);
            run_client(target).await
        }
        _ => {
            eprintln!("Usage: {} [server|client] [target_count]", args[0]);
            std::process::exit(1);
        }
    }
}

async fn run_server() -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("cancelable_task_server").build()?;

    let _server = node
        .create_action_server::<CancelableTask>("cancelable_task")
        .build()?
        .with_handler(|executing: ExecutingGoal<CancelableTask>| async move {
            let target = executing.goal.target_count;
            let mut count = 0;

            println!("Starting cancelable task to count to {}", target);

            while count < target {
                // Check for cancellation
                if executing.is_cancel_requested() {
                    println!("Task canceled at count {}", count);
                    executing
                        .canceled(CancelableTaskResult { final_count: count })
                        .unwrap();
                    return;
                }

                count += 1;

                // Publish feedback
                executing
                    .publish_feedback(CancelableTaskFeedback {
                        current_count: count,
                    })
                    .expect("Failed to publish feedback");

                tokio::time::sleep(Duration::from_millis(200)).await;
            }

            println!("Task completed successfully!");
            executing
                .succeed(CancelableTaskResult { final_count: count })
                .unwrap();
        });

    println!("CancelableTask action server started");
    tokio::signal::ctrl_c().await?;
    Ok(())
}

async fn run_client(target_count: i32) -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("cancelable_task_client").build()?;

    let client = node
        .create_action_client::<CancelableTask>("cancelable_task")
        .build()?;

    println!("Sending goal: target_count={}", target_count);
    let mut goal_handle = client
        .send_goal(CancelableTaskGoal { target_count })
        .await?;

    // Spawn task to monitor feedback
    if let Some(mut feedback_stream) = goal_handle.feedback_stream() {
        tokio::spawn(async move {
            while let Some(fb) = feedback_stream.recv().await {
                println!("Feedback: current_count={}", fb.current_count);
            }
        });
    }

    // Cancel after 2 seconds
    tokio::time::sleep(Duration::from_secs(2)).await;
    println!("Canceling goal...");
    let _cancel_response = goal_handle.cancel().await?;

    // Wait for result
    let result = goal_handle.result().await?;
    println!("Final result: final_count={}", result.final_count);

    Ok(())
}
