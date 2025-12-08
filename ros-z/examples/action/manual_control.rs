// ros-z/examples/action_manual_control.rs
use std::time::Duration;

use ros_z::{Builder, Result, action::ZAction, context::ZContextBuilder};
use serde::{Deserialize, Serialize};

// Define ManualControl action messages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ManualGoal {
    pub target_value: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ManualResult {
    pub achieved_value: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ManualFeedback {
    pub current_value: i32,
}

// Define the action type
pub struct ManualAction;

impl ZAction for ManualAction {
    type Goal = ManualGoal;
    type Result = ManualResult;
    type Feedback = ManualFeedback;

    fn name() -> &'static str {
        "manual_action"
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();

    match args.get(1).map(String::as_str) {
        Some("server") => run_server().await,
        Some("client") => {
            let target = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(50);
            run_client(target).await
        }
        _ => {
            eprintln!("Usage: {} [server|client] [target_value]", args[0]);
            std::process::exit(1);
        }
    }
}

async fn run_server() -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("manual_server").build()?;

    let server = node
        .create_action_server::<ManualAction>("manual_action")
        .build()?;

    println!("Manual control action server started");

    loop {
        tokio::select! {
            // Handle incoming goals
            Ok(requested) = server.recv_goal() => {
                println!("Received goal: target_value={}", requested.goal.target_value);

                // Decide whether to accept or reject
                if requested.goal.target_value > 100 {
                    println!("Rejecting goal: target too high");
                    let _ = requested.reject();
                    continue;
                }

                println!("Accepting goal");
                let accepted = requested.accept();
                let executing = accepted.execute();

                // Simulate work
                tokio::spawn(async move {
                    let mut current = 0;
                    let target = executing.goal.target_value;

                    while current < target {
                        current += 1;

                        // Publish feedback
                        let _ = executing.publish_feedback(ManualFeedback { current_value: current });

                        tokio::time::sleep(Duration::from_millis(100)).await;

                        // Check for cancellation
                        if executing.is_cancel_requested() {
                            println!("Goal canceled at {}", current);
                            let _ = executing.canceled(ManualResult { achieved_value: current });
                            return;
                        }
                    }

                    println!("Goal succeeded with value {}", current);
                    let _ = executing.succeed(ManualResult { achieved_value: current });
                });
            }

            // Handle cancel requests
            Ok((cancel_request, _query)) = server.recv_cancel() => {
                println!("Received cancel request for goal {:?}", cancel_request.goal_info.goal_id);
                // The server automatically handles cancellation in the background
            }

            // Handle result requests
            Ok((_result_request, _query)) = server.recv_result_request() => {
                println!("Received result request");
                // Results are handled automatically by the server
            }

            _ = tokio::signal::ctrl_c() => {
                println!("Shutting down server");
                break;
            }
        }
    }

    Ok(())
}

async fn run_client(target_value: i32) -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("manual_client").build()?;

    let client = node
        .create_action_client::<ManualAction>("manual_action")
        .build()?;

    println!("Sending goal: target_value={}", target_value);
    let mut goal_handle = client.send_goal(ManualGoal { target_value }).await?;

    // Monitor feedback
    if let Some(mut feedback_stream) = goal_handle.feedback_stream() {
        tokio::spawn(async move {
            while let Some(fb) = feedback_stream.recv().await {
                println!("Feedback: current_value={}", fb.current_value);
            }
        });
    }

    // Wait for result
    let result = goal_handle.result().await?;
    println!("Final result: achieved_value={}", result.achieved_value);

    Ok(())
}
