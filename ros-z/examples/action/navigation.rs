// ros-z/examples/action_navigation.rs
use std::time::Duration;

use ros_z::{
    Builder, Result,
    action::{ZAction, server::ExecutingGoal},
    context::ZContextBuilder,
};
use serde::{Deserialize, Serialize};

// Define navigation action messages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavigateToPoseGoal {
    pub target_x: f64,
    pub target_y: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavigateToPoseResult {
    pub success: bool,
    pub final_x: f64,
    pub final_y: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavigateToPoseFeedback {
    pub current_x: f64,
    pub current_y: f64,
    pub distance_remaining: f64,
}

// Define the action type
pub struct NavigateToPose;

impl ZAction for NavigateToPose {
    type Goal = NavigateToPoseGoal;
    type Result = NavigateToPoseResult;
    type Feedback = NavigateToPoseFeedback;

    fn name() -> &'static str {
        "navigate_to_pose"
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();

    match args.get(1).map(String::as_str) {
        Some("server") => run_server().await,
        Some("client") => {
            let target_x = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(10.0);
            let target_y = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(5.0);
            run_client(target_x, target_y).await
        }
        _ => {
            eprintln!("Usage: {} [server|client] [target_x] [target_y]", args[0]);
            eprintln!("Example: {} client 10.0 5.0", args[0]);
            std::process::exit(1);
        }
    }
}

async fn run_server() -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("navigation_server").build()?;

    let _server = node
        .create_action_server::<NavigateToPose>("navigate_to_pose")
        .build()?
        .with_handler(|executing: ExecutingGoal<NavigateToPose>| async move {
            let target_x = executing.goal.target_x;
            let target_y = executing.goal.target_y;

            println!("Navigating to pose: ({}, {})", target_x, target_y);

            // Start at origin
            let mut current_x = 0.0;
            let mut current_y = 0.0;
            let speed = 0.5; // units per second
            let update_interval = Duration::from_millis(200);

            let mut canceled = false;

            loop {
                // Check for cancellation
                if executing.is_cancel_requested() {
                    println!("Navigation canceled!");
                    canceled = true;
                    break;
                }

                // Calculate distance to target
                let dx = target_x - current_x;
                let dy = target_y - current_y;
                let distance = (dx * dx + dy * dy).sqrt();

                // Check if we've reached the target
                if distance < 0.1 {
                    println!("Reached target!");
                    break;
                }

                // Move towards target
                let move_distance = speed * (update_interval.as_millis() as f64 / 1000.0);
                if distance > 0.0 {
                    current_x += dx / distance * move_distance;
                    current_y += dy / distance * move_distance;
                }

                // Publish feedback
                executing
                    .publish_feedback(NavigateToPoseFeedback {
                        current_x,
                        current_y,
                        distance_remaining: distance,
                    })
                    .expect("Failed to publish feedback");

                tokio::time::sleep(update_interval).await;
            }

            let result = NavigateToPoseResult {
                success: !canceled,
                final_x: current_x,
                final_y: current_y,
            };

            if canceled {
                executing.canceled(result).unwrap();
            } else {
                println!("Navigation succeeded!");
                executing.succeed(result).unwrap();
            }
        });

    println!("Navigation action server started");
    tokio::signal::ctrl_c().await?;
    Ok(())
}

async fn run_client(target_x: f64, target_y: f64) -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("navigation_client").build()?;

    let client = node
        .create_action_client::<NavigateToPose>("navigate_to_pose")
        .build()?;

    println!("Sending navigation goal: ({}, {})", target_x, target_y);
    let mut goal_handle = client
        .send_goal(NavigateToPoseGoal { target_x, target_y })
        .await?;

    // Spawn task to monitor feedback
    if let Some(mut feedback_stream) = goal_handle.feedback_stream() {
        tokio::spawn(async move {
            while let Some(fb) = feedback_stream.recv().await {
                println!(
                    "Position: ({:.2}, {:.2}), Distance remaining: {:.2}",
                    fb.current_x, fb.current_y, fb.distance_remaining
                );
            }
        });
    }

    // Wait for result
    let result = goal_handle.result().await?;
    println!(
        "Navigation result: success={}, final position: ({:.2}, {:.2})",
        result.success, result.final_x, result.final_y
    );

    Ok(())
}
