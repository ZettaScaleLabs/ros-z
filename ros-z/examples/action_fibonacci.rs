// ros-z/examples/action_fibonacci.rs
use std::sync::Arc;
use std::time::Duration;

use ros_z::{
    Builder, Result,
    action::{server::ExecutingGoal, ZAction},
    context::ZContextBuilder,
};
use serde::{Deserialize, Serialize};

// Define Fibonacci action messages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FibonacciGoal {
    pub order: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FibonacciResult {
    pub sequence: Vec<i32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FibonacciFeedback {
    pub partial_sequence: Vec<i32>,
}

// Define the action type
pub struct Fibonacci;

impl ZAction for Fibonacci {
    type Goal = FibonacciGoal;
    type Result = FibonacciResult;
    type Feedback = FibonacciFeedback;

    fn name() -> &'static str {
        "fibonacci"
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();

    match args.get(1).map(String::as_str) {
        Some("server") => run_server().await,
        Some("client") => {
            let order = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(10);
            run_client(order).await
        }
        _ => {
            eprintln!("Usage: {} [server|client] [order]", args[0]);
            std::process::exit(1);
        }
    }
}

async fn run_server() -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("fibonacci_server").build()?;

    let server = node
        .create_action_server::<Fibonacci>("fibonacci")
        .build()?;
    let _server = Arc::try_unwrap(server).ok().unwrap()        .with_handler(|executing: ExecutingGoal<Fibonacci>| async move {
            let order = executing.goal.order;
            let mut sequence = vec![0, 1];

            println!("Executing Fibonacci goal with order {}", order);

            let mut canceled = false;
            let mut cancel_sequence = None;

            for i in 2..=order {
                // Check for cancellation
                if executing.is_cancel_requested() {
                    println!("Goal canceled!");
                    canceled = true;
                    cancel_sequence = Some(sequence.clone());
                    break;
                }

                let next = sequence[i as usize - 1] + sequence[i as usize - 2];
                sequence.push(next);

                // Publish feedback
                executing
                    .publish_feedback(FibonacciFeedback {
                        partial_sequence: sequence.clone(),
                    })
                    .expect("Failed to publish feedback");

                tokio::time::sleep(Duration::from_millis(500)).await;
            }

            if canceled {
                executing.canceled(FibonacciResult {
                    sequence: cancel_sequence.unwrap(),
                }).unwrap();
            } else {
                println!("Goal succeeded!");
                executing.succeed(FibonacciResult { sequence }).unwrap();
            }
        });

    println!("Fibonacci action server started");
    tokio::signal::ctrl_c().await?;
    Ok(())
}

async fn run_client(order: i32) -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("fibonacci_client").build()?;

    let client = node
        .create_action_client::<Fibonacci>("fibonacci")
        .build()?;

    println!("Sending goal: order={}", order);
    let mut goal_handle = client.send_goal(FibonacciGoal { order }).await?;

    // Spawn task to monitor feedback
    if let Some(mut feedback_stream) = goal_handle.feedback_stream() {
        tokio::spawn(async move {
            while let Some(fb) = feedback_stream.recv().await {
                println!("Feedback: {:?}", fb.partial_sequence);
            }
        });
    }

    // Wait for result
    let result = goal_handle.result().await?;
    println!("Final result: {:?}", result.sequence);

    Ok(())
}
