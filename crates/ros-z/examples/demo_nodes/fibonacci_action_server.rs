use std::time::Duration;

use ros_z::{Builder, Result, action::server::ExecutingGoal, context::ZContext};
use ros_z_msgs::action_tutorials_interfaces::{
    FibonacciFeedback, FibonacciResult, action::Fibonacci,
};

// ANCHOR: full_example
/// Fibonacci action server node that computes Fibonacci sequences
///
/// # Arguments
/// * `ctx` - The ROS-Z context
/// * `timeout` - Optional timeout duration. If None, runs until ctrl+c.
pub async fn run_fibonacci_action_server(ctx: ZContext, timeout: Option<Duration>) -> Result<()> {
    // ANCHOR: node_setup
    // Create a node named "fibonacci_action_server"
    let node = ctx.create_node("fibonacci_action_server").build()?;
    // ANCHOR_END: node_setup

    // ANCHOR: action_server_setup
    // Create an action server
    // Note: The server variable must be kept alive for the duration of the function
    // to ensure the action server and its background tasks remain active
    let _server = node
        .create_action_server::<Fibonacci>("fibonacci")
        .build()?
        .with_handler(|executing: ExecutingGoal<Fibonacci>| async move {
            let order = executing.goal.order;
            let mut sequence = vec![0, 1];

            println!("Executing Fibonacci goal with order {}", order);

            let mut canceled = false;
            let mut cancel_sequence = None;

            // ANCHOR: execution_loop
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
            // ANCHOR_END: execution_loop

            // ANCHOR: completion
            if canceled {
                executing
                    .canceled(FibonacciResult {
                        sequence: cancel_sequence.unwrap(),
                    })
                    .unwrap();
            } else {
                println!("Goal succeeded!");
                executing.succeed(FibonacciResult { sequence }).unwrap();
            }
            // ANCHOR_END: completion
        });
    // ANCHOR_END: action_server_setup

    println!("Fibonacci action server started");

    if let Some(timeout) = timeout {
        // For testing: run for the specified timeout
        tokio::time::sleep(timeout).await;
    } else {
        tokio::signal::ctrl_c().await?;
    }

    Ok(())
}
// ANCHOR_END: full_example

// Only compile main when building as a binary (not when included as a module)
#[cfg(not(any(test, doctest)))]
fn main() -> Result<()> {
    use ros_z::context::ZContextBuilder;

    let args: Vec<String> = std::env::args().collect();

    // Initialize logging
    zenoh::init_log_from_env_or("error");

    // Create the ROS-Z context with optional configuration
    let mut builder = ZContextBuilder::default();
    if let Some(e) = args.get(1) {
        builder = builder.with_connect_endpoints([e]);
    } else {
        // Connect to local zenohd for testing
        builder = builder.with_connect_endpoints(["tcp/127.0.0.1:7447"]);
    }
    let ctx = builder.build()?;

    // Run the server
    tokio::runtime::Runtime::new()
        .unwrap()
        .block_on(run_fibonacci_action_server(ctx, None))
}
