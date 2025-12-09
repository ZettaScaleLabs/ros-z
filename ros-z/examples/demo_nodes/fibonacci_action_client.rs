use ros_z::{
    Builder, Result,
    action::ZAction,
    context::ZContext,
};
use ros_z_msgs::example_interfaces::{FibonacciFeedback, FibonacciGoal, FibonacciResult};

// Define the Fibonacci action using standard ROS2 messages
pub struct FibonacciAction;

impl ZAction for FibonacciAction {
    type Goal = FibonacciGoal;
    type Result = FibonacciResult;
    type Feedback = FibonacciFeedback;

    fn name() -> &'static str {
        "fibonacci"
    }
}

/// Fibonacci action client node that sends goals to compute Fibonacci sequences
///
/// # Arguments
/// * `ctx` - The ROS-Z context
/// * `order` - The order of the Fibonacci sequence to compute
pub async fn run_fibonacci_action_client(ctx: ZContext, order: i32) -> Result<Vec<i32>> {
    // Create a node named "fibonacci_action_client"
    let node = ctx.create_node("fibonacci_action_client").build()?;

    // Create an action client
    let client = node.create_action_client::<FibonacciAction>("fibonacci").build()?;

    println!("Fibonacci action client started, sending goal with order {}", order);

    // Send the goal
    let mut goal_handle = client.send_goal(FibonacciGoal { order }).await?;

    // Set up feedback monitoring
    if let Some(mut feedback_stream) = goal_handle.feedback_stream() {
        tokio::spawn(async move {
            while let Some(fb) = feedback_stream.recv().await {
                println!("Feedback: {:?}", fb.sequence);
            }
        });
    }

    // Wait for the result
    let result = goal_handle.result().await?;
    println!("Final result: {:?}", result.sequence);

    Ok(result.sequence)
}

// Only compile main when building as a binary (not when included as a module)
#[cfg(not(any(test, doctest)))]
fn main() -> Result<()> {
    use clap::Parser;
    use ros_z::context::ZContextBuilder;

    let args = Args::parse();

    // Initialize logging
    zenoh::init_log_from_env_or("error");

    // Create the ROS-Z context with optional configuration
    let mut builder = ZContextBuilder::default();
    if let Some(e) = args.endpoint {
        builder = builder.with_connect_endpoints([e]);
    } else {
        // Connect to local zenohd for testing
        builder = builder.with_connect_endpoints(["tcp/127.0.0.1:7447"]);
    }
    let ctx = builder.build()?;

    // Run the client
    let result = tokio::runtime::Runtime::new()
        .unwrap()
        .block_on(run_fibonacci_action_client(ctx, args.order))?;

    println!("Action completed with sequence: {:?}", result);

    Ok(())
}

#[cfg(not(any(test, doctest)))]
#[derive(Debug, clap::Parser)]
#[command(
    name = "demo_nodes_fibonacci_action_client",
    about = "ROS 2 demo fibonacci action client node - sends goals to compute Fibonacci sequences"
)]
struct Args {
    /// Order of the Fibonacci sequence to compute
    #[arg(short, long, default_value = "10")]
    order: i32,

    /// Zenoh router endpoint to connect to (e.g., tcp/localhost:7447)
    #[arg(short, long)]
    endpoint: Option<String>,
}
