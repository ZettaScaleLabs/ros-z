use ros_z::{Builder, Result, context::ZContext};

// Humble uses action_tutorials_interfaces, Kilted+ uses example_interfaces
#[cfg(feature = "humble")]
use ros_z_msgs::action_tutorials_interfaces::{FibonacciGoal, action::Fibonacci};
#[cfg(not(feature = "humble"))]
use ros_z_msgs::example_interfaces::{FibonacciGoal, action::Fibonacci};

// ANCHOR: full_example
/// Fibonacci action client node that sends goals to compute Fibonacci sequences
///
/// # Arguments
/// * `ctx` - The ROS-Z context
/// * `order` - The order of the Fibonacci sequence to compute
pub async fn run_fibonacci_action_client(ctx: ZContext, order: i32) -> Result<Vec<i32>> {
    // ANCHOR: node_setup
    // Create a node named "fibonacci_action_client"
    let node = ctx.create_node("fibonacci_action_client").build()?;
    // ANCHOR_END: node_setup

    // ANCHOR: client_setup
    // Create an action client
    let client = node
        .create_action_client::<Fibonacci>("fibonacci")
        .build()?;

    // Wait a bit for the server to be discovered
    tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;

    println!(
        "Fibonacci action client started, sending goal with order {}",
        order
    );
    // ANCHOR_END: client_setup

    // ANCHOR: send_goal
    // Send the goal
    let mut goal_handle = client.send_goal(FibonacciGoal { order }).await?;
    println!("Goal sent and accepted!");
    // ANCHOR_END: send_goal

    // ANCHOR: feedback_monitoring
    // Set up feedback monitoring
    if let Some(mut feedback_stream) = goal_handle.feedback() {
        tokio::spawn(async move {
            while let Some(fb) = feedback_stream.recv().await {
                // action_tutorials_interfaces uses partial_sequence, example_interfaces uses sequence
                #[cfg(feature = "humble")]
                println!("Feedback: {:?}", fb.partial_sequence);
                #[cfg(not(feature = "humble"))]
                println!("Feedback: {:?}", fb.sequence);
            }
        });
    }
    // ANCHOR_END: feedback_monitoring

    // ANCHOR: wait_result
    // Wait for the result with timeout
    println!("Waiting for result (timeout: 10s)...");
    let result = match tokio::time::timeout(
        tokio::time::Duration::from_secs(10),
        goal_handle.result(),
    )
    .await
    {
        Ok(Ok(result)) => {
            println!("Final result: {:?}", result.sequence);
            result
        }
        Ok(Err(e)) => {
            eprintln!("Action failed: {}", e);
            return Err(e);
        }
        Err(_) => {
            eprintln!("Timeout waiting for action result");
            return Err(zenoh::Error::from("Timeout waiting for action result"));
        }
    };
    // ANCHOR_END: wait_result

    Ok(result.sequence)
}
// ANCHOR_END: full_example

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
