use std::time::Duration;

use ros_z::{Builder, Result, context::ZContext};

use crate::types::{Calculate, CalculateRequest};

/// Run the calculator service client
///
/// # Arguments
/// * `ctx` - The ROS-Z context
/// * `service_name` - Name of the service to call
/// * `operations` - List of (operation, a, b) tuples to execute
pub fn run_service_client(
    ctx: ZContext,
    service_name: &str,
    operations: Vec<(&str, f64, f64)>,
) -> Result<()> {
    let node = ctx.create_node("calculator_client").build()?;

    let client = node.create_client::<Calculate>(service_name).build()?;

    println!("Calculator service client started");
    println!("Calling service '{}'...\n", service_name);

    // Give server time to start if needed
    std::thread::sleep(Duration::from_millis(500));

    for (op, a, b) in operations {
        println!("[Client] Sending: {} {} {}", a, op, b);

        let request = CalculateRequest {
            a,
            b,
            operation: op.to_string(),
        };

        // Send request and wait for response
        let response = tokio::runtime::Runtime::new()
            .unwrap()
            .block_on(async {
                client.send_request(&request).await?;
                client.take_response_timeout(Duration::from_secs(5))
            })?;

        if response.success {
            println!("[Client] ✓ Success: {}", response.message);
        } else {
            println!("[Client] ✗ Failed: {}", response.message);
        }
        println!();

        std::thread::sleep(Duration::from_millis(200));
    }

    println!("Successfully demonstrated protobuf service calls!");
    println!("\nKey points:");
    println!("1. Request/Response: Protobuf messages generated from .proto files");
    println!("2. Both implement MessageTypeInfo, WithTypeInfo, and ZMessage traits");
    println!("3. Service type implements ZService and ServiceTypeInfo");
    println!("4. Works seamlessly with node.create_service() and node.create_client()");
    println!("5. Uses ProtobufSerdes for pure protobuf serialization (not CDR!)");

    Ok(())
}
