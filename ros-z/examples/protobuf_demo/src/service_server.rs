use ros_z::{Builder, Result, context::ZContext};

use crate::types::{Calculate, CalculateResponse};

/// Run the calculator service server
///
/// # Arguments
/// * `ctx` - The ROS-Z context
/// * `service_name` - Name of the service
/// * `max_requests` - Optional maximum number of requests to handle. If None, handles indefinitely.
pub fn run_service_server(
    ctx: ZContext,
    service_name: &str,
    max_requests: Option<usize>,
) -> Result<()> {
    let node = ctx.create_node("calculator_server").build()?;

    let mut server = node.create_service::<Calculate>(service_name).build()?;

    println!("Calculator service server started on '{}'", service_name);
    println!("Waiting for requests...\n");

    let mut count = 0;
    loop {
        match server.take_request() {
            Ok((key, request)) => {
                count += 1;
                println!(
                    "[Server] Request {}: {} {} {}",
                    count, request.a, request.operation, request.b
                );

                let response = match request.operation.as_str() {
                    "add" => CalculateResponse {
                        success: true,
                        result: request.a + request.b,
                        message: format!("{} + {} = {}", request.a, request.b, request.a + request.b),
                    },
                    "subtract" => CalculateResponse {
                        success: true,
                        result: request.a - request.b,
                        message: format!("{} - {} = {}", request.a, request.b, request.a - request.b),
                    },
                    "multiply" => CalculateResponse {
                        success: true,
                        result: request.a * request.b,
                        message: format!("{} * {} = {}", request.a, request.b, request.a * request.b),
                    },
                    "divide" => {
                        if request.b == 0.0 {
                            CalculateResponse {
                                success: false,
                                result: 0.0,
                                message: "Error: Division by zero".to_string(),
                            }
                        } else {
                            CalculateResponse {
                                success: true,
                                result: request.a / request.b,
                                message: format!(
                                    "{} / {} = {}",
                                    request.a,
                                    request.b,
                                    request.a / request.b
                                ),
                            }
                        }
                    }
                    _ => CalculateResponse {
                        success: false,
                        result: 0.0,
                        message: format!("Error: Unknown operation '{}'", request.operation),
                    },
                };

                server.send_response(&response, &key)?;

                // Check if we've reached max_requests
                if let Some(max) = max_requests {
                    if count >= max {
                        println!("[Server] Processed {} requests, exiting", count);
                        break;
                    }
                }
            }
            Err(e) => {
                eprintln!("[Server] Error: {}", e);
                std::thread::sleep(std::time::Duration::from_millis(100));
            }
        }
    }

    Ok(())
}
