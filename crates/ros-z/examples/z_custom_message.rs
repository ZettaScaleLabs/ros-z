use std::time::Duration;

use clap::Parser;
use ros_z::{
    Builder, MessageTypeInfo, Result, ServiceTypeInfo, context::ZContextBuilder, entity::TypeHash,
    msg::ZService,
};
use serde::{Deserialize, Serialize};

// Custom message for pub/sub example
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct RobotStatus {
    pub robot_id: String,
    pub battery_percentage: f64,
    pub position_x: f64,
    pub position_y: f64,
    pub is_moving: bool,
}

impl MessageTypeInfo for RobotStatus {
    fn type_name() -> &'static str {
        "custom_msgs::msg::dds_::RobotStatus_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero()
    }
}

impl ros_z::WithTypeInfo for RobotStatus {}

// Custom service request
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct NavigateToRequest {
    pub target_x: f64,
    pub target_y: f64,
    pub max_speed: f64,
}

impl MessageTypeInfo for NavigateToRequest {
    fn type_name() -> &'static str {
        "custom_msgs::srv::dds_::NavigateTo_Request_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero()
    }
}

impl ros_z::WithTypeInfo for NavigateToRequest {}

// Custom service response
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct NavigateToResponse {
    pub success: bool,
    pub estimated_duration: f64,
    pub message: String,
}

impl MessageTypeInfo for NavigateToResponse {
    fn type_name() -> &'static str {
        "custom_msgs::srv::dds_::NavigateTo_Response_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero()
    }
}

impl ros_z::WithTypeInfo for NavigateToResponse {}

// Service type definition
pub struct NavigateTo;

impl ServiceTypeInfo for NavigateTo {
    fn service_type_info() -> ros_z::entity::TypeInfo {
        ros_z::entity::TypeInfo::new("custom_msgs::srv::dds_::NavigateTo_", TypeHash::zero())
    }
}

impl ZService for NavigateTo {
    type Request = NavigateToRequest;
    type Response = NavigateToResponse;
}

#[derive(Debug, Parser)]
struct Args {
    #[arg(
        short,
        long,
        default_value = "status-pub",
        help = "Mode: status-pub, status-sub, nav-server, or nav-client"
    )]
    mode: String,

    #[arg(
        long,
        default_value = "robot_1",
        help = "Robot ID (for status-pub mode)"
    )]
    robot_id: String,

    #[arg(long, default_value = "10.0", help = "Target X coordinate")]
    target_x: f64,

    #[arg(long, default_value = "20.0", help = "Target Y coordinate")]
    target_y: f64,

    #[arg(long, default_value = "1.5", help = "Maximum speed")]
    max_speed: f64,
}

#[tokio::main]
async fn main() -> Result<()> {
    match Args::parse().mode.as_str() {
        "status-pub" => run_status_publisher(Args::parse().robot_id).await,
        "status-sub" => run_status_subscriber().await,
        "nav-server" => run_navigation_server(),
        "nav-client" => {
            let args = Args::parse();
            run_navigation_client(args.target_x, args.target_y, args.max_speed)
        }
        mode => {
            eprintln!(
                "Invalid mode: {mode}. Use 'status-pub', 'status-sub', 'nav-server', or 'nav-client'"
            );
            std::process::exit(1);
        }
    }
}

async fn run_status_publisher(robot_id: String) -> Result<()> {
    println!("Starting robot status publisher for robot: {robot_id}");

    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("robot_status_publisher").build()?;
    let zpub = node.create_pub::<RobotStatus>("/robot_status").build()?;

    let mut position_x = 0.0;
    let mut position_y = 0.0;
    let mut battery = 100.0;
    let mut moving = true;

    loop {
        // Simulate robot movement and battery drain
        if moving {
            position_x += 0.5;
            position_y += 0.3;
            battery -= 0.1;

            if battery < 20.0 {
                moving = false;
                println!("Low battery! Robot stopped.");
            }
        }

        let status = RobotStatus {
            robot_id: robot_id.clone(),
            battery_percentage: battery,
            position_x,
            position_y,
            is_moving: moving,
        };

        println!(
            "Publishing status: pos=({:.1}, {:.1}), battery={:.1}%, moving={}",
            status.position_x, status.position_y, status.battery_percentage, status.is_moving
        );

        zpub.async_publish(&status).await?;

        tokio::time::sleep(Duration::from_secs(1)).await;

        // Reset simulation when battery too low
        if battery < 10.0 {
            battery = 100.0;
            moving = true;
            println!("Battery recharged! Resuming movement.");
        }
    }
}

async fn run_status_subscriber() -> Result<()> {
    println!("Starting robot status subscriber...");

    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("robot_status_subscriber").build()?;
    let zsub = node.create_sub::<RobotStatus>("/robot_status").build()?;

    loop {
        let status = zsub.async_recv().await?;
        println!(
            "Received status from {}: pos=({:.1}, {:.1}), battery={:.1}%, moving={}",
            status.robot_id,
            status.position_x,
            status.position_y,
            status.battery_percentage,
            status.is_moving
        );

        if status.battery_percentage < 20.0 {
            println!("WARNING: {} has low battery!", status.robot_id);
        }
    }
}

fn run_navigation_server() -> Result<()> {
    println!("Starting navigation service server...");

    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("navigation_server").build()?;
    let mut zsrv = node.create_service::<NavigateTo>("/navigate_to").build()?;

    println!("Navigation server ready, waiting for requests...");

    loop {
        if let Ok((request_id, request)) = zsrv.take_request() {
            println!(
                "Received navigation request: target=({:.1}, {:.1}), max_speed={:.1}",
                request.target_x, request.target_y, request.max_speed
            );

            // Simulate path planning
            std::thread::sleep(Duration::from_millis(500));

            let distance = (request.target_x.powi(2) + request.target_y.powi(2)).sqrt();
            let duration = distance / request.max_speed;

            let response = if request.max_speed > 0.0 && request.max_speed < 5.0 {
                NavigateToResponse {
                    success: true,
                    estimated_duration: duration,
                    message: format!(
                        "Path planned successfully. Distance: {:.2}m, ETA: {:.2}s",
                        distance, duration
                    ),
                }
            } else {
                NavigateToResponse {
                    success: false,
                    estimated_duration: 0.0,
                    message: "Invalid max_speed. Must be between 0 and 5 m/s.".to_string(),
                }
            };

            println!("Sending response: {:?}", response);
            zsrv.send_response(&response, &request_id)?;
        }

        std::thread::sleep(Duration::from_millis(100));
    }
}

fn run_navigation_client(target_x: f64, target_y: f64, max_speed: f64) -> Result<()> {
    println!("Starting navigation client...");

    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("navigation_client").build()?;
    let zcli = node.create_client::<NavigateTo>("/navigate_to").build()?;

    let request = NavigateToRequest {
        target_x,
        target_y,
        max_speed,
    };

    println!(
        "Sending navigation request: target=({:.1}, {:.1}), max_speed={:.1}",
        request.target_x, request.target_y, request.max_speed
    );

    tokio::runtime::Runtime::new()
        .unwrap()
        .block_on(async { zcli.send_request(&request).await })?;

    println!("Waiting for response...");

    loop {
        if let Ok(response) = zcli.take_response() {
            println!("Received response:");
            println!("Success: {}", response.success);
            println!("Duration: {:.2}s", response.estimated_duration);
            println!("Message: {}", response.message);
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }

    Ok(())
}
