use clap::{Parser, ValueEnum};
use ros_z::{
    Builder, Result,
    context::{ZContext, ZContextBuilder},
};
use ros_z_msgs::example_interfaces::{AddTwoIntsRequest, AddTwoIntsResponse, srv::AddTwoInts};

#[derive(Debug, Clone, Copy, ValueEnum)]
enum Backend {
    /// RmwZenoh backend (default) - compatible with rmw_zenoh nodes
    /// Uses key expressions with domain prefix
    RmwZenoh,
    /// Ros2Dds backend - compatible with zenoh-bridge-ros2dds
    /// Uses key expressions without domain prefix
    #[cfg(feature = "ros2dds")]
    Ros2Dds,
}

#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "server", help = "Mode: server or client")]
    mode: String,

    #[arg(short, long, default_value = "1", help = "First number (client mode)")]
    a: i64,

    #[arg(short, long, default_value = "2", help = "Second number (client mode)")]
    b: i64,

    /// Backend selection: rmw-zenoh (default) or ros2-dds
    #[arg(long, value_enum, default_value = "rmw-zenoh")]
    backend: Backend,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();

    // Convert backend enum to KeyExprFormat
    let format = match args.backend {
        Backend::RmwZenoh => ros_z_protocol::KeyExprFormat::RmwZenoh,
        #[cfg(feature = "ros2dds")]
        Backend::Ros2Dds => ros_z_protocol::KeyExprFormat::Ros2Dds,
    };

    let ctx = ZContextBuilder::default().keyexpr_format(format).build()?;

    match args.mode.as_str() {
        "server" => run_server(ctx),
        "client" => run_client(ctx, args.a, args.b).await,
        mode => {
            eprintln!("Invalid mode: {}. Use 'server' or 'client'", mode);
            std::process::exit(1);
        }
    }
}

fn run_server(ctx: ZContext) -> Result<()> {
    let node = ctx.create_node("add_two_ints_server").build()?;
    let mut zsrv = node.create_service::<AddTwoInts>("add_two_ints").build()?;

    println!("AddTwoInts service server started, waiting for requests...");

    loop {
        let (key, req) = zsrv.take_request()?;
        println!("Received request: {} + {}", req.a, req.b);

        let resp = AddTwoIntsResponse { sum: req.a + req.b };

        println!("Sending response: {}", resp.sum);
        zsrv.send_response(&resp, &key)?;
    }
}

async fn run_client(ctx: ZContext, a: i64, b: i64) -> Result<()> {
    let node = ctx.create_node("add_two_ints_client").build()?;
    let zcli = node.create_client::<AddTwoInts>("add_two_ints").build()?;

    println!("AddTwoInts service client started");

    let req = AddTwoIntsRequest { a, b };
    println!("Sending request: {} + {}", req.a, req.b);

    zcli.send_request(&req).await?;
    let resp = zcli.take_response()?;

    println!("Received response: {}", resp.sum);

    Ok(())
}
