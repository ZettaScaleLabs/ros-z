use clap::Parser;
use ros_z::{Builder, Result, context::ZContextBuilder};
use ros_z_msgs::example_interfaces::{AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse};

#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "server", help = "Mode: server or client")]
    mode: String,

    #[arg(short, long, default_value = "1", help = "First number (client mode)")]
    a: i64,

    #[arg(short, long, default_value = "2", help = "Second number (client mode)")]
    b: i64,
}

fn main() -> Result<()> {
    let args = Args::parse();

    match args.mode.as_str() {
        "server" => run_server(),
        "client" => run_client(args.a, args.b),
        _ => {
            eprintln!("Invalid mode: {}. Use 'server' or 'client'", args.mode);
            std::process::exit(1);
        }
    }
}

fn run_server() -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("add_two_ints_server").build()?;
    let mut zsrv = node
        .create_service::<AddTwoInts>("add_two_ints")
        .build()?;

    println!("AddTwoInts service server started, waiting for requests...");

    loop {
        let (key, req) = zsrv.take_request()?;
        println!("Received request: {} + {}", req.a, req.b);

        let resp = AddTwoIntsResponse { sum: req.a + req.b };

        println!("Sending response: {}", resp.sum);
        zsrv.send_response(&resp, &key)?;
    }
}

fn run_client(a: i64, b: i64) -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("add_two_ints_client").build()?;
    let zcli = node
        .create_client::<AddTwoInts>("add_two_ints")
        .build()?;

    println!("AddTwoInts service client started");

    let req = AddTwoIntsRequest { a, b };
    println!("Sending request: {} + {}", req.a, req.b);

    zcli.send_request(&req)?;
    let resp = zcli.take_response()?;

    println!("Received response: {}", resp.sum);

    Ok(())
}
