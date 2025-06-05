use zenoh::{Result, Wait, qos::CongestionControl};

use clap::Parser;
#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "")]
    config: String,
    #[arg(long)]
    callback: bool,
    #[arg(long)]
    express: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let config = if args.config.is_empty() {
        zenoh::Config::default()
    } else {
        zenoh::Config::from_file(args.config)?
    };
    let session = zenoh::open(config).wait()?;

    if args.callback {
        let zpub = session
            .declare_publisher("pong")
            .congestion_control(CongestionControl::Block)
            .express(args.express)
            .wait()?;
        let _zsub = session
            .declare_subscriber("ping")
            .callback(move |sample| {
                zpub.put(sample.payload().clone()).wait().unwrap();
            })
            .background()
            .wait()?;
        std::thread::park();
    } else {
        let zsub = session.declare_subscriber("ping").wait()?;
        let zpub = session
            .declare_publisher("pong")
            .congestion_control(CongestionControl::Block)
            .express(args.express)
            .wait()?;

        loop {
            let msg = zsub.recv()?;
            zpub.put(msg.payload().clone()).wait()?;
        }
    }
    Ok(())
}
