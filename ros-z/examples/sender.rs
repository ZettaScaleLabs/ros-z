use std::time::{Duration, SystemTime, UNIX_EPOCH};

use zenoh::{Result, Wait, qos::CongestionControl};

use clap::Parser;
#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "64")]
    payload: usize,
    #[arg(short, long, default_value = "10")]
    frequency: usize,
    #[arg(short, long, default_value = "100")]
    sample: usize,
    #[arg(short, long, default_value = "")]
    config: String,
    #[arg(long)]
    express: bool,
    #[arg(long)]
    cc_block: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let config = if args.config.is_empty() {
        zenoh::Config::default()
    } else {
        zenoh::Config::from_file(args.config)?
    };
    let session = zenoh::open(config).wait()?;

    let zpub = session
        .declare_publisher("ping")
        .congestion_control(if args.cc_block {
            CongestionControl::Block
        } else {
            CongestionControl::Drop
        })
        .express(args.express)
        .wait()?;

    let period = if args.frequency == 0 {
        None
    } else {
        Some(Duration::from_secs_f64(1.0 / args.frequency as f64))
    };

    println!(
        "Freq: {} Hz, Payload: {} bytes, Samples: {}",
        &args.frequency, &args.payload, &args.sample
    );

    let mut msg = vec![0xAA; args.payload];
    for _ in 0..args.sample {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("Time went backwards")
            .as_nanos() as u64;
        msg[0..8].copy_from_slice(&now.to_le_bytes());
        zpub.put(&msg).wait()?;

        if let Some(ref period) = period {
            std::thread::sleep(*period);
        }
    }
    println!("Finished");
    Ok(())
}
