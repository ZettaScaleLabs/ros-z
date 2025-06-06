use std::{
    fs::File,
    path::PathBuf,
    time::{SystemTime, UNIX_EPOCH},
};

use csv::Writer;
use zenoh::{Wait, Result};

fn get_percentile(data: &[u64], percentile: f64) -> u64 {
    if data.is_empty() {
        return 0;
    }
    let idx = ((percentile * data.len() as f64).round() as usize).min(data.len() - 1);
    data[idx]
}

fn print_statistics(mut data: Vec<u64>) {
    data.sort();
    println!("\nLatency stats (nanoseconds):");
    println!("Min : {}", data.first().unwrap());
    println!("p05 : {}", get_percentile(&data, 0.05));
    println!("p50 : {}", get_percentile(&data, 0.50));
    println!("p95 : {}", get_percentile(&data, 0.95));
    println!("Max : {}", data.last().unwrap());
}

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
    log: String,
    #[arg(short, long, default_value = "")]
    config: String,
}

#[derive(Debug)]
struct DataLogger {
    payload: usize,
    frequency: usize,
    path: PathBuf,
}

impl DataLogger {
    fn write(&self, data: Vec<u64>) -> Result<()> {
        let file = File::create(&self.path)?;
        let mut wtr = Writer::from_writer(file);
        wtr.write_record(
            ["Frequency", "Payload", "RTT"]
                .iter()
                .map(|x| x.to_string()),
        )?;

        for val in data {
            wtr.write_record(
                [self.frequency, self.payload, val as _]
                    .iter()
                    .map(|x| x.to_string()),
            )?;
            wtr.flush()?;
        }
        Ok(())
    }
}

fn main() -> Result<()> {
    let args = Args::parse();

    println!(
        "Freq: {} Hz, Payload: {} bytes, Samples: {}",
        &args.frequency, &args.payload, &args.sample
    );

    let config = if args.config.is_empty() {
        zenoh::Config::default()
    } else {
        zenoh::Config::from_file(args.config)?
    };
    let session = zenoh::open(config).wait()?;

    let zsub = session.declare_subscriber("ping").wait()?;

    let mut rtts = Vec::with_capacity(args.sample);
    while let Ok(sample) = zsub.recv() {
        let msg = sample.payload().to_bytes();
        let sent_time = u64::from_le_bytes(msg[0..8].try_into().unwrap());
        let now_ns = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("Time went backwards")
            .as_nanos() as u64;
        let rtt = now_ns.saturating_sub(sent_time);
        rtts.push(rtt);
        if rtts.len() >= args.sample {
            break
        }
    }
    print_statistics(rtts.clone());

    if !args.log.is_empty() {
        let logger = DataLogger {
            frequency: args.frequency,
            payload: args.payload,
            path: PathBuf::from(args.log),
        };

        logger.write(rtts)?;
    }

    Ok(())
}
