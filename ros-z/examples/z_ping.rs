use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering::SeqCst},
    },
    time::{Duration, Instant},
};

use ros_z::{Result, context::ZContext, pubsub::Builder, ros_msg::ByteMultiArray};

fn get_percentile(data: &[u64], percentile: f64) -> u64 {
    if data.is_empty() {
        return 0;
    }
    let idx = ((percentile * data.len() as f64).round() as usize).min(data.len() - 1);
    data[idx]
}

fn print_statistics(mut rtts: Vec<u64>) {
    rtts.sort();
    println!("\nRTT stats (nanoseconds):");
    println!("Min : {}", rtts.first().unwrap());
    println!("p05 : {}", get_percentile(&rtts, 0.05));
    println!("p50 : {}", get_percentile(&rtts, 0.50));
    println!("p95 : {}", get_percentile(&rtts, 0.95));
    println!("Max : {}", rtts.last().unwrap());
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
}

fn main() -> Result<()> {
    let args = Args::parse();
    let ctx = ZContext::new()?;
    let node = ctx.create_node();
    let zpub = node.create_pub::<ByteMultiArray>("ping").build()?;
    let zsub = node.create_sub::<ByteMultiArray>("pong").build()?;
    let period = Duration::from_secs_f64(1.0 / args.frequency as f64);
    let finished = Arc::new(AtomicBool::new(false));
    let c_finished = finished.clone();

    println!(
        "Freq: {} Hz, Payload: {} bytes, Samples: {}",
        &args.frequency, &args.payload, &args.sample
    );

    let start = Instant::now();
    std::thread::spawn(move || {
        let mut rtts = Vec::with_capacity(args.sample);
        while rtts.len() < args.sample {
            let msg = zsub.recv().unwrap();
            let sent_time = u64::from_le_bytes(msg.data[0..8].try_into().unwrap());
            let rtt = start.elapsed().as_nanos() as u64 - sent_time;
            rtts.push(rtt);
        }
        print_statistics(rtts);
        c_finished.store(true, SeqCst);
    });

    while !finished.load(SeqCst) {
        let mut msg = ByteMultiArray {
            data: vec![0xAA; args.payload],
            ..Default::default()
        };
        let now = start.elapsed().as_nanos() as u64;
        msg.data[0..8].copy_from_slice(&now.to_le_bytes());
        zpub.publish(&msg)?;
        std::thread::sleep(period);
    }
    Ok(())
}
