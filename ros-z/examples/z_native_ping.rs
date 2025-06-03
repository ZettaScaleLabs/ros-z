#![allow(unused)]

use std::sync::atomic::Ordering::SeqCst;
use std::sync::{Arc, atomic::AtomicBool};
use std::time::{Duration, Instant};
use cdr::{CdrLe, Infinite};
use ros_z::ros_msg::ByteMultiArray;
use zenoh::{Result, Wait};

fn get_percentile(data: &Vec<u128>, percentile: f64) -> u128 {
    if data.is_empty() {
        return 0;
    }
    let idx = ((percentile * data.len() as f64).round() as usize).min(data.len() - 1);
    data[idx]
}

fn print_statistics(mut rtts: Vec<u128>) {
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
    zenoh::init_log_from_env_or("error");
    let args = Args::parse();
    let session = zenoh::open(zenoh::Config::default()).wait()?;
    let (tx, rx) = flume::bounded(10);
    let zsub = session
        .declare_subscriber("pong")
        .callback(move |sample| {
            // let msg = cdr::deserialize::<ByteMultiArray>(&sample.payload().to_bytes()).unwrap();
            // let _ = tx.try_send(msg);
            let _ = tx.try_send(sample);
        })
        .wait()?;
    let zpub = session.declare_publisher("ping").wait()?;

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
            if let Ok(msg) = rx.recv() {
                let sent_time = u128::from_le_bytes(msg.payload().to_bytes().to_vec()[0..16].try_into().unwrap());
                // let sent_time = u128::from_le_bytes(msg.data[0..16].try_into().unwrap());
                let rtt = start.elapsed().as_nanos() as u128 - sent_time;
                rtts.push(rtt);
            }
        }
        // println!("Total(secs) = {:.2}, len = {}", rtts.iter().sum::<u128>() as f64 / 1e9, rtts.len());
        print_statistics(rtts);
        c_finished.store(true, SeqCst);
    });

    // let mut idx = 0;
    while !finished.load(SeqCst) {
        // let mut msg = ByteMultiArray::default();
        // msg.data = vec![0xAA; args.payload];
        let mut msg = vec![0xAA; args.payload];
        msg[0..16].copy_from_slice(&(start.elapsed().as_nanos() as u128).to_le_bytes());
        // msg.data[0..8].copy_from_slice(&now.to_le_bytes());
        // msg.data[0..16].copy_from_slice(&(start.elapsed().as_nanos() as u128).to_le_bytes());
        // let bytes = cdr::serialize::<_, _, CdrLe>(&msg, Infinite).unwrap();
        // let now = Instant::now();
        zpub.put(&msg).wait()?;
        // zpub.put(&bytes).wait()?;

        // let deadline = start + idx * period;
        // std::thread::sleep(deadline - now);
        std::thread::sleep(period);
        // idx += 1;
    }
    Ok(())
}
