use std::{
    fs::File,
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering::SeqCst},
    },
    time::{Duration, Instant},
};

use clap::Parser;
use csv::Writer;
use ros_z::{
    Builder, Result,
    context::ZContextBuilder,
    entity::{TypeHash, TypeInfo},
    ros_msg::ByteMultiArray,
};

#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "ping", help = "Mode: ping or pong")]
    mode: String,
    #[arg(short, long, default_value = "64", help = "Payload size in bytes")]
    payload: usize,
    #[arg(short, long, default_value = "10", help = "Frequency in Hz")]
    frequency: usize,
    #[arg(short, long, default_value = "100", help = "Number of samples")]
    sample: usize,
    #[arg(short, long, default_value = "", help = "Log file path")]
    log: String,
}

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
    println!("p25 : {}", get_percentile(&rtts, 0.25));
    println!("p50 : {}", get_percentile(&rtts, 0.50));
    println!("p75 : {}", get_percentile(&rtts, 0.75));
    println!("p95 : {}", get_percentile(&rtts, 0.95));
    println!("Max : {}", rtts.last().unwrap());
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

fn run_ping(args: &Args) -> Result<()> {
    let type_info = TypeInfo::new(
        "std_msgs::msg::dds_::UInt8MultiArray_",
        TypeHash::from_rihs_string(
            "RIHS01_5687e861b8d307a5e48b7515467ae7a5fc2daf805bd0ce6d8e9e604bade9f385",
        )
        .unwrap(),
    );
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("ping_node").build()?;
    let zpub = node
        .create_pub::<ByteMultiArray>("ping")
        .with_type_info(type_info.clone())
        .build()?;
    let zsub = node
        .create_sub::<ByteMultiArray>("pong")
        .with_type_info(type_info)
        .build()?;
    let period = Duration::from_secs_f64(1.0 / args.frequency as f64);
    let finished = Arc::new(AtomicBool::new(false));
    let c_finished = finished.clone();

    println!(
        "Freq: {} Hz, Payload: {} bytes, Samples: {}",
        &args.frequency, &args.payload, &args.sample
    );

    let logger = if args.log.is_empty() {
        None
    } else {
        Some(DataLogger {
            frequency: args.frequency,
            payload: args.payload,
            path: PathBuf::from(args.log.clone()),
        })
    };

    let start = Instant::now();
    let sample_count = args.sample;
    std::thread::spawn(move || {
        let mut rtts = Vec::with_capacity(sample_count);
        while rtts.len() < sample_count {
            if let Ok(msg) = zsub.recv() {
                let sent_time = u64::from_le_bytes(msg.data[0..8].try_into().unwrap());
                let rtt = start.elapsed().as_nanos() as u64 - sent_time;
                rtts.push(rtt);
            }
        }
        if let Some(logger) = logger {
            logger.write(rtts.clone()).expect("Failed to write the log");
        }
        print_statistics(rtts);
        c_finished.store(true, SeqCst);
    });

    let mut msg = ByteMultiArray {
        data: vec![0xAA; args.payload],
        ..Default::default()
    };
    while !finished.load(SeqCst) {
        let now = start.elapsed().as_nanos() as u64;
        msg.data[0..8].copy_from_slice(&now.to_le_bytes());
        zpub.publish(&msg)?;
        std::thread::sleep(period);
    }
    Ok(())
}

fn run_pong() -> Result<()> {
    let type_info = TypeInfo::new(
        "std_msgs::msg::dds_::UInt8MultiArray_",
        TypeHash::from_rihs_string(
            "RIHS01_5687e861b8d307a5e48b7515467ae7a5fc2daf805bd0ce6d8e9e604bade9f385",
        )
        .unwrap(),
    );
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("pong_node").build()?;
    let zsub = node
        .create_sub::<ByteMultiArray>("ping")
        .with_type_info(type_info.clone())
        .build()?;
    let zpub = node
        .create_pub::<ByteMultiArray>("pong")
        .with_type_info(type_info)
        .build()?;

    println!("Pong begin looping...");

    let mut message_count = 0u64;
    let mut last_print_time = Instant::now();
    let mut last_timestamp = 0u64;
    let mut last_payload_size = 0usize;

    loop {
        if let Ok(msg) = zsub.recv() {
            message_count += 1;

            last_timestamp = u64::from_le_bytes(msg.data[0..8].try_into().unwrap());
            last_payload_size = msg.data.len();

            zpub.publish(&msg)?;
        }

        let current_time = Instant::now();
        if current_time.duration_since(last_print_time) >= Duration::from_secs(2) {
            println!(
                "Pong status: received {} messages (last payload: {} bytes, last timestamp: {} ns)",
                message_count, last_payload_size, last_timestamp
            );
            last_print_time = current_time;
        }
    }
}

fn main() -> Result<()> {
    let args = Args::parse();

    if args.mode != "ping" && args.mode != "pong" {
        eprintln!("Invalid mode: {}. Must be 'ping' or 'pong'", args.mode);
        std::process::exit(1);
    }

    match args.mode.as_str() {
        "ping" => run_ping(&args),
        "pong" => run_pong(),
        _ => unreachable!(),
    }
}
