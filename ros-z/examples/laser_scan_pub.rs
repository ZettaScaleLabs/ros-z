use ros_z::{Result, Builder, context::ZContextBuilder, ros_msg::{LaserScan, Header, Time}};
use std::thread;
use std::time::Duration;

fn main() -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("laser_scan_publisher").build()?;
    let zpub = node
        .create_pub_with_info::<LaserScan>("scan")
        .build()?;

    println!("Publishing LaserScan messages on /scan...");

    let mut seq = 0u32;
    loop {
        // Simulate a 270-degree laser scanner with 540 points
        let angle_min = -135.0_f32.to_radians();
        let angle_max = 135.0_f32.to_radians();
        let num_readings = 540;
        let angle_increment = (angle_max - angle_min) / (num_readings as f32 - 1.0);

        let mut ranges = Vec::with_capacity(num_readings);
        let mut intensities = Vec::with_capacity(num_readings);

        // Generate simulated laser scan data
        for i in 0..num_readings {
            let angle = angle_min + (i as f32) * angle_increment;

            // Simulate a simple environment: closer ranges in front, farther on sides
            let base_range = 3.0 + 2.0 * angle.cos();

            // Add some variation
            let variation = 0.1 * ((seq as f32 * 0.1 + i as f32 * 0.05).sin());
            let range = base_range + variation;

            ranges.push(range);
            intensities.push(100.0 + 50.0 * (i as f32 / num_readings as f32));
        }

        let msg = LaserScan {
            header: Header {
                stamp: Time {
                    sec: (seq / 10) as i32,
                    nanosec: (seq % 10) * 100_000_000,
                },
                frame_id: "laser".to_string(),
            },
            angle_min,
            angle_max,
            angle_increment,
            time_increment: 0.0001,
            scan_time: 0.1,
            range_min: 0.1,
            range_max: 10.0,
            ranges,
            intensities,
        };

        zpub.publish(&msg)?;
        println!(
            "Published LaserScan #{}: {} ranges, angle [{:.2}, {:.2}] rad",
            seq, msg.ranges.len(), msg.angle_min, msg.angle_max
        );

        seq += 1;
        thread::sleep(Duration::from_millis(100));
    }
}
