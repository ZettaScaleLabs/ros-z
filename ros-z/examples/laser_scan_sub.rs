use ros_z::{Result, Builder, context::ZContextBuilder, ros_msg::LaserScan};

fn main() -> Result<()> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("laser_scan_subscriber").build()?;
    let zsub = node
        .create_sub_with_info::<LaserScan>("scan")
        .build()?;

    println!("Listening for LaserScan messages on /scan...");

    loop {
        let msg = zsub.recv()?;
        println!("Received LaserScan:");
        println!("  Frame: {}", msg.header.frame_id);
        println!("  Angle range: [{:.2}, {:.2}] rad", msg.angle_min, msg.angle_max);
        println!("  Angle increment: {:.4} rad", msg.angle_increment);
        println!("  Range: [{:.2}, {:.2}] m", msg.range_min, msg.range_max);
        println!("  Number of ranges: {}", msg.ranges.len());
        println!("  Scan time: {:.3} s", msg.scan_time);

        if !msg.ranges.is_empty() {
            let valid_ranges: Vec<f32> = msg.ranges
                .iter()
                .filter(|&&r| r >= msg.range_min && r <= msg.range_max)
                .copied()
                .collect();

            if !valid_ranges.is_empty() {
                let min = valid_ranges.iter().fold(f32::INFINITY, |a, &b| a.min(b));
                let max = valid_ranges.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
                println!("  Valid ranges: {} (min: {:.2}m, max: {:.2}m)", valid_ranges.len(), min, max);
            }
        }
        println!("---");
    }
}
