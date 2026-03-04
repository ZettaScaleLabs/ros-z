//! Lifecycle-managed talker node.
//!
//! Demonstrates the ROS 2 lifecycle state machine:
//! - Registers `on_configure` / `on_activate` / `on_deactivate` callbacks
//! - Creates a lifecycle-gated publisher for `std_msgs/msg/String`
//! - Drives the node through its full lifecycle programmatically
//!
//! Run with:
//! ```bash
//! cargo run --example lifecycle_talker --features jazzy
//! ```

// ANCHOR: full_example
use ros_z::{Builder, Result, context::ZContextBuilder, lifecycle::CallbackReturn};
use ros_z_msgs::std_msgs::String as RosString;

fn main() -> Result<()> {
    // ANCHOR: node_setup
    let ctx = ZContextBuilder::default().build()?;
    let mut node = ctx.create_lifecycle_node("lifecycle_talker").build()?;
    // ANCHOR_END: node_setup

    // ANCHOR: callbacks
    node.on_configure = Box::new(|_prev| {
        println!("[configure] loading parameters");
        CallbackReturn::Success
    });

    node.on_activate = Box::new(|_prev| {
        println!("[activate] publisher enabled");
        CallbackReturn::Success
    });

    node.on_deactivate = Box::new(|_prev| {
        println!("[deactivate] publisher paused");
        CallbackReturn::Success
    });

    node.on_cleanup = Box::new(|_prev| {
        println!("[cleanup] releasing resources");
        CallbackReturn::Success
    });
    // ANCHOR_END: callbacks

    // ANCHOR: publisher
    // Publisher starts deactivated — publish() silently drops messages until
    // the node is activated.
    let pub_ = node.create_publisher::<RosString>("chatter")?;
    // ANCHOR_END: publisher

    // ANCHOR: lifecycle
    // Drive the node through its lifecycle
    node.configure()?;

    // Still inactive — this message is dropped silently
    pub_.publish(&RosString {
        data: "dropped (inactive)".to_string(),
    })?;

    node.activate()?;

    // Active — messages are delivered
    for i in 0..5 {
        let msg = RosString {
            data: format!("hello {i}"),
        };
        pub_.publish(&msg)?;
        println!("published: {}", msg.data);
    }

    node.deactivate()?;
    node.cleanup()?;
    node.shutdown()?;
    // ANCHOR_END: lifecycle

    println!("node finalized");
    Ok(())
}
// ANCHOR_END: full_example
