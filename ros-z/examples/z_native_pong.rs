#![allow(unused)]

use cdr::{CdrLe, Infinite};
use ros_z::ros_msg::ByteMultiArray;
use zenoh::{Result, Wait};

fn main() -> Result<()> {
    zenoh::init_log_from_env_or("error");
    let session = zenoh::open(zenoh::Config::default()).wait()?;
    let (tx, rx) = flume::bounded(10);
    let zsub = session
        .declare_subscriber("ping")
        .callback(move |sample| {
            // let msg = cdr::deserialize::<ByteMultiArray>(&sample.payload().to_bytes()).unwrap();
            let msg = sample;
            tx.send(msg).unwrap();
        })
        .wait()?;
    let zpub = session.declare_publisher("pong").wait()?;

    loop {
        let msg = rx.recv()?;
        // let bytes = cdr::serialize::<_, _, CdrLe>(&msg, Infinite).unwrap();
        let bytes = msg.payload().to_bytes();
        zpub.put(&bytes).wait()?;
    }
}
