use cdr::{CdrLe, Infinite};
use ros_z_msgs::std_msgs::ByteMultiArray;

#[test]
fn test_bytes() {
    let msg = ByteMultiArray::default();
    cdr::serialize::<_, _, CdrLe>(&msg, Infinite).unwrap();
}
