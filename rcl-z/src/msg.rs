use crate::type_support::TypeSupport;
use ros_z::msg::{ZDeserializer, ZMessage, ZSerializer};

pub struct RosMessage {
    msg: *const crate::c_void,
    ts: TypeSupport,
}

impl RosMessage {
    pub fn new(msg: *const crate::c_void, ts: TypeSupport) -> Self {
        RosMessage { msg, ts }
    }
}

pub struct RosSerdes;

impl ZDeserializer for RosSerdes {
    type Input<'a> = (&'a Vec<u8>, &'a mut RosMessage);
    type Output = bool;
    fn deserialize<'a>(input: Self::Input<'a>) -> Self::Output {
        let (bytes, msg) = input;
        msg.ts.deserialize_message(&bytes, msg.msg as *mut _);
        true
    }
}

impl ZSerializer for RosSerdes {
    type Input<'a> = &'a RosMessage;
    fn serialize<'a>(input: &'a RosMessage) -> Vec<u8> {
        let RosMessage { msg, ts } = input;
        let mut bytes = vec![0; ts.get_serialized_size(*msg)];
        ts.serialize_message(*msg, &mut bytes);
        bytes
    }
}

impl ZMessage for RosMessage {
    type Serdes = RosSerdes;
}

unsafe impl Sync for RosMessage {}
unsafe impl Send for RosMessage {}
