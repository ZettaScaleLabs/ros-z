use crate::type_support::MessageTypeSupport;
use ros_z::entity::{TypeHash, TypeInfo};
use ros_z::msg::{ZDeserializer, ZMessage, ZSerializer, ZService};
use ros_z::ros_msg::WithTypeInfo;
use ros_z::{MessageTypeInfo, ServiceTypeInfo};

pub struct RosMessage {
    msg: *const crate::c_void,
    ts: MessageTypeSupport,
}

impl RosMessage {
    pub fn new(msg: *const crate::c_void, ts: MessageTypeSupport) -> Self {
        RosMessage { msg, ts }
    }
}

pub struct RosSerdes;

impl ZDeserializer for RosSerdes {
    type Input<'a> = (&'a Vec<u8>, &'a mut RosMessage);
    type Output = bool;
    fn deserialize<'a>(input: Self::Input<'a>) -> Self::Output {
        let (bytes, msg) = input;
        unsafe { msg.ts.deserialize_message(bytes, msg.msg as *mut _) };
        true
    }
}

impl ZSerializer for RosSerdes {
    type Input<'a> = &'a RosMessage;
    fn serialize(input: &RosMessage) -> Vec<u8> {
        let RosMessage { msg, ts } = input;
        unsafe { ts.serialize_message(*msg) }
    }
}

impl ZMessage for RosMessage {
    type Serdes = RosSerdes;
}

impl MessageTypeInfo for RosMessage {
    // Static methods return placeholder values since RosMessage is a generic wrapper
    // The actual type info varies per instance
    fn type_name() -> &'static str {
        "rcl_z::RosMessage"
    }

    fn type_hash() -> TypeHash {
        // Placeholder - actual hash is instance-specific
        TypeHash::zero()
    }

    // Dynamic methods fetch real type info from C++ type support
    fn type_name_dyn(&self) -> String {
        self.ts.get_type_prefix()
    }

    fn type_hash_dyn(&self) -> TypeHash {
        self.ts.get_type_hash()
    }

    fn type_info_dyn(&self) -> TypeInfo {
        self.ts.get_type_info()
    }
}

impl WithTypeInfo for RosMessage {}

unsafe impl Sync for RosMessage {}
unsafe impl Send for RosMessage {}

pub struct RosService;

impl ZService for RosService {
    type Response = RosMessage;
    type Request = RosMessage;
}

impl ServiceTypeInfo for RosService {
    // Static method returns placeholder value since RosService is a generic wrapper
    fn service_type_info() -> TypeInfo {
        TypeInfo::new("rcl_z::RosService", TypeHash::zero())
    }

    // Note: Dynamic method would need to be implemented differently for services
    // since RosService doesn't hold type support. This may require refactoring
    // to store ServiceTypeSupport in RosService instances.
}
