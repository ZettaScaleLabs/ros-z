use crate::entity::{TypeHash, TypeInfo};

/// Trait for ROS messages that provides compile-time message metadata
/// This trait is independent of serialization format
pub trait MessageTypeInfo {
    /// Returns the ROS message type name (e.g., "geometry_msgs::msg::dds_::Vector3_")
    fn type_name() -> &'static str;

    /// Returns the type hash (RIHS01 for ROS2, MD5 for ROS1)
    fn type_hash() -> TypeHash;

    /// Returns complete TypeInfo combining name and hash
    fn type_info() -> TypeInfo {
        TypeInfo::new(Self::type_name(), Self::type_hash())
    }

    /// Returns the package name (extracted from type name)
    fn package_name() -> &'static str {
        Self::type_name().split("::").next().unwrap_or("unknown")
    }

    /// Returns whether this message has a fixed size (for optimization)
    fn is_fixed_size() -> bool {
        false
    }
}

/// Backward compatibility alias for existing code
pub trait WithTypeInfo: MessageTypeInfo {}

/// Trait for ROS service types that provides service-level type information
/// For services, the type name should be based on the service name (not Request/Response)
/// and the hash should be the composite service hash (not just request or response hash).
///
/// This trait should be implemented by the code generator for each service type.
/// The service hash in ROS2 is computed from a composite type that includes:
/// - request_message (the Request type)
/// - response_message (the Response type)
/// - event_message (a virtual Event type containing ServiceEventInfo, request[], and response[])
pub trait ServiceTypeInfo {
    /// Returns the service type info (type name and hash for the service)
    fn service_type_info() -> TypeInfo;
}

pub mod srv {

    use crate::msg::{ZMessage, ZService};

    #[allow(non_snake_case)]
    pub mod AddTwoInts {
        use serde::{Deserialize, Serialize};

        pub type Service = (Request, Response);

        #[derive(Debug, Serialize, Deserialize, Default, Clone)]
        pub struct Request {
            pub a: i64,
            pub b: i64,
        }

        #[derive(Debug, Serialize, Deserialize, Default, Clone)]
        pub struct Response {
            pub sum: i64,
        }
    }

    pub enum ZSrv<L, R> {
        L(L),
        R(R),
    }
    impl<RQ: ZMessage, RP: ZMessage> ZService for ZSrv<RQ, RP> {
        type Request = RQ;
        type Response = RP;
    }
    impl<RQ: ZMessage, RP: ZMessage> ZService for (RQ, RP) {
        type Request = RQ;
        type Response = RP;
    }
}
