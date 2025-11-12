use crate::entity::{TypeHash, TypeInfo};

/// Trait for ROS messages that provides message metadata
/// This trait supports both compile-time (static) and runtime (dynamic) type information.
///
/// ## Static Methods (Compile-time)
/// For generated Rust types where type info is known at compile time (e.g., ros-z-msgs).
///
/// ## Dynamic Methods (Runtime)
/// For wrapper types where type info must be queried at runtime (e.g., rcl-z RosMessage).
/// Default implementations delegate to static methods for backward compatibility.
pub trait MessageTypeInfo {
    /// Returns the ROS message type name (e.g., "geometry_msgs::msg::dds_::Vector3_")
    /// Static method for compile-time known types
    fn type_name() -> &'static str;

    /// Returns the type hash (RIHS01 for ROS2, MD5 for ROS1)
    /// Static method for compile-time known types
    fn type_hash() -> TypeHash;

    /// Returns complete TypeInfo combining name and hash
    /// Static method for compile-time known types
    fn type_info() -> TypeInfo {
        TypeInfo::new(Self::type_name(), Self::type_hash())
    }

    /// Returns the package name (extracted from type name)
    /// Static method for compile-time known types
    fn package_name() -> &'static str {
        Self::type_name().split("::").next().unwrap_or("unknown")
    }

    /// Returns whether this message has a fixed size (for optimization)
    /// Static method for compile-time known types
    fn is_fixed_size() -> bool {
        false
    }

    // === Dynamic Methods (Runtime) ===

    /// Returns the ROS message type name at runtime
    /// Override this for types that need to query type info dynamically
    fn type_name_dyn(&self) -> String {
        Self::type_name().to_string()
    }

    /// Returns the type hash at runtime
    /// Override this for types that need to query type info dynamically
    fn type_hash_dyn(&self) -> TypeHash {
        Self::type_hash()
    }

    /// Returns complete TypeInfo at runtime
    /// Override this for types that need to query type info dynamically
    fn type_info_dyn(&self) -> TypeInfo {
        TypeInfo::new(&self.type_name_dyn(), self.type_hash_dyn())
    }

    /// Returns the package name at runtime
    fn package_name_dyn(&self) -> String {
        self.type_name_dyn()
            .split("::")
            .next()
            .unwrap_or("unknown")
            .to_string()
    }
}

/// Backward compatibility alias for existing code
pub trait WithTypeInfo: MessageTypeInfo {}

/// Trait for ROS service types that provides service-level type information
/// This trait supports both compile-time (static) and runtime (dynamic) type information.
///
/// For services, the type name should be based on the service name (not Request/Response)
/// and the hash should be the composite service hash (not just request or response hash).
///
/// The service hash in ROS2 is computed from a composite type that includes:
/// - request_message (the Request type)
/// - response_message (the Response type)
/// - event_message (a virtual Event type containing ServiceEventInfo, request[], and response[])
///
/// ## Static Methods (Compile-time)
/// For generated Rust service types where type info is known at compile time (e.g., ros-z-msgs).
///
/// ## Dynamic Methods (Runtime)
/// For wrapper types where type info must be queried at runtime (e.g., rcl-z RosService).
/// Default implementations delegate to static methods for backward compatibility.
pub trait ServiceTypeInfo {
    /// Returns the service type info (type name and hash for the service)
    /// Static method for compile-time known types
    fn service_type_info() -> TypeInfo;

    /// Returns the service type info at runtime
    /// Override this for types that need to query type info dynamically
    fn service_type_info_dyn(&self) -> TypeInfo {
        Self::service_type_info()
    }
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
