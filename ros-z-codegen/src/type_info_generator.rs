use proc_macro2::TokenStream;
use quote::quote;

/// Helper struct for generating MessageTypeInfo trait implementations
pub struct TypeInfoGenerator;

impl TypeInfoGenerator {
    /// Generate MessageTypeInfo implementation for a ROS message type
    pub fn generate_type_info_impl(
        rust_type_name: &str,
        ros_type_name: &str,
        type_hash: &str,
    ) -> String {
        format!(
            r#"
impl ::ros_z::MessageTypeInfo for {rust_type_name} {{
    fn type_name() -> &'static str {{
        "{ros_type_name}"
    }}

    fn type_hash() -> ::ros_z::entity::TypeHash {{
        ::ros_z::entity::TypeHash::from_rihs_string("{type_hash}")
            .expect("Invalid RIHS hash string")
    }}

    fn type_info() -> ::ros_z::entity::TypeInfo {{
        ::ros_z::entity::TypeInfo::new(Self::type_name(), Self::type_hash())
    }}
}}
"#,
            rust_type_name = rust_type_name,
            ros_type_name = ros_type_name,
            type_hash = type_hash
        )
    }

    /// Convert a ROS message name to a Rust module path
    /// Example: "geometry_msgs/Vector3" -> "geometry_msgs::msg::Vector3_"
    pub fn ros_name_to_rust_path(pkg: &str, msg_name: &str) -> String {
        format!("{}::msg::{}_ ", pkg, msg_name)
    }

    /// Convert a ROS message name to DDS format (used in ROS 2)
    /// Example: "geometry_msgs/Vector3" -> "geometry_msgs::msg::dds_::Vector3_"
    pub fn ros_name_to_dds_format(pkg: &str, msg_name: &str) -> String {
        format!("{}::msg::dds_::{}_ ", pkg, msg_name)
    }
}
