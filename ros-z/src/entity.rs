use std::fmt::Display;

use zenoh::Result;
use zenoh::{key_expr::KeyExpr, session::ZenohId};

use crate::attachment::GidArray;
use sha2::Digest;

#[derive(Debug, Hash, Clone)]
pub struct NodeEntity {
    pub domain_id: usize,
    pub z_id: ZenohId,
    pub id: usize,
    pub name: String,
    pub namespace: Option<String>,
}


const EMPTY_NAMESPACE: &'static str = "%";

impl NodeEntity {
    pub fn new(
        domain_id: usize,
        z_id: ZenohId,
        id: usize,
        name: String,
        namespace: Option<String>,
    ) -> Self {
        Self {
            domain_id,
            z_id,
            id,
            name,
            namespace,
        }
    }
}

impl Display for NodeEntity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // <ADMIN_SPACE>/<domainid>/<zid>/<nid>/<id>/<entity>/<namespace>/<nodename>
        let NodeEntity {
            domain_id,
            z_id,
            id,
            name,
            namespace,
        } = self;
        // TODO: polish this
        let x = EMPTY_NAMESPACE.to_string();
        let namespace = namespace.as_ref().unwrap_or(&x);
        write!(f, "{domain_id}/{z_id}/{id}/{id}/NN/{namespace}/{name}")
    }
}

#[derive(Debug, Hash, strum::EnumString, strum::Display)]
pub enum EndpointKind {
    #[strum(serialize = "MP")]
    Publisher,
    #[strum(serialize = "MS")]
    Subscription,
    #[strum(serialize = "SS")]
    Service,
    #[strum(serialize = "SC")]
    Client,
}

#[derive(Debug, Hash)]
pub struct TypeInfo {
    pub name: String,
    pub hash: String,
}

impl TypeInfo {
    pub fn new(name: &str, hash: &str) -> Self {
        TypeInfo {
            name: name.to_string(),
            hash: hash.to_string(),
        }
    }
}

impl Display for TypeInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let Self { name, hash } = self;
        write!(f, "{name}/{hash}")
    }
}

#[derive(Debug, Hash)]
pub struct EndpointEntity {
    pub id: usize,
    pub node: NodeEntity,
    pub kind: EndpointKind,
    pub topic: String,
    pub type_info: Option<TypeInfo>,
}

impl Display for EndpointEntity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // <ADMIN_SPACE>/<domainid>/<zid>/<id>/<entity>/<namespace>/<nodename>/<topic_name>/<topic_type>/<topic_type_hash>/<topic_qos>
        let EndpointEntity {
            id,
            node,
            kind,
            topic,
            type_info,
        } = self;
        let NodeEntity {
            domain_id,
            z_id,
            id: node_id,
            name: node_name,
            namespace: node_namespace,
        } = &node;
        let x = EMPTY_NAMESPACE.to_string();
        let node_namespace = node_namespace.as_ref().unwrap_or(&x);
        write!(
            f,
            "{domain_id}/{z_id}/{node_id}/{id}/{kind}/{node_namespace}/{node_name}/{topic}{}",
            type_info
                .as_ref()
                .map_or("".to_string(), |x| format!("/{x}"),)
        )
    }
}

impl EndpointEntity {
    pub fn topic_key_expr(&self) -> Result<KeyExpr<'static>> {
        // Example: 0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18
        let NodeEntity { domain_id, .. } = &self.node;
        let topic = {
            let s = &self.topic;
            let s = s.strip_prefix('/').unwrap_or(s);
            s.strip_suffix('/').unwrap_or(s)
        };
        let topic_info = if let Some(type_info) = &self.type_info {
            format!("{topic}/{type_info}")
        } else {
            topic.to_string()
        };
        format!("{domain_id}/{topic_info}").try_into()
    }

    pub fn gid(&self) -> GidArray {
        let mut gid = GidArray::default();
        let hash = sha2::Sha256::digest(self.to_string().as_bytes());
        let len = gid.len();
        gid.copy_from_slice(&hash[..len]);
        gid
    }
}

#[derive(Debug, Hash)]
pub enum Entity {
    Node(NodeEntity),
    Endpoint(EndpointEntity),
}

// impl From<NodeEntity> for Entity {
//     fn from(value: NodeEntity) -> Self {
//         let id = value.node_id;
//         Self {
//             node: value,
//             id,
//             sub: None,
//         }
//     }
// }

// impl GraphParticipant {
//     pub fn get_publisher_keyexpr(&self) -> String {
//         [
//             self.domain_id.to_string(),
//             Self::mangle_name(&self.endpoint_name),
//             Self::mangle_name(&self.endpoint_type.to_string()),
//             Self::mangle_name(&self.endpoint_typehash),
//             self.z_id.to_string(),
//             self.node_id.to_string(),
//         ]
//         .join("/")
//     }
// }
//
// // const auto target = "0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";
//
//<ADMIN_SPACE>/<domainid>/<zid>/<id>/<entity>/<namespace>/<nodename>/<topic_name>/<topic_type>/<topic_type_hash>/<topic_qos>
// "@ros2_lv/0/q1w2e3r4t5y6/1/32/MP/_/talker/dds_::std_msgs::msg::String/2:1:2,10".
//
//
// PUB KE: 0/rosout/rcl_interfaces::msg::dds_::Log_/RIHS01_e28ce254ca8abc06abf92773b74602cdbf116ed34fbaf294fb9f81da9f318eac
// PUB liveliness KE: @ros2_lv/0/ae35e834a80cb2398c738e251c05e716/0/1/MP/%/%/talker/%rosout/rcl_interfaces::msg::dds_::Log_/RIHS01_e28ce254ca8abc06abf92773b74602cdbf116ed34fbaf294fb9f81da9f318eac/:1:,1000:,:10,0:,,
// PUB KE: 0/parameter_events/rcl_interfaces::msg::dds_::ParameterEvent_/RIHS01_043e627780fcad87a22d225bc2a037361dba713fca6a6b9f4b869a5aa0393204
// PUB liveliness KE: @ros2_lv/0/ae35e834a80cb2398c738e251c05e716/0/8/MP/%/%/talker/%parameter_events/rcl_interfaces::msg::dds_::ParameterEvent_/RIHS01_043e627780fcad87a22d225bc2a037361dba713fca6a6b9f4b869a5aa0393204/::,1000:,:,:,,
// SUB KE: 0/parameter_events/rcl_interfaces::msg::dds_::ParameterEvent_/RIHS01_043e627780fcad87a22d225bc2a037361dba713fca6a6b9f4b869a5aa0393204
// SUB liveliness KE: @ros2_lv/0/ae35e834a80cb2398c738e251c05e716/0/9/MS/%/%/talker/%parameter_events/rcl_interfaces::msg::dds_::ParameterEvent_/RIHS01_043e627780fcad87a22d225bc2a037361dba713fca6a6b9f4b869a5aa0393204/::,1000:,:,:,,
// PUB KE: 0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18
// PUB liveliness KE: @ros2_lv/0/ae35e834a80cb2398c738e251c05e716/0/11/MP/%/%/talker/%chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/::,7:,:,:,,
