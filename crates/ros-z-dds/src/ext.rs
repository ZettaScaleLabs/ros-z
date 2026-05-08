//! DdsBridgeExt — typed extension trait on ZNode for ergonomic DDS bridging.

use std::time::Duration;

use anyhow::Result;
use ros_z::{MessageTypeInfo, node::ZNode};
use ros_z_protocol::entity::TypeHash;

use crate::{
    names::dds_type_to_ros2_type,
    participant::{BridgeQos, DdsParticipant},
    pubsub::{ZDdsPubBridge, ZDdsSubBridge},
    service::{ZDdsClientBridge, ZDdsServiceBridge},
};

/// Typed convenience methods on [`ZNode`] for creating DDS↔Zenoh bridges.
///
/// Each method extracts the ROS 2 type name and type hash from the compile-time
/// type parameter `T` and delegates to the corresponding untyped bridge constructor.
///
/// ```rust,ignore
/// use ros_z_dds::DdsBridgeExt;
///
/// let _pub = node.bridge_dds_pub::<RosString>("/chatter", &participant).await?;
/// let _sub = node.bridge_dds_sub::<RosString>("/chatter", &participant).await?;
/// ```
pub trait DdsBridgeExt {
    /// Bridge a DDS topic publisher to a Zenoh publisher.
    ///
    /// The type `T` must implement `MessageTypeInfo` — all generated ros-z message
    /// types satisfy this automatically.
    async fn bridge_dds_pub<T: MessageTypeInfo>(
        &self,
        topic: &str,
        participant: &impl DdsParticipant,
    ) -> Result<ZDdsPubBridge<impl DdsParticipant>>;

    /// Bridge a Zenoh subscriber to a DDS topic writer.
    async fn bridge_dds_sub<T: MessageTypeInfo>(
        &self,
        topic: &str,
        participant: &impl DdsParticipant,
    ) -> Result<ZDdsSubBridge<impl DdsParticipant>>;

    /// Bridge a DDS service server to a Zenoh queryable.
    async fn bridge_dds_service<T: MessageTypeInfo>(
        &self,
        name: &str,
        participant: &impl DdsParticipant,
        timeout: Duration,
    ) -> Result<ZDdsServiceBridge<impl DdsParticipant>>;

    /// Bridge a DDS service client to a Zenoh querier.
    async fn bridge_dds_client<T: MessageTypeInfo>(
        &self,
        name: &str,
        participant: &impl DdsParticipant,
        timeout: Duration,
    ) -> Result<ZDdsClientBridge<impl DdsParticipant>>;
}

impl DdsBridgeExt for ZNode {
    async fn bridge_dds_pub<T: MessageTypeInfo>(
        &self,
        topic: &str,
        participant: &impl DdsParticipant,
    ) -> Result<ZDdsPubBridge<impl DdsParticipant>> {
        let ros2_type = dds_type_to_ros2_type(T::type_name());
        let type_hash: TypeHash = T::type_hash();
        ZDdsPubBridge::new(
            self,
            topic,
            &ros2_type,
            Some(type_hash),
            participant,
            BridgeQos::default(),
            true,
            10,
        )
        .await
    }

    async fn bridge_dds_sub<T: MessageTypeInfo>(
        &self,
        topic: &str,
        participant: &impl DdsParticipant,
    ) -> Result<ZDdsSubBridge<impl DdsParticipant>> {
        let ros2_type = dds_type_to_ros2_type(T::type_name());
        let type_hash: TypeHash = T::type_hash();
        ZDdsSubBridge::new(
            self,
            topic,
            &ros2_type,
            Some(type_hash),
            participant,
            BridgeQos::default(),
            true,
        )
        .await
    }

    async fn bridge_dds_service<T: MessageTypeInfo>(
        &self,
        name: &str,
        participant: &impl DdsParticipant,
        timeout: Duration,
    ) -> Result<ZDdsServiceBridge<impl DdsParticipant>> {
        let ros2_type = dds_type_to_ros2_type(T::type_name());
        let type_hash: TypeHash = T::type_hash();
        ZDdsServiceBridge::new(
            self,
            name,
            &ros2_type,
            Some(type_hash),
            participant,
            BridgeQos::default(),
            timeout,
        )
        .await
    }

    async fn bridge_dds_client<T: MessageTypeInfo>(
        &self,
        name: &str,
        participant: &impl DdsParticipant,
        timeout: Duration,
    ) -> Result<ZDdsClientBridge<impl DdsParticipant>> {
        let ros2_type = dds_type_to_ros2_type(T::type_name());
        let type_hash: TypeHash = T::type_hash();
        ZDdsClientBridge::new(
            self,
            name,
            &ros2_type,
            Some(type_hash),
            participant,
            BridgeQos::default(),
            timeout,
        )
        .await
    }
}
