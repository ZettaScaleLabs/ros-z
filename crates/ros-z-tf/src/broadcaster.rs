use std::num::NonZeroUsize;

use ros_z::Builder;
use ros_z::msg::NativeCdrSerdes;
use ros_z::node::ZNode;
use ros_z::pubsub::ZPub;
use ros_z::qos::{QosDurability, QosHistory, QosProfile, QosReliability};
use ros_z_msgs::geometry_msgs::TransformStamped;
use ros_z_msgs::tf2_msgs::TFMessage;

type TfPub = ZPub<TFMessage, NativeCdrSerdes<TFMessage>>;

fn volatile_qos() -> QosProfile {
    QosProfile {
        reliability: QosReliability::Reliable,
        durability: QosDurability::Volatile,
        history: QosHistory::KeepLast(NonZeroUsize::new(100).unwrap()),
        ..Default::default()
    }
}

fn transient_local_qos() -> QosProfile {
    QosProfile {
        reliability: QosReliability::Reliable,
        durability: QosDurability::TransientLocal,
        history: QosHistory::KeepLast(NonZeroUsize::new(100).unwrap()),
        ..Default::default()
    }
}

/// Publishes dynamic transforms to `/tf` (Volatile durability).
pub struct TransformBroadcaster {
    pub_: TfPub,
}

impl TransformBroadcaster {
    /// Create a broadcaster attached to `node`. Declares a publisher on `/tf`.
    pub fn new(node: &ZNode) -> zenoh::Result<Self> {
        let pub_ = node
            .create_pub::<TFMessage>("/tf")
            .with_qos(volatile_qos())
            .build()?;
        Ok(Self { pub_ })
    }

    /// Publish a single transform to `/tf`.
    pub fn send_transform(&self, tf: TransformStamped) -> zenoh::Result<()> {
        self.send_transforms(vec![tf])
    }

    /// Publish multiple transforms to `/tf` in a single message.
    pub fn send_transforms(&self, transforms: Vec<TransformStamped>) -> zenoh::Result<()> {
        self.pub_.publish(&TFMessage { transforms })
    }
}

/// Publishes static transforms to `/tf_static` (TransientLocal durability).
///
/// Late-joining subscribers automatically receive all previously published
/// static transforms via `PublicationCache` replay.
pub struct StaticTransformBroadcaster {
    pub_: TfPub,
}

impl StaticTransformBroadcaster {
    /// Create a static broadcaster attached to `node`. Declares a publisher on `/tf_static`.
    pub fn new(node: &ZNode) -> zenoh::Result<Self> {
        let pub_ = node
            .create_pub::<TFMessage>("/tf_static")
            .with_qos(transient_local_qos())
            .build()?;
        Ok(Self { pub_ })
    }

    /// Publish a single static transform to `/tf_static`.
    pub fn send_transform(&self, tf: TransformStamped) -> zenoh::Result<()> {
        self.send_transforms(vec![tf])
    }

    /// Publish multiple static transforms to `/tf_static` in a single message.
    pub fn send_transforms(&self, transforms: Vec<TransformStamped>) -> zenoh::Result<()> {
        self.pub_.publish(&TFMessage { transforms })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_tf(parent: &str, child: &str) -> TransformStamped {
        use ros_z_msgs::builtin_interfaces::Time;
        use ros_z_msgs::geometry_msgs::{Quaternion, Transform, Vector3};
        use ros_z_msgs::std_msgs::Header;
        TransformStamped {
            header: Header {
                frame_id: parent.to_string(),
                stamp: Time { sec: 1, nanosec: 0 },
                ..Default::default()
            },
            child_frame_id: child.to_string(),
            transform: Transform {
                translation: Vector3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        }
    }

    #[test]
    fn send_transform_builds_correct_message() {
        let tf = make_tf("map", "odom");
        let msg = TFMessage {
            transforms: vec![tf.clone()],
        };
        assert_eq!(msg.transforms.len(), 1);
        assert_eq!(msg.transforms[0].child_frame_id, "odom");
    }

    #[test]
    fn send_transforms_batches_multiple() {
        let tfs = vec![make_tf("map", "odom"), make_tf("odom", "base_link")];
        let msg = TFMessage { transforms: tfs };
        assert_eq!(msg.transforms.len(), 2);
    }
}
