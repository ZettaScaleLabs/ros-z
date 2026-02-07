"""Auto-generated ROS 2 message types for geometry_msgs."""

import msgspec
from typing import ClassVar


class QuaternionStamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    quaternion: "geometry_msgs.Quaternion | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/QuaternionStamped"
    __hash__: ClassVar[str] = (
        "RIHS01_08102545bbf3be2f05cef58e5e2cf2bcdd952a2ebceb37d9dc89838c05eac93d"
    )


class PointStamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    point: "geometry_msgs.Point | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PointStamped"
    __hash__: ClassVar[str] = (
        "RIHS01_e5d7f8f41db45ffdf4623e912f33b7153bc9cf5cd629f6c91438d1a25fb2086c"
    )


class Wrench(msgspec.Struct, frozen=True, kw_only=True):
    force: "geometry_msgs.Vector3 | None" = None
    torque: "geometry_msgs.Vector3 | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Wrench"
    __hash__: ClassVar[str] = (
        "RIHS01_018e8519d57c16adbe97c9fe1460ef21fec7e31bc541de3d653a35895677ce52"
    )


class PolygonInstance(msgspec.Struct, frozen=True, kw_only=True):
    polygon: "geometry_msgs.Polygon | None" = None
    id: int = 0

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PolygonInstance"
    __hash__: ClassVar[str] = (
        "RIHS01_e004c1a05442fa20f781cd33a9ace1080439b6948ad92f3aa43ef3dc3ad0b612"
    )


class Inertia(msgspec.Struct, frozen=True, kw_only=True):
    m: float = 0.0
    com: "geometry_msgs.Vector3 | None" = None
    ixx: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyy: float = 0.0
    iyz: float = 0.0
    izz: float = 0.0

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Inertia"
    __hash__: ClassVar[str] = (
        "RIHS01_2ddd5dab5c347825ba2e56c895ddccfd0b8efe53ae931bf67f905529930b4bd7"
    )


class Quaternion(msgspec.Struct, frozen=True, kw_only=True):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 0.0

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Quaternion"
    __hash__: ClassVar[str] = (
        "RIHS01_8a765f66778c8ff7c8ab94afcc590a2ed5325a1d9a076ffff38fbce36f458684"
    )


class Vector3(msgspec.Struct, frozen=True, kw_only=True):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Vector3"
    __hash__: ClassVar[str] = (
        "RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d"
    )


class AccelStamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    accel: "geometry_msgs.Accel | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/AccelStamped"
    __hash__: ClassVar[str] = (
        "RIHS01_854383e034e15fe92ede4aac8432d3dba62d1d1fa1812b68227df2ec2ad91b30"
    )


class TwistWithCovarianceStamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    twist: "geometry_msgs.TwistWithCovariance | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/TwistWithCovarianceStamped"
    __hash__: ClassVar[str] = (
        "RIHS01_deedf672d5df96d4634eddbbe33ed40d3bfa7a209a51f4fef68dcb5225e11b64"
    )


class TransformStamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    child_frame_id: str = ""
    transform: "geometry_msgs.Transform | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/TransformStamped"
    __hash__: ClassVar[str] = (
        "RIHS01_2172ac533a0f3f3b22069bbda9ec684fe46a265270301755dd79e86bdf49bcbe"
    )


class Point(msgspec.Struct, frozen=True, kw_only=True):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Point"
    __hash__: ClassVar[str] = (
        "RIHS01_6963084842a9b04494d6b2941d11444708d892da2f4b09843b9c43f42a7f6881"
    )


class PoseWithCovarianceStamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    pose: "geometry_msgs.PoseWithCovariance | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PoseWithCovarianceStamped"
    __hash__: ClassVar[str] = (
        "RIHS01_522581568f7d14bf7fd7149caf07c8a3cf76e3c722addfb971f34f3907c41795"
    )


class Pose2D(msgspec.Struct, frozen=True, kw_only=True):
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Pose2D"
    __hash__: ClassVar[str] = (
        "RIHS01_d68efa5b46e70f7b16ca23085474fdac5a44b638783ec42f661da64da4724ccc"
    )


class TwistWithCovariance(msgspec.Struct, frozen=True, kw_only=True):
    twist: "geometry_msgs.Twist | None" = None
    covariance: list[float] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/TwistWithCovariance"
    __hash__: ClassVar[str] = (
        "RIHS01_8e4a0fc072036eea0945ad64be41149a6bf642f11d048e3b483e63e93e0cc623"
    )


class Transform(msgspec.Struct, frozen=True, kw_only=True):
    translation: "geometry_msgs.Vector3 | None" = None
    rotation: "geometry_msgs.Quaternion | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Transform"
    __hash__: ClassVar[str] = (
        "RIHS01_beb83fbe698636351461f6f35d1abb20010c43d55374d81bd041f1ba2581fddc"
    )


class VelocityStamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    body_frame_id: str = ""
    reference_frame_id: str = ""
    velocity: "geometry_msgs.Twist | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/VelocityStamped"
    __hash__: ClassVar[str] = (
        "RIHS01_8781a6ca5bf3eff3fb5ad800fc5af8e7c356a9d8a6c5449b245b9a44bcde3bab"
    )


class InertiaStamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    inertia: "geometry_msgs.Inertia | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/InertiaStamped"
    __hash__: ClassVar[str] = (
        "RIHS01_90cfe8b4bceadd8f0eecd02cb9178756aae704fbec5ae5354437ebc5880278b2"
    )


class Twist(msgspec.Struct, frozen=True, kw_only=True):
    linear: "geometry_msgs.Vector3 | None" = None
    angular: "geometry_msgs.Vector3 | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Twist"
    __hash__: ClassVar[str] = (
        "RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a"
    )


class PolygonStamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    polygon: "geometry_msgs.Polygon | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PolygonStamped"
    __hash__: ClassVar[str] = (
        "RIHS01_43faa3510fabf610793de253717e9296eb225508d78b573b335cb27774248e65"
    )


class Point32(msgspec.Struct, frozen=True, kw_only=True):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Point32"
    __hash__: ClassVar[str] = (
        "RIHS01_2fc4db7cae16a4582c79a56b66173a8d48d52c7dc520ddc55a0d4bcf2a4bfdbc"
    )


class PolygonInstanceStamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    polygon: "geometry_msgs.PolygonInstance | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PolygonInstanceStamped"
    __hash__: ClassVar[str] = (
        "RIHS01_e24a66fb9e41a350991a058b4375a8f757d1cf762fd541413f04b906ac73e3a0"
    )


class AccelWithCovarianceStamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    accel: "geometry_msgs.AccelWithCovariance | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/AccelWithCovarianceStamped"
    __hash__: ClassVar[str] = (
        "RIHS01_798923ce9800871d36b094a8d6a1317d6960404b979351e39f959a7d74fe9169"
    )


class Accel(msgspec.Struct, frozen=True, kw_only=True):
    linear: "geometry_msgs.Vector3 | None" = None
    angular: "geometry_msgs.Vector3 | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Accel"
    __hash__: ClassVar[str] = (
        "RIHS01_dc448243ded9b1fcbcca24aba0c22f013dae06c354ba2d849571c0a2a3f57ca0"
    )


class PoseStamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    pose: "geometry_msgs.Pose | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PoseStamped"
    __hash__: ClassVar[str] = (
        "RIHS01_74ce7f85e3559a142668c1ed972632f07c4deb9e731e0ccafd36cab66c243caa"
    )


class Pose(msgspec.Struct, frozen=True, kw_only=True):
    position: "geometry_msgs.Point | None" = None
    orientation: "geometry_msgs.Quaternion | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Pose"
    __hash__: ClassVar[str] = (
        "RIHS01_d501954e9476cea2996984e812054b68026ae0bfae789d9a10b23daf35cc90fa"
    )


class WrenchStamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    wrench: "geometry_msgs.Wrench | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/WrenchStamped"
    __hash__: ClassVar[str] = (
        "RIHS01_6eaafce47863ed26775d2be7fe3d5a4d096fdcf76398b8f851576d7d8dd51b44"
    )


class AccelWithCovariance(msgspec.Struct, frozen=True, kw_only=True):
    accel: "geometry_msgs.Accel | None" = None
    covariance: list[float] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/AccelWithCovariance"
    __hash__: ClassVar[str] = (
        "RIHS01_9f9ea639e3282f3f746759ab888a7e864393d3c6a84cf42f96d5c7b2bcdb7c8b"
    )


class Vector3Stamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    vector: "geometry_msgs.Vector3 | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Vector3Stamped"
    __hash__: ClassVar[str] = (
        "RIHS01_8e52cf433d6c87f9f01dd8b7a4b2a3a08135de44c639ac76fa8975390fd3d6bd"
    )


class PoseWithCovariance(msgspec.Struct, frozen=True, kw_only=True):
    pose: "geometry_msgs.Pose | None" = None
    covariance: list[float] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PoseWithCovariance"
    __hash__: ClassVar[str] = (
        "RIHS01_7e9ab4e6b590ceec936178376b944fd06ffa6f0a9f37ba67464a4d534813b144"
    )


class TwistStamped(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    twist: "geometry_msgs.Twist | None" = None

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/TwistStamped"
    __hash__: ClassVar[str] = (
        "RIHS01_c75f07101edaeac089e5414274e4b3d13711b77ad198c6253c132c0946e29fa4"
    )


class PoseArray(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    poses: list["geometry_msgs.Pose"] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PoseArray"
    __hash__: ClassVar[str] = (
        "RIHS01_52b3aaaa2e2367611da609b445b71a5a50bcc5302bc5a7a920345badd7a7b5f5"
    )


class Polygon(msgspec.Struct, frozen=True, kw_only=True):
    points: list["geometry_msgs.Point32"] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Polygon"
    __hash__: ClassVar[str] = (
        "RIHS01_3782f9f0bf044964d692d6c017d705e37611afb1f0bf6a9dee248a7dda0f784a"
    )
