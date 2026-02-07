"""Auto-generated ROS 2 message types for std_msgs."""

import msgspec
from typing import ClassVar


class Int8(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "std_msgs/msg/Int8"
    __hash__: ClassVar[str] = (
        "RIHS01_26525065a403d972cb672f0777e333f0c799ad444ae5fcd79e43d1e73bd0f440"
    )


class Int64MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "std_msgs.MultiArrayLayout | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "std_msgs/msg/Int64MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_edc98ff2c11b17a38af7999d0009123738106c49f710e19ca872edda8f595a8b"
    )


class Bool(msgspec.Struct, frozen=True, kw_only=True):
    data: bool = False

    __msgtype__: ClassVar[str] = "std_msgs/msg/Bool"
    __hash__: ClassVar[str] = (
        "RIHS01_feb91e995ff9ebd09c0cb3d2aed18b11077585839fb5db80193b62d74528f6c9"
    )


class Float64MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "std_msgs.MultiArrayLayout | None" = None
    data: list[float] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "std_msgs/msg/Float64MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_710c64a1072f6c998def22f65ba3f10749d50ebc01184ed3607c40bcd1f6cc22"
    )


class MultiArrayDimension(msgspec.Struct, frozen=True, kw_only=True):
    label: str = ""
    size: int = 0
    stride: int = 0

    __msgtype__: ClassVar[str] = "std_msgs/msg/MultiArrayDimension"
    __hash__: ClassVar[str] = (
        "RIHS01_5e773a60a4c7fc8a54985f307c7837aa2994252a126c301957a24e31282c9cbe"
    )


class Int8MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "std_msgs.MultiArrayLayout | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "std_msgs/msg/Int8MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_daa8f33ecf631d38e49be8989d409f9ea5c15281f65b5c043b6ecdb9ba3c3bdc"
    )


class Header(msgspec.Struct, frozen=True, kw_only=True):
    stamp: "builtin_interfaces.Time | None" = msgspec.field(
        default_factory=lambda: {"sec": 0, "nanosec": 0}
    )
    frame_id: str = ""

    __msgtype__: ClassVar[str] = "std_msgs/msg/Header"
    __hash__: ClassVar[str] = (
        "RIHS01_f49fb3ae2cf070f793645ff749683ac6b06203e41c891e17701b1cb597ce6a01"
    )


class Float32(msgspec.Struct, frozen=True, kw_only=True):
    data: float = 0.0

    __msgtype__: ClassVar[str] = "std_msgs/msg/Float32"
    __hash__: ClassVar[str] = (
        "RIHS01_7170d3d8f841f7be3172ce5f4f59f3a4d7f63b0447e8b33327601ad64d83d6e2"
    )


class Int16(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "std_msgs/msg/Int16"
    __hash__: ClassVar[str] = (
        "RIHS01_1dcc3464e47c288a55f943a389d337cdb06804de3f5cd7a266b0de718eee17e5"
    )


class ByteMultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "std_msgs.MultiArrayLayout | None" = None
    data: bytes = b""

    __msgtype__: ClassVar[str] = "std_msgs/msg/ByteMultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_17223b88cda4ecb8eb60276a19b8110e7456c30938b5804c85b79d18d10d22f2"
    )


class UInt32MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "std_msgs.MultiArrayLayout | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt32MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_6f9b8b67fcd40e4c7fd5d5883c163e465e12bf3934dc4fdb3aaa18092d84bcd2"
    )


class UInt8MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "std_msgs.MultiArrayLayout | None" = None
    data: bytes = b""

    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt8MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_88c0cb2c9556da467c68c6cd865784e7ac7c8cae0e1683a752abec78b92e9629"
    )


class Int32(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "std_msgs/msg/Int32"
    __hash__: ClassVar[str] = (
        "RIHS01_b6578ded3c58c626cfe8d1a6fb6e04f706f97e9f03d2727c9ff4e74b1cef0deb"
    )


class Int16MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "std_msgs.MultiArrayLayout | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "std_msgs/msg/Int16MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_00c30d1b46b5aad16d7d5a84a987031ec83f85b38ea266d14063dcc33bdd6605"
    )


class UInt16MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "std_msgs.MultiArrayLayout | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt16MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_738da9d37b14efa750a70a80f03a8327c0e11a0f06d2bd1c7d16d012886c3a89"
    )


class Float32MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "std_msgs.MultiArrayLayout | None" = None
    data: list[float] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "std_msgs/msg/Float32MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_9464d01f2df8b20ae934d3a81c66f827b6a7166662e88a2350d8901b48bae2cf"
    )


class String(msgspec.Struct, frozen=True, kw_only=True):
    data: str = ""

    __msgtype__: ClassVar[str] = "std_msgs/msg/String"
    __hash__: ClassVar[str] = (
        "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
    )


class MultiArrayLayout(msgspec.Struct, frozen=True, kw_only=True):
    dim: list["std_msgs.MultiArrayDimension"] = msgspec.field(default_factory=list)
    data_offset: int = 0

    __msgtype__: ClassVar[str] = "std_msgs/msg/MultiArrayLayout"
    __hash__: ClassVar[str] = (
        "RIHS01_4c66e6f78e740ac103a94cf63259f968e48c617e7699e829b63c21a5cb50dac6"
    )


class UInt8(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt8"
    __hash__: ClassVar[str] = (
        "RIHS01_6138bd83d8c3569cb80a667db03cfc1629f529fee79d944c39c34e352e72f010"
    )


class Empty(msgspec.Struct, frozen=True, kw_only=True):
    __msgtype__: ClassVar[str] = "std_msgs/msg/Empty"
    __hash__: ClassVar[str] = (
        "RIHS01_20b625256f32d5dbc0d04fee44f43c41e51c70d3502f84b4a08e7a9c26a96312"
    )


class Byte(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "std_msgs/msg/Byte"
    __hash__: ClassVar[str] = (
        "RIHS01_03c6c2278882b7b237e69ab9c4e75d8fa5741972c2abda44ea682268b4f2bbd3"
    )


class Char(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "std_msgs/msg/Char"
    __hash__: ClassVar[str] = (
        "RIHS01_3ad2d04dd29ba19d04b16659afa3ccaedd691914b02a64e82e252f2fa6a586a9"
    )


class UInt64MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "std_msgs.MultiArrayLayout | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt64MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_71cf4fd4cf20b69d1c98715d948154baefbfbe7b9c629fa1b5db4614ca06c929"
    )


class UInt64(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt64"
    __hash__: ClassVar[str] = (
        "RIHS01_fbdc52018fc13755dce18024d1a671c856aa8b4aaf63adfb095b608f98e8c943"
    )


class UInt16(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt16"
    __hash__: ClassVar[str] = (
        "RIHS01_08a406e4b022bc22e907f985d6a9e9dd1d4fbecae573549cf49350113e7757b1"
    )


class UInt32(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt32"
    __hash__: ClassVar[str] = (
        "RIHS01_a5c874829b752bc5fa190024b0ad76f578cc278271e855c7d02a818b3516fb4a"
    )


class Float64(msgspec.Struct, frozen=True, kw_only=True):
    data: float = 0.0

    __msgtype__: ClassVar[str] = "std_msgs/msg/Float64"
    __hash__: ClassVar[str] = (
        "RIHS01_705ba9c3d1a09df43737eb67095534de36fd426c0587779bda2bc51fe790182a"
    )


class Int32MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "std_msgs.MultiArrayLayout | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "std_msgs/msg/Int32MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_9c7af5ff72d7eec40261f723066229358fad36bea0f5137c17faf202aab36be6"
    )


class Int64(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "std_msgs/msg/Int64"
    __hash__: ClassVar[str] = (
        "RIHS01_8cd1048c2f186b6bd9a92472dc1ce51723c0833a221e2b7aecfff111774f4b49"
    )


class ColorRGBA(msgspec.Struct, frozen=True, kw_only=True):
    r: float = 0.0
    g: float = 0.0
    b: float = 0.0
    a: float = 0.0

    __msgtype__: ClassVar[str] = "std_msgs/msg/ColorRGBA"
    __hash__: ClassVar[str] = (
        "RIHS01_77a7a5b9ae477306097665106e0413ba74440245b1f3d0c6d6405fe5c7813fe8"
    )
