"""Auto-generated ROS 2 message types for example_interfaces."""

import msgspec
from typing import ClassVar


class UInt16(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/UInt16"
    __hash__: ClassVar[str] = (
        "RIHS01_e123d0a691fa0ce58b682f1a4eee55137dd3f20c81665f0c556f53596c7fb377"
    )


class Float64MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "example_interfaces.MultiArrayLayout | None" = None
    data: list[float] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Float64MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_7176d4a5ec53eae7df454a68ac15d11ff15db5df5d29948652eb7372c9eb22a7"
    )


class ByteMultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "example_interfaces.MultiArrayLayout | None" = None
    data: bytes = b""

    __msgtype__: ClassVar[str] = "example_interfaces/msg/ByteMultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_9e9749a3bd74cc775f014ee7a450141135a30684ec3dceecfe0a2199bbf1dd4f"
    )


class Float32(msgspec.Struct, frozen=True, kw_only=True):
    data: float = 0.0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Float32"
    __hash__: ClassVar[str] = (
        "RIHS01_6a112d9235f8e8088d7a2bc77cb955341ac0d5c9870bdc592651a4186bb246f3"
    )


class Int64(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Int64"
    __hash__: ClassVar[str] = (
        "RIHS01_1b3b9a6502f560d079520c73c685a9550e5a1838d2cefd537fe0aba75a3639a0"
    )


class MultiArrayLayout(msgspec.Struct, frozen=True, kw_only=True):
    dim: list["example_interfaces.MultiArrayDimension"] = msgspec.field(
        default_factory=list
    )
    data_offset: int = 0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/MultiArrayLayout"
    __hash__: ClassVar[str] = (
        "RIHS01_ba42ea30074e5826a1e91f70f3660dda2996937169487f877fc20a8a402e2c27"
    )


class Int8MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "example_interfaces.MultiArrayLayout | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Int8MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_734b525732547f74adafe8653ab89750b9256bda71572c9537a830c62c47c53b"
    )


class Bool(msgspec.Struct, frozen=True, kw_only=True):
    data: bool = False

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Bool"
    __hash__: ClassVar[str] = (
        "RIHS01_4765c142500f8fd4e1a32fb3edd7b7d9d822a16ec270445f5120e772c5f9aed5"
    )


class UInt64MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "example_interfaces.MultiArrayLayout | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "example_interfaces/msg/UInt64MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_d800076c044d23f2279d763fdd312d94dfb2e13667c5e1bb49c820efcd7e421c"
    )


class Int16MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "example_interfaces.MultiArrayLayout | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Int16MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_2693cf95eef4c94ac590516638b2ef51465ab64ab4a103869729922066933a02"
    )


class UInt32MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "example_interfaces.MultiArrayLayout | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "example_interfaces/msg/UInt32MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_417c4cc5116cccbb42da4545cb0589dd0719d67115e7978b6c160d1b5800ecd4"
    )


class UInt32(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/UInt32"
    __hash__: ClassVar[str] = (
        "RIHS01_e86cccba586f30c16498f3ccd3550a764b255239a6142be3a4d7a2fa9a43515c"
    )


class Byte(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Byte"
    __hash__: ClassVar[str] = (
        "RIHS01_442c03677b2a9f7b9553b5ac364a6237840bc3ef4c0bc9e0bc326b5e23d1d335"
    )


class Char(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Char"
    __hash__: ClassVar[str] = (
        "RIHS01_320dcd57e1183fb08463cc3ab50bf7e5ce0ecee39f64d15a9e9eeca3384c91a5"
    )


class Empty(msgspec.Struct, frozen=True, kw_only=True):
    __msgtype__: ClassVar[str] = "example_interfaces/msg/Empty"
    __hash__: ClassVar[str] = (
        "RIHS01_73c22a7341eeccf8ef504a991e60d4078223f0931a5d5d212800e7c978903c58"
    )


class MultiArrayDimension(msgspec.Struct, frozen=True, kw_only=True):
    label: str = ""
    size: int = 0
    stride: int = 0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/MultiArrayDimension"
    __hash__: ClassVar[str] = (
        "RIHS01_a785cb9839e177e3eb760260139a919fec87821edc3314c592f2725abbf0bfcd"
    )


class UInt8(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/UInt8"
    __hash__: ClassVar[str] = (
        "RIHS01_9255b6d0dd98f5b573afbff223131279b788ac45cf051fb462c12dd9a30f4061"
    )


class UInt64(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/UInt64"
    __hash__: ClassVar[str] = (
        "RIHS01_6a3f8548c5818b7add62dd6cbbd840fd1ab17fbf9d73cad6690557b7326d8908"
    )


class Int32(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Int32"
    __hash__: ClassVar[str] = (
        "RIHS01_5cd04cd7f3adb9d6c6064c316047b24c76622eb89144f300b536d657fd55e652"
    )


class UInt8MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "example_interfaces.MultiArrayLayout | None" = None
    data: bytes = b""

    __msgtype__: ClassVar[str] = "example_interfaces/msg/UInt8MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_90a7d0b0d0a17f43664f4951d9eef55864d237f0cd2a553c5d12b644392d937c"
    )


class Int64MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "example_interfaces.MultiArrayLayout | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Int64MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_9ddc055712dca8d61990f61441193cc80375841f42bbb98fcc7c0ad4688bb345"
    )


class Int32MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "example_interfaces.MultiArrayLayout | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Int32MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_0988bc221fccb138dd3b678486fa5f4e4be5a1b84e420634c3ca856b7940b9d3"
    )


class UInt16MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "example_interfaces.MultiArrayLayout | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "example_interfaces/msg/UInt16MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_f3ab8f7db3e1cb5aed51393e1c432b193b74e68a7eb26f6c9b54dc17b260de0c"
    )


class Float64(msgspec.Struct, frozen=True, kw_only=True):
    data: float = 0.0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Float64"
    __hash__: ClassVar[str] = (
        "RIHS01_74c137b7930c26339425a95fcfab441199bc41e0e572d3a0c9e95badd72b50da"
    )


class Int16(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Int16"
    __hash__: ClassVar[str] = (
        "RIHS01_332d94306732e4e35da38e5ae744ff35bbdaeca300908dc43488d3a844687cd6"
    )


class Float32MultiArray(msgspec.Struct, frozen=True, kw_only=True):
    layout: "example_interfaces.MultiArrayLayout | None" = None
    data: list[float] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Float32MultiArray"
    __hash__: ClassVar[str] = (
        "RIHS01_90e00022805862ae536fa704b14a52b78b9d0aaf0dc905a74247fa65af0e9642"
    )


class String(msgspec.Struct, frozen=True, kw_only=True):
    data: str = ""

    __msgtype__: ClassVar[str] = "example_interfaces/msg/String"
    __hash__: ClassVar[str] = (
        "RIHS01_5509d866a579951f2fc6c19577c32605ba16f308cae7b498341d79536d4eb06b"
    )


class Int8(msgspec.Struct, frozen=True, kw_only=True):
    data: int = 0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/Int8"
    __hash__: ClassVar[str] = (
        "RIHS01_2e9ef643d84ff37840fe787d1269aa06268960e294f4a0f5eb1e9d4eb21cbb57"
    )


class AddTwoIntsRequest(msgspec.Struct, frozen=True, kw_only=True):
    a: int = 0
    b: int = 0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/AddTwoIntsRequest"
    __hash__: ClassVar[str] = (
        "RIHS01_e118de6bf5eeb66a2491b5bda11202e7b68f198d6f67922cf30364858239c81a"
    )


class AddTwoIntsResponse(msgspec.Struct, frozen=True, kw_only=True):
    sum: int = 0

    __msgtype__: ClassVar[str] = "example_interfaces/msg/AddTwoIntsResponse"
    __hash__: ClassVar[str] = (
        "RIHS01_e118de6bf5eeb66a2491b5bda11202e7b68f198d6f67922cf30364858239c81a"
    )


class SetBoolRequest(msgspec.Struct, frozen=True, kw_only=True):
    data: bool = False

    __msgtype__: ClassVar[str] = "example_interfaces/msg/SetBoolRequest"
    __hash__: ClassVar[str] = (
        "RIHS01_a69782e5631b12e15c8e218410de1685bbf13e382718295adad14037a24afbe8"
    )


class SetBoolResponse(msgspec.Struct, frozen=True, kw_only=True):
    success: bool = False
    message: str = ""

    __msgtype__: ClassVar[str] = "example_interfaces/msg/SetBoolResponse"
    __hash__: ClassVar[str] = (
        "RIHS01_a69782e5631b12e15c8e218410de1685bbf13e382718295adad14037a24afbe8"
    )


class TriggerRequest(msgspec.Struct, frozen=True, kw_only=True):
    __msgtype__: ClassVar[str] = "example_interfaces/msg/TriggerRequest"
    __hash__: ClassVar[str] = (
        "RIHS01_cfeeee47f8105dd7685e4c92d46d4074669cb1c477402be1dea37486542a69e0"
    )


class TriggerResponse(msgspec.Struct, frozen=True, kw_only=True):
    success: bool = False
    message: str = ""

    __msgtype__: ClassVar[str] = "example_interfaces/msg/TriggerResponse"
    __hash__: ClassVar[str] = (
        "RIHS01_cfeeee47f8105dd7685e4c92d46d4074669cb1c477402be1dea37486542a69e0"
    )
