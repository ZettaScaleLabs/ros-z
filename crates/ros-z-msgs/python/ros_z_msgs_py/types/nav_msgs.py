"""Auto-generated ROS 2 message types for nav_msgs."""

import msgspec
from typing import ClassVar


class Odometry(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    child_frame_id: str = ""
    pose: "geometry_msgs.PoseWithCovariance | None" = None
    twist: "geometry_msgs.TwistWithCovariance | None" = None

    __msgtype__: ClassVar[str] = "nav_msgs/msg/Odometry"
    __hash__: ClassVar[str] = (
        "RIHS01_63c153364853f6a0d424a418b3a755be2ab74900eaee0eb5a8548f2e219ca2c6"
    )


class MapMetaData(msgspec.Struct, frozen=True, kw_only=True):
    map_load_time: "builtin_interfaces.Time | None" = msgspec.field(
        default_factory=lambda: {"sec": 0, "nanosec": 0}
    )
    resolution: float = 0.0
    width: int = 0
    height: int = 0
    origin: "geometry_msgs.Pose | None" = None

    __msgtype__: ClassVar[str] = "nav_msgs/msg/MapMetaData"
    __hash__: ClassVar[str] = (
        "RIHS01_4f66ea516967c81622d5355d949fb3444634317b2ba783ef98a61e1525547e7c"
    )


class Path(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    poses: list["geometry_msgs.PoseStamped"] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "nav_msgs/msg/Path"
    __hash__: ClassVar[str] = (
        "RIHS01_532e368df23803690d8b7bfd257440feb8de6ecbd25f1c49792b81bd7f9547ad"
    )


class OccupancyGrid(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    info: "nav_msgs.MapMetaData | None" = None
    data: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "nav_msgs/msg/OccupancyGrid"
    __hash__: ClassVar[str] = (
        "RIHS01_e1a43cb368791bc5d7b2d98dda0cec8552b588c362294f19b13bf0c9e3694daf"
    )


class GridCells(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    cell_width: float = 0.0
    cell_height: float = 0.0
    cells: list["geometry_msgs.Point"] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "nav_msgs/msg/GridCells"
    __hash__: ClassVar[str] = (
        "RIHS01_408bcf8ca6913f4563fcd9c3e49e4ce508fc936c0919520632b4ed4d3691e776"
    )


class Goals(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    goals: list["geometry_msgs.PoseStamped"] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "nav_msgs/msg/Goals"
    __hash__: ClassVar[str] = (
        "RIHS01_494384ce892a8f012158c5e1aedcec340aedde346f11455d4448f82ba66e3d92"
    )


class GetMapRequest(msgspec.Struct, frozen=True, kw_only=True):
    __msgtype__: ClassVar[str] = "nav_msgs/msg/GetMapRequest"
    __hash__: ClassVar[str] = (
        "RIHS01_7b4e570e6bfa63812c8b5fb61fb05e4f86b5396987bbc42ad9d0cb0399469051"
    )


class GetMapResponse(msgspec.Struct, frozen=True, kw_only=True):
    map: "nav_msgs.OccupancyGrid | None" = None

    __msgtype__: ClassVar[str] = "nav_msgs/msg/GetMapResponse"
    __hash__: ClassVar[str] = (
        "RIHS01_7b4e570e6bfa63812c8b5fb61fb05e4f86b5396987bbc42ad9d0cb0399469051"
    )


class GetPlanRequest(msgspec.Struct, frozen=True, kw_only=True):
    start: "geometry_msgs.PoseStamped | None" = None
    goal: "geometry_msgs.PoseStamped | None" = None
    tolerance: float = 0.0

    __msgtype__: ClassVar[str] = "nav_msgs/msg/GetPlanRequest"
    __hash__: ClassVar[str] = (
        "RIHS01_1b391672216834b6dfe97529cec4931bfb6e7a5207b58af297dcd110af612520"
    )


class GetPlanResponse(msgspec.Struct, frozen=True, kw_only=True):
    plan: "nav_msgs.Path | None" = None

    __msgtype__: ClassVar[str] = "nav_msgs/msg/GetPlanResponse"
    __hash__: ClassVar[str] = (
        "RIHS01_1b391672216834b6dfe97529cec4931bfb6e7a5207b58af297dcd110af612520"
    )


class LoadMapRequest(msgspec.Struct, frozen=True, kw_only=True):
    map_url: str = ""

    __msgtype__: ClassVar[str] = "nav_msgs/msg/LoadMapRequest"
    __hash__: ClassVar[str] = (
        "RIHS01_55246818dc8b5b2af0de20f811994cf1b574f5c277d1c9c965f5800003eed16f"
    )


class LoadMapResponse(msgspec.Struct, frozen=True, kw_only=True):
    map: "nav_msgs.OccupancyGrid | None" = None
    result: int = 0

    __msgtype__: ClassVar[str] = "nav_msgs/msg/LoadMapResponse"
    __hash__: ClassVar[str] = (
        "RIHS01_55246818dc8b5b2af0de20f811994cf1b574f5c277d1c9c965f5800003eed16f"
    )


class SetMapRequest(msgspec.Struct, frozen=True, kw_only=True):
    map: "nav_msgs.OccupancyGrid | None" = None
    initial_pose: "geometry_msgs.PoseWithCovarianceStamped | None" = None

    __msgtype__: ClassVar[str] = "nav_msgs/msg/SetMapRequest"
    __hash__: ClassVar[str] = (
        "RIHS01_1bfd4f0da52b091b41cf85ad2111a0e2ac90b619b74716a1a2a6b37258516440"
    )


class SetMapResponse(msgspec.Struct, frozen=True, kw_only=True):
    success: bool = False

    __msgtype__: ClassVar[str] = "nav_msgs/msg/SetMapResponse"
    __hash__: ClassVar[str] = (
        "RIHS01_1bfd4f0da52b091b41cf85ad2111a0e2ac90b619b74716a1a2a6b37258516440"
    )
