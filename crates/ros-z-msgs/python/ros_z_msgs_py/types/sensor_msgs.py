"""Auto-generated ROS 2 message types for sensor_msgs."""

import msgspec
from typing import ClassVar


class LaserScan(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    angle_min: float = 0.0
    angle_max: float = 0.0
    angle_increment: float = 0.0
    time_increment: float = 0.0
    scan_time: float = 0.0
    range_min: float = 0.0
    range_max: float = 0.0
    ranges: list[float] = msgspec.field(default_factory=list)
    intensities: list[float] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/LaserScan"
    __hash__: ClassVar[str] = (
        "RIHS01_cb82473c260a6312a7ce1e007a94c33bbd1d165c31d36399f9133797fdac203b"
    )


class NavSatFix(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    status: "sensor_msgs.NavSatStatus | None" = None
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    position_covariance: list[float] = msgspec.field(default_factory=list)
    position_covariance_type: int = 0

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/NavSatFix"
    __hash__: ClassVar[str] = (
        "RIHS01_30b984610e2c08bcf6bc4dbad70b89e2877a3b7f46c38df8c5331df4dcd7fd3f"
    )


class PointCloud(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    points: list["geometry_msgs.Point32"] = msgspec.field(default_factory=list)
    channels: list["sensor_msgs.ChannelFloat32"] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/PointCloud"
    __hash__: ClassVar[str] = (
        "RIHS01_6d3bd53b7ae4273e395407770c00820365dec494a3d3f6ff7fcce7002f11b026"
    )


class NavSatStatus(msgspec.Struct, frozen=True, kw_only=True):
    status: int = 0
    service: int = 0

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/NavSatStatus"
    __hash__: ClassVar[str] = (
        "RIHS01_d1ed3befa628e09571bd273b888ba1c1fd187c9a5e0006b385d7e5e9095a3204"
    )


class PointCloud2(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    height: int = 0
    width: int = 0
    fields: list["sensor_msgs.PointField"] = msgspec.field(default_factory=list)
    is_bigendian: bool = False
    point_step: int = 0
    row_step: int = 0
    data: bytes = b""
    is_dense: bool = False

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/PointCloud2"
    __hash__: ClassVar[str] = (
        "RIHS01_31b92e5b6825ae0153d372edd4f8c26e7ac5e41cff7083e71f56c043521ad160"
    )


class Imu(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    orientation: "geometry_msgs.Quaternion | None" = None
    orientation_covariance: list[float] = msgspec.field(default_factory=list)
    angular_velocity: "geometry_msgs.Vector3 | None" = None
    angular_velocity_covariance: list[float] = msgspec.field(default_factory=list)
    linear_acceleration: "geometry_msgs.Vector3 | None" = None
    linear_acceleration_covariance: list[float] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/Imu"
    __hash__: ClassVar[str] = (
        "RIHS01_f263b3ba17eff3ab102ac7a84e87d2ba9e7bb6948e6b7a2d1a894080fb5ecdf3"
    )


class FluidPressure(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    fluid_pressure: float = 0.0
    variance: float = 0.0

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/FluidPressure"
    __hash__: ClassVar[str] = (
        "RIHS01_5ab249ecaa9b0bb0948356bc7e3293454cc31a094b8f9b4536b617e4443e7e0e"
    )


class Illuminance(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    illuminance: float = 0.0
    variance: float = 0.0

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/Illuminance"
    __hash__: ClassVar[str] = (
        "RIHS01_6c5adcdd896f69f00ded89d2d9dab73b99ca9d4d628e53a027b191b8cb8a393c"
    )


class RegionOfInterest(msgspec.Struct, frozen=True, kw_only=True):
    x_offset: int = 0
    y_offset: int = 0
    height: int = 0
    width: int = 0
    do_rectify: bool = False

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/RegionOfInterest"
    __hash__: ClassVar[str] = (
        "RIHS01_ad16bcba5f9131dcdba6fbded19f726f5440e3c513b4fb586dd3027eeed8abb1"
    )


class LaserEcho(msgspec.Struct, frozen=True, kw_only=True):
    echoes: list[float] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/LaserEcho"
    __hash__: ClassVar[str] = (
        "RIHS01_0fbc05a0db7d37fe52c0f0375356db55da0046f7ef5bd27ca6b34bd0582bc952"
    )


class PointField(msgspec.Struct, frozen=True, kw_only=True):
    name: str = ""
    offset: int = 0
    datatype: int = 0
    count: int = 0

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/PointField"
    __hash__: ClassVar[str] = (
        "RIHS01_5c6a4750728c2bcfbbf7037225b20b02d4429634732146b742dee1726637ef01"
    )


class CompressedImage(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    format: str = ""
    data: bytes = b""

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/CompressedImage"
    __hash__: ClassVar[str] = (
        "RIHS01_03b59359fb204d5777d592494651ba5c28015400b5e6960434ff55277c88fa02"
    )


class Joy(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    axes: list[float] = msgspec.field(default_factory=list)
    buttons: list[int] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/Joy"
    __hash__: ClassVar[str] = (
        "RIHS01_0f600b67564ac107758b77fa13dfe34efa34e2cce2c79fcacd5827ddbe763ad8"
    )


class JoyFeedbackArray(msgspec.Struct, frozen=True, kw_only=True):
    array: list["sensor_msgs.JoyFeedback"] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/JoyFeedbackArray"
    __hash__: ClassVar[str] = (
        "RIHS01_a1898a9c9cfd705138af9afd823fef8f627bf2a450d9ab7af258d482677d64bf"
    )


class TimeReference(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    time_ref: "builtin_interfaces.Time | None" = msgspec.field(
        default_factory=lambda: {"sec": 0, "nanosec": 0}
    )
    source: str = ""

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/TimeReference"
    __hash__: ClassVar[str] = (
        "RIHS01_dd66e84cf40bbb5d5a40472e6ecf2675a031334d4c426abdb2ad41801a8efc99"
    )


class Range(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    radiation_type: int = 0
    field_of_view: float = 0.0
    min_range: float = 0.0
    max_range: float = 0.0
    range: float = 0.0
    variance: float = 0.0

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/Range"
    __hash__: ClassVar[str] = (
        "RIHS01_dc560f04b2652e0f4857f59067fac610bc6bec6f1e79ed52909afa139a4629ad"
    )


class ChannelFloat32(msgspec.Struct, frozen=True, kw_only=True):
    name: str = ""
    values: list[float] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/ChannelFloat32"
    __hash__: ClassVar[str] = (
        "RIHS01_92665437ddf39346f4ba39ee32e648390605b633cc077d40f4bd4d7b58af6cd4"
    )


class Image(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    height: int = 0
    width: int = 0
    encoding: str = ""
    is_bigendian: int = 0
    step: int = 0
    data: bytes = b""

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/Image"
    __hash__: ClassVar[str] = (
        "RIHS01_9bfd6e372ca6f9ad3c87a61be15ea49b3b465dcfdaf40be22988badc3c36ce7c"
    )


class JoyFeedback(msgspec.Struct, frozen=True, kw_only=True):
    type: int = 0
    id: int = 0
    intensity: float = 0.0

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/JoyFeedback"
    __hash__: ClassVar[str] = (
        "RIHS01_231dd362f71d6fc08272770d07120ad5fe5874ce2dbac70109b28986834290cd"
    )


class RelativeHumidity(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    relative_humidity: float = 0.0
    variance: float = 0.0

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/RelativeHumidity"
    __hash__: ClassVar[str] = (
        "RIHS01_7994dfdd1bc463a9cab06d073aa51f420a5cc14144ebf0bab6e851e5291a601c"
    )


class MultiEchoLaserScan(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    angle_min: float = 0.0
    angle_max: float = 0.0
    angle_increment: float = 0.0
    time_increment: float = 0.0
    scan_time: float = 0.0
    range_min: float = 0.0
    range_max: float = 0.0
    ranges: list["sensor_msgs.LaserEcho"] = msgspec.field(default_factory=list)
    intensities: list["sensor_msgs.LaserEcho"] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/MultiEchoLaserScan"
    __hash__: ClassVar[str] = (
        "RIHS01_9bd5c655b71dd44daa283669a0c9e76ac51338f520e1c131a25182cee9c07d97"
    )


class JointState(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    name: list[str] = msgspec.field(default_factory=list)
    position: list[float] = msgspec.field(default_factory=list)
    velocity: list[float] = msgspec.field(default_factory=list)
    effort: list[float] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/JointState"
    __hash__: ClassVar[str] = (
        "RIHS01_1e29e91a849228ded7e9e811f1d7a0ecb2c989dbd2e35b4bdb1c9793dbb510f1"
    )


class CameraInfo(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    height: int = 0
    width: int = 0
    distortion_model: str = ""
    d: list[float] = msgspec.field(default_factory=list)
    k: list[float] = msgspec.field(default_factory=list)
    r: list[float] = msgspec.field(default_factory=list)
    p: list[float] = msgspec.field(default_factory=list)
    binning_x: int = 0
    binning_y: int = 0
    roi: "sensor_msgs.RegionOfInterest | None" = None

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/CameraInfo"
    __hash__: ClassVar[str] = (
        "RIHS01_beafebb50eec7586aacedcd0eb1b7bbf86c8f15a3bbe175bf821144e46682f10"
    )


class MultiDOFJointState(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    joint_names: list[str] = msgspec.field(default_factory=list)
    transforms: list["geometry_msgs.Transform"] = msgspec.field(default_factory=list)
    twist: list["geometry_msgs.Twist"] = msgspec.field(default_factory=list)
    wrench: list["geometry_msgs.Wrench"] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/MultiDOFJointState"
    __hash__: ClassVar[str] = (
        "RIHS01_0cf7222e569d28b7192cd90988ed16e3d449d1abae1b4827a60657289e33f526"
    )


class Temperature(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    temperature: float = 0.0
    variance: float = 0.0

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/Temperature"
    __hash__: ClassVar[str] = (
        "RIHS01_2be361ed1a2a1314e8b55c49baccfdd6e50ce3e2b3252b4b8ab4b2956f4921ae"
    )


class BatteryState(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    voltage: float = 0.0
    temperature: float = 0.0
    current: float = 0.0
    charge: float = 0.0
    capacity: float = 0.0
    design_capacity: float = 0.0
    percentage: float = 0.0
    power_supply_status: int = 0
    power_supply_health: int = 0
    power_supply_technology: int = 0
    present: bool = False
    cell_voltage: list[float] = msgspec.field(default_factory=list)
    cell_temperature: list[float] = msgspec.field(default_factory=list)
    location: str = ""
    serial_number: str = ""

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/BatteryState"
    __hash__: ClassVar[str] = (
        "RIHS01_bfb69e99942433a64000b7ed80a374f110d40e288f8da4c80bde8cd8860ca187"
    )


class MagneticField(msgspec.Struct, frozen=True, kw_only=True):
    header: "std_msgs.Header | None" = None
    magnetic_field: "geometry_msgs.Vector3 | None" = None
    magnetic_field_covariance: list[float] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/MagneticField"
    __hash__: ClassVar[str] = (
        "RIHS01_7b9c1e26e1d58172e0f14de56c765b30c280ea1c7362667256b9c8ee89b9df56"
    )


class SetCameraInfoRequest(msgspec.Struct, frozen=True, kw_only=True):
    camera_info: "sensor_msgs.CameraInfo | None" = None

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/SetCameraInfoRequest"
    __hash__: ClassVar[str] = (
        "RIHS01_bd533dbb08795c76f5b32ce79f98fb87367f82f03903ddf4a38efafc9149a231"
    )


class SetCameraInfoResponse(msgspec.Struct, frozen=True, kw_only=True):
    success: bool = False
    status_message: str = ""

    __msgtype__: ClassVar[str] = "sensor_msgs/msg/SetCameraInfoResponse"
    __hash__: ClassVar[str] = (
        "RIHS01_bd533dbb08795c76f5b32ce79f98fb87367f82f03903ddf4a38efafc9149a231"
    )
