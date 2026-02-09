"""Auto-generated ROS 2 message types for action_msgs."""

import msgspec
from typing import ClassVar


class GoalStatus(msgspec.Struct, frozen=True, kw_only=True):
    goal_info: "action_msgs.GoalInfo | None" = None
    status: int = 0

    __msgtype__: ClassVar[str] = "action_msgs/msg/GoalStatus"
    __hash__: ClassVar[str] = (
        "RIHS01_6e21370c4382c09860cbdc5089cda1fe5b802bdbd56d60fe6cfd79c12a2ec048"
    )


class GoalInfo(msgspec.Struct, frozen=True, kw_only=True):
    goal_id: "unique_identifier_msgs.UUID | None" = None
    stamp: "builtin_interfaces.Time | None" = msgspec.field(
        default_factory=lambda: {"sec": 0, "nanosec": 0}
    )

    __msgtype__: ClassVar[str] = "action_msgs/msg/GoalInfo"
    __hash__: ClassVar[str] = (
        "RIHS01_6398fe763154554353930716b225947f93b672f0fb2e49fdd01bb7a7e37933e9"
    )


class GoalStatusArray(msgspec.Struct, frozen=True, kw_only=True):
    status_list: list["action_msgs.GoalStatus"] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "action_msgs/msg/GoalStatusArray"
    __hash__: ClassVar[str] = (
        "RIHS01_49d3f45fe30c84e4dd2eadb8e5125cc71dcc4e31b3f0363afd4f794fde652e27"
    )


class CancelGoalRequest(msgspec.Struct, frozen=True, kw_only=True):
    goal_info: "action_msgs.GoalInfo | None" = None

    __msgtype__: ClassVar[str] = "action_msgs/msg/CancelGoalRequest"
    __hash__: ClassVar[str] = (
        "RIHS01_0894e514b8d27fc42bb40ca22eb8c44da1c0f30a2973590151bb20958c4d312e"
    )


class CancelGoalResponse(msgspec.Struct, frozen=True, kw_only=True):
    return_code: int = 0
    goals_canceling: list["action_msgs.GoalInfo"] = msgspec.field(default_factory=list)

    __msgtype__: ClassVar[str] = "action_msgs/msg/CancelGoalResponse"
    __hash__: ClassVar[str] = (
        "RIHS01_0894e514b8d27fc42bb40ca22eb8c44da1c0f30a2973590151bb20958c4d312e"
    )
