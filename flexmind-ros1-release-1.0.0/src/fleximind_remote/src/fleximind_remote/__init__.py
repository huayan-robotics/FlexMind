from .gello import *

import enum
from fleximind_remote.gello.robots.CPS import RbtFSM

class ControlType(enum.Enum):
    REAL = "real"
    REMOTE = "remote"


class ArmType(enum.Enum):
    LEFT = "left"
    RIGHT = "right"
    BIMANUAL = "bimanual"


class SystemState(enum.Enum):
    """
    系统状态枚举类
    """

    UNINITIALIZED = 0  # 未初始化
    STANDBY = 1  # 待机
    SYNCING = 2  # 同步中
    SYNCED = 3  # 已同步
    FOLLOWING = 4  # 跟随中
    SOFTWARE_EXCEPTION = 5  # 软件异常
    HARDWARE_EXCEPTION = 6  # 硬件异常
    EMERGENCY_STOP = 7  # 急停


class SoftwareFault(enum.Enum):
    UNEXPECTED = "Unexpected"
    COLLISION = "Collision"
    FOLLOW_ERROR = "FollowError"


SYSTEM_STATES = [state.name for state in SystemState]
GELLO_GRIPPER_BORDER = 253

DEFAULT_ROBOT_PORT = 10003
HIGH_SPEED_ROBOT_PORT = 8892

DEFAULT_CALIBRATE_PRESSED_TIME_S = 5

ROBOT_ERROR_MAP = [
    RbtFSM.enCPSState_Error,
    RbtFSM.enCPSState_HRAppError,
    RbtFSM.enCPSState_SafetyGuardError,
    RbtFSM.enCPSState_EtherCATError,
    RbtFSM.enCPSState_ControllerVersionError,
    RbtFSM.enCPSState_RobotOutofSafeSpace,
    RbtFSM.enCPSState_RobotCollisionStop,
]