import enum


class HardwareInterface(enum.Enum):
    STATUS = "/system/hardware/status"
    POSE_ERROR = "/system/sync/pose_error"

    CAMERA_STATUS = "/system/cameras/status"
    ROBOT_STATUS = "/system/robots/status"
    GRIPPER_STATUS = "/system/grippers/status"


class RemoteControlInterface(enum.Enum):
    STATUS = "/remote_control/status"
    JOINT_STATES = "/remote_control/joint_states"

    CALIBRATE = "/remote_control/calibrate"
    GELLO_LOCK_CONTROL = "/gello/lock_control"


class RemoteSystemInterface(enum.Enum):
    STATUS = "/remote_system/status"
    IO_STATE = "/remote_system/io_state"


class RobotInterface(enum.Enum):
    # 话题
    STATUS = "robot/state"
    INFO = "robot/info"
    DATA_ACTION_LEFT_ARM = "/robot/data/action/left_arm"
    DATA_ACTION_RIGHT_ARM = "/robot/data/action/right_arm"
    OBSERVATION_ACTION_LEFT_ARM = "robot/data/observation/left_arm"
    OBSERVATION_ACTION_RIGHT_ARM = "robot/data/observation/right_arm"
    DUAL_ROBOT_SYSTEM_STATE = "/dual_robot_system_state"

    # 服务
    GET_IP = "/robot/get_ip"
    RESET = "/robot/reset"
    EMERGENCY_STOP = "/robot/emergency_stop"
    START = "/robot/start"
    POWER_OUTAGE = "/robot/power_outage"
    ENABLE = "/robot/enable"
    DISABLE = "/robot/disable"
    SERVOJ = "/robot/servoj"
    DATA_PUB = "/robot/data_publishing_switch"
    SETSPEED = "/robot/setspeed"

    # 动作
    MOVE_JOINTS = "/robot/move_joints"


class GripperInterface(enum.Enum):
    # 话题
    STATUS = "gripper/state"
    INFO = "gripper/info"

    # 服务
    GRIPPER_DATA_PUB = "/gripper/data_publishing_switch"


class ManagingConfig(enum.Enum):
    # 服务
    CONFIG = "/config_manager/service"


class MainInteracting(enum.Enum):
    # 服务
    VERSION = "/version"
    GET_PLAYBACK_LIST = "/playback_list/get"
    EXPORT_PLAYBACK_FILES = "/playback_files/export"
    IMPORT_PLAYBACK_FILES = "/playback_files/import"
    DELETE_PLAYBACK = "/playback/delete"


class RecordServer(enum.Enum):
    # 话题
    RECORDING_STATUS_DETAIL = "/record_bag/recording_status_detail"
    RECORDING_STATE = "/record_bag/recording_state"
