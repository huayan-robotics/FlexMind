import enum


class HardwareState(enum.Enum):
    ONLINE = 0
    OFFLINE = 1
    WARNING = 2


class HardwareType(enum.Enum):
    CAMERA = "camera"
    ROBOT = "robot"
    GRIPPER = "gripper"
    REMOTE = "remote" 


MIN_BUTTON_PRESSED_TIME_S = 0.1
MIN_GRIPPER_HELD_TIME_S = 0.5
