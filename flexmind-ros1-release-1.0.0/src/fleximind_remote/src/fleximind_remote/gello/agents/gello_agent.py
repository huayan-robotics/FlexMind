import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from fleximind_remote.gello.agents.agent import Agent
from fleximind_remote.gello.robots.dynamixel import DynamixelRobot


@dataclass
class DynamixelRobotConfig:
    joint_ids: Sequence[int]
    """The joint ids of GELLO (not including the gripper). Usually (1, 2, 3 ...)."""

    joint_offsets: Sequence[float]
    """The joint offsets of GELLO. There needs to be a joint offset for each joint_id and should be a multiple of pi/2."""

    joint_signs: Sequence[int]
    """The joint signs of GELLO. There needs to be a joint sign for each joint_id and should be either 1 or -1.

    This will be different for each arm design. Refernce the examples below for the correct signs for your robot.
    """

    gripper_config: Tuple[int, int, int]
    """The gripper config of GELLO. This is a tuple of (gripper_joint_id, degrees in open_position, degrees in closed_position)."""

    gripper_angle: Optional[int]
    """The gripper close angle of GELLO."""

    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_offsets)
        assert len(self.joint_ids) == len(self.joint_signs)

    def make_robot(
        self,
        port: str = "/dev/ttyUSB0",
        start_joints: Optional[np.ndarray] = None,
        calibrate_joints: Optional[np.ndarray] = None,
    ) -> DynamixelRobot:
        return DynamixelRobot(
            joint_ids=self.joint_ids,
            joint_offsets=list(self.joint_offsets),
            real=True,
            joint_signs=list(self.joint_signs),
            port=port,
            gripper_config=self.gripper_config,
            gripper_angle=self.gripper_angle,
            start_joints=start_joints,
            calibrate_joints=calibrate_joints,
        )


class GelloAgent(Agent):

    def __init__(
        self,
        port: str,
        dynamixel_config: DynamixelRobotConfig,
        start_joints: Optional[np.ndarray] = None,
        calibrate_joints: Optional[np.ndarray] = None,
    ):
        if dynamixel_config is None:
            raise ValueError("[dynamixel_config] should not be None!!!")

        self._robot = dynamixel_config.make_robot(
            port=port, start_joints=start_joints, calibrate_joints=calibrate_joints
        )

    def act(self) -> np.ndarray:
        try:
            return self._robot.get_joint_state()
        except Exception as e:
            print(f"手柄读取角度出现错误{e}")
            return None

    def reset_offset(self):
        try:
            return self._robot.calculate_offsets(reload=True)
        except Exception as e:
            print(f"手柄矫正角度出现错误{e}")
            return None

    def reset_gripper(self):
        try:
            return self._robot.get_gripper_calibration(reload=True)
        except Exception as e:
            print(f"手柄矫正夹爪角度出现错误{e}")
            return None
