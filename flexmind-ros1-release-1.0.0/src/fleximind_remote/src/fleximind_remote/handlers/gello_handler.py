from dataclasses import dataclass
import glob
import os
import rospy
from typing import Optional, Tuple, Sequence

import numpy as np

from fleximind_remote import ArmType
from fleximind_remote.handlers.base import BaseRemoteHandler
from fleximind_remote.gello.agents.agent import BimanualAgent
from fleximind_remote.gello.agents.gello_agent import GelloAgent, DynamixelRobotConfig


@dataclass
class GelloArmConfig:
    port: str = ""
    start_joints: Optional[Tuple[float, ...]] = None
    calibrate_joints: Optional[Tuple[float, ...]] = None
    joint_ids: Optional[Sequence[int]] = None
    joint_offsets: Optional[Tuple[float, ...]] = None
    joint_signs: Optional[Tuple[float, ...]] = None
    gripper_config: Optional[Tuple[int, int, int]] = None
    gripper_angle: Optional[int] = None,


@dataclass
class GelloConfig:
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    hz: int = 100
    arm_type: str = ArmType.BIMANUAL
    gripper_boarder_tolerance: float = 0.1
    gripper_boarder_ratio: float = 1.0

    left_arm: Optional[GelloArmConfig] = None
    right_arm: Optional[GelloArmConfig] = None


class GelloHandler(BaseRemoteHandler):
    JOINT_LIMITS_MIN = [0, 0, 0, 0, 0, 0]
    JOINT_LIMITS_MAX = [255, 255, 255, 255, 255, 255]

    def __init__(self, config: GelloConfig):
        super().__init__()

        self.config = config
        self.arm_type = config.arm_type
        angle_rate = np.pi / 2
        left_joint_offsets = [
            offset * angle_rate for offset in self.config.left_arm.joint_offsets
        ]
        right_joint_offsets = [
            offset * angle_rate for offset in self.config.right_arm.joint_offsets
        ]
        self.left_gello_config = DynamixelRobotConfig(
            self.config.left_arm.joint_ids,
            left_joint_offsets,
            self.config.left_arm.joint_signs,
            self.config.left_arm.gripper_config,
            self.config.left_arm.gripper_angle,
        )
        self.right_gello_config = DynamixelRobotConfig(
            self.config.right_arm.joint_ids,
            right_joint_offsets,
            self.config.right_arm.joint_signs,
            self.config.right_arm.gripper_config,
            self.config.right_arm.gripper_angle,
        )
        self.left_arm_axises = len(self.config.left_arm.joint_ids)
        self.right_arm_axises = len(self.config.right_arm.joint_ids)
        self.left_gripper_id = self.config.left_arm.gripper_config[0]
        self.right_gripper_id = self.config.right_arm.gripper_config[0]
        self.total_axises = self.left_arm_axises + self.right_arm_axises + 2

        self.gripper_boarder_tolerance = (
            config.gripper_boarder_tolerance
        )  # gripper angle diff
        self.gripper_boarder_ratio = (
            config.gripper_boarder_ratio
        )  # gripper cannot push fully
        self.gripper_boarder_range_max = [
            1 - self.gripper_boarder_tolerance,
            1 + self.gripper_boarder_tolerance,
        ]
        self.gripper_boarder_range_min = [0, self.gripper_boarder_tolerance]

        self._agent = None

    @staticmethod
    def _find_available_port():
        """尝试自动寻找可用端口"""
        usb_ports = glob.glob("/dev/serial/by-id/*")
        print(f"Found {len(usb_ports)} ports")
        return usb_ports[0] if usb_ports else None

    def connect(self):
        try:
            if self.config.arm_type == ArmType.BIMANUAL.value:
                if not os.path.exists(self.config.left_arm.port):
                    print(f"左臂端口[{self.config.left_arm.port}]不存在")
                    self._is_connected = False
                    return self._is_connected

                if not os.path.exists(self.config.right_arm.port):
                    print(f"右臂端口[{self.config.right_arm.port}]不存在")
                    self._is_connected = False
                    return self._is_connected

                left_agent = GelloAgent(
                    port=self.config.left_arm.port,
                    dynamixel_config=self.left_gello_config,
                    calibrate_joints=self.config.left_arm.calibrate_joints,
                )
                right_agent = GelloAgent(
                    port=self.config.right_arm.port,
                    dynamixel_config=self.right_gello_config,
                    calibrate_joints=self.config.right_arm.calibrate_joints,
                )
                self._agent = BimanualAgent(left_agent, right_agent)

            elif self.config.arm_type == ArmType.LEFT.value:
                gello_port = self.config.left_arm.port or self._find_available_port()
                if gello_port and os.path.exists(gello_port):
                    self._agent = GelloAgent(
                        port=gello_port,
                        dynamixel_config=self.left_gello_config,
                        calibrate_joints=self.config.left_arm.calibrate_joints,
                    )
                else:
                    print(f"未找到可用的gello USB接口: {gello_port}")
                    self._is_connected = False
                    return self._is_connected

            elif self.config.arm_type == ArmType.RIGHT.value:
                gello_port = self.config.right_arm.port or self._find_available_port()
                if gello_port and os.path.exists(gello_port):
                    self._agent = GelloAgent(
                        port=gello_port,
                        dynamixel_config=self.right_gello_config,
                        calibrate_joints=self.config.right_arm.calibrate_joints,
                    )
                else:
                    print(f"未找到可用的gello USB接口: {gello_port}")
                    self._is_connected = False
                    return self._is_connected

            else:
                print(
                    f"cannot create gello agent, error arm type: {self.config.arm_type}"
                )
                self._is_connected = False
                return self._is_connected

        except Exception as e:
            print(f"cannot create gello agent, error: {e}")
            self._is_connected = False
            return self._is_connected

        self._is_connected = True
        return self._is_connected

    def disconnect(self):
        self._agent = None

    def reconnect(self):
        self._agent = None
        return self.connect()

    def calibrate(self, reset_gripper=False):
        offset_l, offset_r = None, None
        gripper_range_l, gripper_range_r = None, None
        if self.config.arm_type == ArmType.BIMANUAL.value:
            offset_l, offset_r = self._agent.reset_offset()
            if reset_gripper:
                gripper_range_l, gripper_range_r = self._agent.reset_gripper()

        elif self.config.arm_type == ArmType.LEFT.value:
            offset_l = self._agent.reset_offset()
            if reset_gripper:
                gripper_range_l = self._agent.reset_gripper()

        elif self.config.arm_type == ArmType.RIGHT.value:
            offset_r = self._agent.reset_offset()
            if reset_gripper:
                gripper_range_r = self._agent.reset_gripper()

        if offset_l:
            offset_l = [int(np.round(offset / (np.pi / 2))) for offset in offset_l]
        if offset_r:
            offset_r = [int(np.round(offset / (np.pi / 2))) for offset in offset_r]

        print(f"offset_l: {offset_l}")
        print(f"offset_r: {offset_r}")
        print(f"gripper_range_l: {gripper_range_l}")
        print(f"gripper_range_r: {gripper_range_r}")

        return offset_l, offset_r, gripper_range_l, gripper_range_r

    @property
    def is_gripper_held(self):
        return self.check_end_effector_state()

    def check_end_effector_state(self, joints_left=None, joints_right=None) -> bool:
        if joints_left is None or joints_right is None:
            joints_left, joints_right = self.get_joint_angles()

        angles_left = None if joints_left is None else joints_left[-1]
        angles_right = None if joints_right is None else joints_right[-1]

        if self.config.arm_type == ArmType.LEFT.value:
            return (
                angles_left is not None
                and angles_left >= self.gripper_boarder_range_max[0]
                and angles_left <= self.gripper_boarder_range_max[1]
            )

        elif self.config.arm_type == ArmType.RIGHT.value:
            return (
                angles_right is not None
                and angles_right >= self.gripper_boarder_range_max[0]
                and angles_right <= self.gripper_boarder_range_max[1]
            )

        else:
            return (
                angles_left is not None
                and angles_right is not None
                and angles_left >= self.gripper_boarder_range_max[0]
                and angles_left <= self.gripper_boarder_range_max[1]
                and angles_right >= self.gripper_boarder_range_max[0]
                and angles_right <= self.gripper_boarder_range_max[1]
            )

    def get_joint_angles(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if not self.is_connected:
            return (None, None)

        try:
            if self.config.arm_type == ArmType.LEFT.value:
                start_pos = self._agent.act()
                target_joints = np.degrees(start_pos[: self.left_arm_axises]).tolist()
                end_angle = start_pos[-1] / self.gripper_boarder_ratio
                target_joints.append(end_angle)
                return (target_joints, None)

            elif self.config.arm_type == ArmType.RIGHT.value:
                start_pos = self._agent.act()
                target_joints = np.degrees(start_pos[: self.right_arm_axises]).tolist()
                end_angle = start_pos[-1] / self.gripper_boarder_ratio
                target_joints.append(end_angle)
                return (None, target_joints)

            else:
                start_pos = self._agent.act()
                target_joints1 = np.degrees(start_pos[: self.left_arm_axises]).tolist()
                end_angle1 = (
                    start_pos[self.left_arm_axises] / self.gripper_boarder_ratio
                )
                target_joints1.append(end_angle1)
                target_joints2 = np.degrees(
                    start_pos[self.left_arm_axises + 1 : self.total_axises - 1]
                ).tolist()
                end_angle2 = start_pos[-1] / self.gripper_boarder_ratio
                target_joints2.append(end_angle2)
                return (target_joints1, target_joints2)

        except Exception as e:
            print(f"手柄连接可能断开{e}")
            return (None, None)

    def check_joint_limits(self):
        joints_left, joints_right = self.get_joint_angles()

        if self.config.arm_type == ArmType.LEFT.value:
            return all(
                self.JOINT_LIMITS_MIN[i] < joint < self.JOINT_LIMITS_MAX[i]
                for i, joint in enumerate(joints_left)
            )

        elif self.config.arm_type == ArmType.RIGHT.value:
            return all(
                self.JOINT_LIMITS_MIN[i] < joint < self.JOINT_LIMITS_MAX[i]
                for i, joint in enumerate(joints_right)
            )

        else:
            return all(
                self.JOINT_LIMITS_MIN[i] < joint < self.JOINT_LIMITS_MAX[i]
                for i, joint in enumerate(joints_left)
            ) and all(
                self.JOINT_LIMITS_MIN[i] < joint < self.JOINT_LIMITS_MAX[i]
                for i, joint in enumerate(joints_right)
            )

    def enable_left_arm_lock(self):
        """启用左臂自锁（开启扭矩模式）"""
        try:
            if not self._agent:
                rospy.logwarn("agent未初始化")
                return False
            if self.config.arm_type == ArmType.BIMANUAL.value:
                if hasattr(self._agent, 'agent_left') and self._agent.agent_left:
                    if hasattr(self._agent.agent_left, '_robot'):
                        robot = self._agent.agent_left._robot
                        robot.set_torque_mode(True)
                        return True
                    else:
                        rospy.logwarn("agent_left没有_robot属性")
                else:
                    rospy.logwarn("agent_left不存在或未初始化")
            elif self.config.arm_type == ArmType.LEFT.value:
                if hasattr(self._agent, '_robot'):
                    self._agent._robot.set_torque_mode(True)
                    return True
                else:
                    rospy.logwarn("单左臂agent没有_robot属性")
            else:
                rospy.logwarn(f"左臂自锁不适用于当前臂型: {self.config.arm_type}")
                
            rospy.logwarn("左臂自锁启用失败")
            return False
        except Exception as e:
            rospy.logerr(f"启用左臂自锁失败: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return False

    def enable_right_arm_lock(self):
        """启用右臂自锁（开启扭矩模式）"""
        try:
            if not self._agent:
                rospy.logwarn("agent未初始化")
                return False
            if self.config.arm_type == ArmType.BIMANUAL.value:
                if hasattr(self._agent, 'agent_right') and self._agent.agent_right:
                    if hasattr(self._agent.agent_right, '_robot'):
                        robot = self._agent.agent_right._robot
                        robot.set_torque_mode(True)
                        return True
                    else:
                        rospy.logwarn("agent_right没有_robot属性")
                else:
                    rospy.logwarn("agent_right不存在或未初始化")
            elif self.config.arm_type == ArmType.RIGHT.value:
                if hasattr(self._agent, '_robot'):
                    self._agent._robot.set_torque_mode(True)
                    return True
                else:
                    rospy.logwarn("单右臂agent没有_robot属性")
            else:
                rospy.logwarn(f"右臂自锁不适用于当前臂型: {self.config.arm_type}")
            rospy.logwarn("右臂自锁启用失败")
            return False
        except Exception as e:
            rospy.logerr(f"启用右臂自锁失败: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return False

    def disable_left_arm_lock(self):
        """禁用左臂自锁（关闭扭矩模式）"""
        try:
            if not self._agent:
                rospy.logwarn("agent未初始化")
                return False
            if self.config.arm_type == ArmType.BIMANUAL.value:
                if hasattr(self._agent, 'agent_left') and self._agent.agent_left:
                    if hasattr(self._agent.agent_left, '_robot'):
                        robot = self._agent.agent_left._robot
                        robot.set_torque_mode(False)
                        return True
                    else:
                        rospy.logwarn("agent_left没有_robot属性")
                else:
                    rospy.logwarn("agent_left不存在或未初始化")
            elif self.config.arm_type == ArmType.LEFT.value:
                if hasattr(self._agent, '_robot'):
                    self._agent._robot.set_torque_mode(False)
                    return True
                else:
                    rospy.logwarn("单左臂agent没有_robot属性")
            else:
                rospy.logwarn(f"左臂自锁不适用于当前臂型: {self.config.arm_type}")
                
            rospy.logwarn("左臂自锁禁用失败")
            return False
        except Exception as e:
            rospy.logerr(f"禁用左臂自锁失败: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return False

    def disable_right_arm_lock(self):
        """禁用右臂自锁（关闭扭矩模式）"""
        try:
            if not self._agent:
                rospy.logwarn("agent未初始化")
                return False
            if self.config.arm_type == ArmType.BIMANUAL.value:
                if hasattr(self._agent, 'agent_right') and self._agent.agent_right:
                    if hasattr(self._agent.agent_right, '_robot'):
                        robot = self._agent.agent_right._robot
                        robot.set_torque_mode(False)
                        return True
                    else:
                        rospy.logwarn("agent_right没有_robot属性")
                else:
                    rospy.logwarn("agent_right不存在或未初始化")
                    
            elif self.config.arm_type == ArmType.RIGHT.value:
                if hasattr(self._agent, '_robot'):
                    self._agent._robot.set_torque_mode(False)
                    return True
                else:
                    rospy.logwarn("单右臂agent没有_robot属性")
            else:
                rospy.logwarn(f"右臂自锁不适用于当前臂型: {self.config.arm_type}")
                
            rospy.logwarn("右臂自锁禁用失败")
            return False
        except Exception as e:
            rospy.logerr(f"禁用右臂自锁失败: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return False