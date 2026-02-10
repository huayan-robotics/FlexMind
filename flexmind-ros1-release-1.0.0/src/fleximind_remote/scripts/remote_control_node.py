#!/usr/bin/env python
import os
from typing import Optional

import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from std_srvs.srv import Trigger, TriggerResponse

from fleximind_bringup import DEFAULT_RECONNECT_TIME_S
from fleximind_bringup.config_utils import load_config, save_config
from fleximind_bringup.interface import RemoteControlInterface
from fleximind_hardware.msg import DualJointStates
from fleximind_remote import ArmType
from fleximind_remote.handlers.gello_handler import (
    GelloArmConfig,
    GelloConfig,
    GelloHandler,
)
from fleximind_remote.msg import LockStatus, RemoteControlStatus
from fleximind_remote.srv import LockControl, LockControlResponse


class RemoteControlNode:
    def __init__(self):
        # ROS1节点初始化
        rospy.init_node("remote_control")

        # 获取包路径
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path("fleximind_remote")
        self.config_file = os.path.join(self.package_path, "config", "remote.yaml")

        # 加载配置参数
        self.remote_config = self.load_config()
        self.remote_controller = GelloHandler(self.remote_config)

        self._timer_period_s = 0.03333
        self._publish_queue_size = 10

        # 自锁状态
        self._lock_enabled = False

        # 初始化发布器
        self.joints_publisher = rospy.Publisher(
            RemoteControlInterface.JOINT_STATES.value,
            DualJointStates,
            queue_size=self._publish_queue_size,
        )

        self.status_publisher = rospy.Publisher(
            RemoteControlInterface.STATUS.value,
            RemoteControlStatus,
            queue_size=self._publish_queue_size,
        )

        # 自锁状态发布器
        self.lock_status_publisher = rospy.Publisher(
            "/gello/lock_status",
            LockStatus,
            queue_size=self._publish_queue_size,
        )

        # 定时器回调
        self.timer = rospy.Timer(
            rospy.Duration(self._timer_period_s), self.publish_status
        )

        # 定时检查连接状态
        rospy.Timer(rospy.Duration(DEFAULT_RECONNECT_TIME_S), self._check_connection)

        # 校准服务
        self.calibrate_service = rospy.Service(
            RemoteControlInterface.CALIBRATE.value,
            Trigger,
            self._handle_calibrate_service,
        )

        # 自锁控制服务
        self.lock_service = rospy.Service(
            RemoteControlInterface.GELLO_LOCK_CONTROL.value,
            LockControl,
            self._handle_lock_control,
        )

        self.calibrate_gripper = rospy.get_param("remote_control")["gripper_reset"]

        rospy.loginfo("RemoteControlNode initialized")

    def _check_connection(self, event):
        if self.remote_controller and not self.remote_controller.is_connected:
            self.remote_controller.connect()

    def _handle_lock_control(self, req):
        """处理自锁控制服务请求"""
        try:
            rospy.loginfo(f"收到自锁切换请求，当前状态: {self._lock_enabled}")

            # 切换自锁状态
            if self._lock_enabled:
                success = self._disable_lock()
                message = "禁用双臂自锁"
                new_state = False
            else:
                success = self._enable_lock()
                message = "启用双臂自锁"
                new_state = True

            # 更新内部状态（即使操作失败也更新，保持一致性）
            self._lock_enabled = new_state

            rospy.loginfo(f"自锁状态切换完成: {message}, 新状态: {new_state}")

            # 发布最新的自锁状态
            self._publish_lock_status()

            # 总是返回成功，让前端能够更新UI状态
            return LockControlResponse(success=True, message=message, locked=new_state)

        except Exception as e:
            rospy.logerr(f"自锁控制错误: {str(e)}")
            import traceback

            rospy.logerr(traceback.format_exc())
            return LockControlResponse(
                success=False, message=str(e), locked=self._lock_enabled
            )

    def _enable_lock(self):
        """启用双臂自锁"""
        try:
            # 启用双臂自锁
            left_success = self.remote_controller.enable_left_arm_lock()
            right_success = self.remote_controller.enable_right_arm_lock()

            rospy.loginfo(
                f"左臂自锁结果: {left_success}, 右臂自锁结果: {right_success}"
            )

            if left_success and right_success:
                self._lock_enabled = True
                return True
            else:
                rospy.logwarn(
                    f"启用自锁失败: 左臂={left_success}, 右臂={right_success}"
                )
                return False
        except Exception as e:
            rospy.logerr(f"启用自锁失败: {str(e)}")
            import traceback

            rospy.logerr(traceback.format_exc())
            return False

    def _disable_lock(self):
        """禁用双臂自锁"""
        try:
            left_success = self.remote_controller.disable_left_arm_lock()
            right_success = self.remote_controller.disable_right_arm_lock()

            rospy.loginfo(
                f"左臂解锁结果: {left_success}, 右臂解锁结果: {right_success}"
            )

            if left_success and right_success:
                self._lock_enabled = False
                return True
            else:
                rospy.logwarn(
                    f"禁用自锁失败: 左臂={left_success}, 右臂={right_success}"
                )
                return False
        except Exception as e:
            rospy.logerr(f"禁用自锁失败: {str(e)}")
            import traceback

            rospy.logerr(traceback.format_exc())
            return False

    def _publish_lock_status(self):
        """发布自锁状态"""
        try:
            # 发布详细状态
            lock_status = LockStatus()
            lock_status.enabled = self._lock_enabled
            self.lock_status_publisher.publish(lock_status)

        except Exception as e:
            rospy.logerr(f"发布自锁状态错误: {str(e)}")

    def _handle_calibrate_service(self, req):
        """处理校准服务请求"""
        try:
            # 在校准前禁用自锁，确保可以自由移动
            if self._lock_enabled:
                self._disable_lock()
                rospy.loginfo("校准前自动禁用自锁")

            offset_l, offset_r, gripper_range_l, gripper_range_r = (
                self.remote_controller.calibrate(reset_gripper=self.calibrate_gripper)
            )

            config = load_config(self.config_file)
            if self.calibrate_gripper:
                if offset_l:
                    config["left_arm"]["joint_offsets"] = offset_l
                if offset_l:
                    config["right_arm"]["joint_offsets"] = offset_r
                if gripper_range_l:
                    config["left_arm"]["gripper_config"][1] = int(gripper_range_l[0])
                    config["left_arm"]["gripper_config"][2] = int(gripper_range_l[1])
                if gripper_range_r:
                    config["right_arm"]["gripper_config"][1] = int(gripper_range_r[0])
                    config["right_arm"]["gripper_config"][2] = int(gripper_range_r[1])
            else:
                if offset_l:
                    config["left_arm"]["joint_offsets"] = offset_l
                if offset_l:
                    config["right_arm"]["joint_offsets"] = offset_r

            save_config(self.config_file, config)

            return TriggerResponse(True, "Calibration routine started successfully")
        except Exception as e:
            return TriggerResponse(False, f"Calibration error: {str(e)}")

    def load_config(self) -> GelloConfig:
        """
        从 ROS 参数服务器加载配置并转换为 GelloConfig 对象。
        """
        try:
            # 获取参数
            config = rospy.get_param("remote_control")
            self.gripper_max_value = config.get("gripper_max_value", 1000)
            left_arm_config = config.get("left_arm", None)
            right_arm_config = config.get("right_arm", None)

            if left_arm_config:
                left_config = GelloArmConfig(
                    port=left_arm_config.get("port", ""),
                    start_joints=left_arm_config.get("start_joints", None),
                    calibrate_joints=left_arm_config.get("calibrate_joints", None),
                    joint_ids=left_arm_config.get("joint_ids", None),
                    joint_offsets=left_arm_config.get("joint_offsets", None),
                    joint_signs=left_arm_config.get("joint_signs", None),
                    gripper_config=left_arm_config.get("gripper_config", None),
                    gripper_angle=left_arm_config.get("gripper_angle", None),
                )
            else:
                left_config = GelloArmConfig()

            if right_arm_config:
                right_config = GelloArmConfig(
                    port=right_arm_config.get("port", ""),
                    start_joints=right_arm_config.get("start_joints", None),
                    calibrate_joints=right_arm_config.get("calibrate_joints", None),
                    joint_ids=right_arm_config.get("joint_ids", None),
                    joint_offsets=right_arm_config.get("joint_offsets", None),
                    joint_signs=right_arm_config.get("joint_signs", None),
                    gripper_config=right_arm_config.get("gripper_config", None),
                    gripper_angle=right_arm_config.get("gripper_angle", None),
                )
            else:
                right_config = GelloArmConfig()

            # 将参数加载到 GelloConfig 数据类中
            return GelloConfig(
                robot_port=config.get("robot_port", 6001),
                hostname=config.get("hostname", "127.0.0.1"),
                hz=config.get("hz", 100),
                arm_type=config.get("arm_type", ArmType.BIMANUAL.value),
                gripper_boarder_tolerance=config.get("gripper_boarder_tolerance", 0.1),
                gripper_boarder_ratio=config.get("gripper_boarder_ratio", 1.0),
                left_arm=left_config,
                right_arm=right_config,
            )
        except KeyError as e:
            rospy.logwarn(f"Missing required parameter: {e}, return default config!!!")
            return GelloConfig()

    def publish_status(self, event=None):  # 添加event参数适配Timer
        # 发布遥操关节数据
        remote_control_joints = DualJointStates()
        remote_control_joints.header.stamp = rospy.Time.now()
        remote_control_joints.left_joints = JointState()
        remote_control_joints.right_joints = JointState()

        # TODO: update remote control velocity and effort
        if self.remote_controller:
            joints_left, joints_right = self.remote_controller.get_joint_angles()
            if joints_left is not None:
                for idx, joint in enumerate(joints_left):
                    remote_control_joints.left_joints.name.append(
                        f"left_remote_control_{idx}"
                    )
                    remote_control_joints.left_joints.position.append(joint)
                    # remote_control_joints.left_joints.velocity.append()
                    # remote_control_joints.left_joints.effort.append()

                remote_control_joints.left_joints.position[-1] = (
                    1 - remote_control_joints.left_joints.position[-1]
                ) * self.gripper_max_value

            if joints_right is not None:
                for idx, joint in enumerate(joints_right):
                    remote_control_joints.right_joints.name.append(
                        f"right_remote_control_{idx}"
                    )
                    remote_control_joints.right_joints.position.append(joint)
                    # remote_control_joints.right_joints.velocity.append()
                    # remote_control_joints.right_joints.effort.append()

                remote_control_joints.right_joints.position[-1] = (
                    1 - remote_control_joints.right_joints.position[-1]
                ) * self.gripper_max_value

            remote_control_joints.is_gripper_held = (
                self.remote_controller.check_end_effector_state(
                    joints_left=joints_left, joints_right=joints_right
                )
            )

        self.joints_publisher.publish(remote_control_joints)

        # 发布遥操硬件状态
        remote_control_status = RemoteControlStatus()
        if self.remote_controller:
            # 连接状态：1表示连接，0表示断开
            remote_control_status.left_connection = (
                1 if self.remote_controller.is_connected else 0
            )
            remote_control_status.right_connection = (
                1 if self.remote_controller.is_connected else 0
            )

            # 自锁状态：1表示锁定，0表示未锁定
            remote_control_status.left_lock_state = 1 if self._lock_enabled else 0
            remote_control_status.right_lock_state = 1 if self._lock_enabled else 0
        else:
            # 控制器不存在，所有状态为0
            remote_control_status.left_connection = 0
            remote_control_status.right_connection = 0
            remote_control_status.left_lock_state = 0
            remote_control_status.right_lock_state = 0

        # 系统状态
        remote_control_status.system_status = 0  # 默认正常状态

        self.status_publisher.publish(remote_control_status)
        # 发布自锁状态
        self._publish_lock_status()


def main():
    remote_control = RemoteControlNode()
    remote_control.remote_controller.connect()
    rospy.spin()


if __name__ == "__main__":
    main()
