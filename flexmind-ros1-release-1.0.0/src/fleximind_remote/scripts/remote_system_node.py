#!/usr/bin/env python
import time

import actionlib
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from transitions.extensions import HierarchicalMachine as HMachine

from fleximind_bringup import (
    DEFAULT_AXIS_NUM,
    DEFAULT_LOG_PERIOD_S,
    DEFAULT_SERVICE_TIMEOUT_S,
)
from fleximind_bringup.interface import (
    GripperInterface,
    HardwareInterface,
    RemoteControlInterface,
    RemoteSystemInterface,
    RobotInterface,
)
from fleximind_gripper.msg import GripperData
from fleximind_hardware import MIN_BUTTON_PRESSED_TIME_S, MIN_GRIPPER_HELD_TIME_S
from fleximind_hardware.io import IOFunction, RobotIO
from fleximind_hardware.msg import DualJointStates, HardwareStatus
from fleximind_remote import (
    DEFAULT_CALIBRATE_PRESSED_TIME_S,
    ROBOT_ERROR_MAP,
    SYSTEM_STATES,
    ArmType,
    SoftwareFault,
    SystemState,
)
from fleximind_remote.gello.robots.CPS import RbtFSM
from fleximind_remote.msg import LockStatus, RemoteControlStatus
from fleximind_remote.srv import LockControl, LockControlResponse
from fleximind_robot.msg import RobotInfo, RobotLongJogJAction, RobotLongJogJGoal
from fleximind_robot.srv import PublishingSwitch, RobotBase
from fleximind_sensors.record_bag_client import RecordBagClient


class RemoteSystemNode:
    def __init__(self):
        # ROS1节点初始化
        rospy.init_node("remote_system")

        # TODO: 通过launch配置参数
        self._publish_queue_size = 10
        self._subscribe_queue_size = 10

        self.enable_timeout = 2
        self.default_axis_num = 12
        self.arm_dimension = 13

        # 配置硬件及状态变量
        self._setup_hardware()

        # 配置服务
        self._setup_service()

        # 数据发布器
        self._setup_publishers()

        # 初始化状态机（使用分层状态机）
        self._state = None
        self.machine = HMachine(
            model=self,
            states=SYSTEM_STATES,
            initial=SystemState.UNINITIALIZED.name,
            auto_transitions=False,
            ignore_invalid_triggers=True,  # 忽略无效触发
            model_attribute="_state",  # 指向可写属性
        )

        # 配置状态转换规则
        self._setup_transitions()

        # 配置订阅接口
        self._setup_subscriptions()

        self.set_robot_monitor(True)
        self.set_gripper_monitor(True)

        rospy.loginfo("RemoteSystemNode initialized")

    def _setup_publishers(self):
        """设置数据发布器"""
        # 关节状态发布器
        self.left_arm_action_pub = rospy.Publisher(
            RobotInterface.DATA_ACTION_LEFT_ARM.value,
            JointState,
            queue_size=self._publish_queue_size,
        )
        self.right_arm_action_pub = rospy.Publisher(
            RobotInterface.DATA_ACTION_RIGHT_ARM.value,
            JointState,
            queue_size=self._publish_queue_size,
        )
        self.left_arm_observation_pub = rospy.Publisher(
            RobotInterface.OBSERVATION_ACTION_LEFT_ARM.value,
            JointState,
            queue_size=self._publish_queue_size,
        )
        self.right_arm_observation_pub = rospy.Publisher(
            RobotInterface.OBSERVATION_ACTION_RIGHT_ARM.value,
            JointState,
            queue_size=self._publish_queue_size,
        )

        rospy.loginfo("数据发布器初始化完成")

    @property
    def state(self) -> str:
        return self._state

    def _publish_status(self, event=None):
        status_msg = RemoteControlStatus()
        status_msg.system_status = SystemState[self.state].value
        self.status_publisher.publish(status_msg)

    def _setup_hardware(self):
        self.recorder_client = RecordBagClient("record_bag")

        # 状态变量触发
        self._last_hardware_status = False
        self._last_record_button_status = False

        self._is_gripper_held = False
        self._gripper_hold_start = time.time()

        # 夹爪数据缓存
        self._last_gripper_left = 1000.0
        self._last_gripper_right = 1000.0
        self._has_gripper_data = False

        io_config: dict = rospy.get_param("remote_control", {}).get("io", {})
        self.io_map = {
            IOFunction.LEFT_STOP: RobotIO(
                ArmType.LEFT.value, io_config.get(IOFunction.LEFT_STOP.value, 0)
            ),
            IOFunction.LEFT_RECORD: RobotIO(
                ArmType.LEFT.value, io_config.get(IOFunction.LEFT_RECORD.value, 1)
            ),
            IOFunction.RIGHT_STOP: RobotIO(
                ArmType.RIGHT.value, io_config.get(IOFunction.RIGHT_STOP.value, 0)
            ),
            IOFunction.RIGHT_LOCK: RobotIO(
                ArmType.RIGHT.value, io_config.get(IOFunction.RIGHT_LOCK.value, 1)
            ),
        }

        self._robots_state = RobotInfo()

        self.remote_joints_left = []
        self.remote_joints_right = []
        self.control_system_joints_left = []
        self.control_system_joints_right = []

    def _setup_subscriptions(self):
        rospy.Subscriber(
            HardwareInterface.STATUS.value,
            HardwareStatus,
            self._hardware_status_callback,
        )

        rospy.Subscriber(
            RobotInterface.INFO.value,
            RobotInfo,
            self._control_status_callback,
        )

        # TODO: change to relative structure
        # 使用 message_filters 来订阅消息
        remote_control_joints_subscriber = message_filters.Subscriber(
            RemoteControlInterface.JOINT_STATES.value, DualJointStates
        )
        control_system_gripper_subscriber = message_filters.Subscriber(
            GripperInterface.INFO.value, GripperData
        )
        control_system_joints_subscriber = message_filters.Subscriber(
            RobotInterface.INFO.value, RobotInfo
        )

        rospy.Subscriber(
            RemoteControlInterface.JOINT_STATES.value,
            DualJointStates,
            self._remote_joints_callback,
        )

        rospy.Subscriber(
            RobotInterface.INFO.value, RobotInfo, self._robot_info_callback
        )

        rospy.Subscriber(
            GripperInterface.INFO.value, GripperData, self._gripper_info_callback
        )

        # 使用 TimeSynchronizer 来同步多个订阅者的消息, 频率差异较大时需要使用ApproximateTimeSynchronizer
        ts = message_filters.ApproximateTimeSynchronizer(
            [
                remote_control_joints_subscriber,
                control_system_gripper_subscriber,
                control_system_joints_subscriber,
            ],
            queue_size=50,
            slop=0.5,
        )
        ts.registerCallback(self._sync_pose_callback)

    def _remote_joints_callback(self, msg: DualJointStates):
        """远程关节数据单独回调"""
        self.remote_joints_left = list(msg.left_joints.position)
        self.remote_joints_right = list(msg.right_joints.position)

        # 确保远程关节数据有7个维度（6个关节+1个夹爪）
        if len(self.remote_joints_left) == DEFAULT_AXIS_NUM - 1:
            # 只有6个关节数据，添加夹爪数据
            self.remote_joints_left.append(0.0)  # 默认夹爪值
        if len(self.remote_joints_right) == DEFAULT_AXIS_NUM - 1:
            self.remote_joints_right.append(0.0)  # 默认夹爪值

        # 如果有足够的数据就发布
        if (
            hasattr(self, "control_system_joints_left")
            and hasattr(self, "control_system_joints_right")
            and len(self.remote_joints_left) == DEFAULT_AXIS_NUM
            and len(self.remote_joints_right) == DEFAULT_AXIS_NUM
        ):

            self._publish_joint_data_only(
                self.remote_joints_left,
                self.remote_joints_right,
                self.control_system_joints_left,
                self.control_system_joints_right,
            )

    def _robot_info_callback(self, msg: RobotInfo):
        """机器人信息单独回调"""
        self.control_system_joints_left = list(msg.left_robot.joint_positions) + list(
            msg.left_robot.eef_positions
        )
        self.control_system_joints_right = list(msg.right_robot.joint_positions) + list(
            msg.right_robot.eef_positions
        )

        if len(self.control_system_joints_left) == self.default_axis_num:
            gripper_value = self._last_gripper_left if self._has_gripper_data else 0.0
            self.control_system_joints_left.append(gripper_value)
        if len(self.control_system_joints_right) == self.default_axis_num:
            gripper_value = self._last_gripper_right if self._has_gripper_data else 0.0
            self.control_system_joints_right.append(gripper_value)

        # 更新机器人状态
        self.update_control_status(msg)
        self.update_io_state(msg)

    def _gripper_info_callback(self, msg: GripperData):
        """夹爪信息单独回调"""
        # 更新夹爪数据缓存
        self._last_gripper_left = msg.left_position
        self._last_gripper_right = msg.right_position
        self._has_gripper_data = True

        # 如果当前有控制系统的关节数据，更新其中的夹爪数据
        if (
            hasattr(self, "control_system_joints_left")
            and len(self.control_system_joints_left) == DEFAULT_AXIS_NUM
        ):
            self.control_system_joints_left[-1] = msg.left_position
        if (
            hasattr(self, "control_system_joints_right")
            and len(self.control_system_joints_right) == DEFAULT_AXIS_NUM
        ):
            self.control_system_joints_right[-1] = msg.right_position

    def _publish_joint_data_only(
        self, remote_left, remote_right, control_left, control_right
    ):
        """仅发布关节数据，不包含其他逻辑"""
        try:
            # 确保所有数据都是7维
            if len(remote_left) != DEFAULT_AXIS_NUM:
                rospy.logwarn_throttle(
                    DEFAULT_LOG_PERIOD_S,
                    f"远程左臂数据维度错误: {len(remote_left)}，期望: {DEFAULT_AXIS_NUM}",
                )
                return
            if len(remote_right) != DEFAULT_AXIS_NUM:
                rospy.logwarn_throttle(
                    DEFAULT_LOG_PERIOD_S,
                    f"远程右臂数据维度错误: {len(remote_right)}，期望: {DEFAULT_AXIS_NUM}",
                )
                return

            if len(control_left) != self.arm_dimension:
                rospy.logwarn_throttle(
                    DEFAULT_LOG_PERIOD_S,
                    f"控制左臂数据维度错误: {len(control_left)}，期望: 13",
                )
                return
            if len(control_right) != self.arm_dimension:
                rospy.logwarn_throttle(
                    DEFAULT_LOG_PERIOD_S,
                    f"控制右臂数据维度错误: {len(control_right)}，期望: 13",
                )
                return

            # 复制control数据，避免修改原始数据
            control_left_modified = control_left.copy()
            control_right_modified = control_right.copy()

            # 将observation的最后一个值（夹爪）替换为action的最后一个值（夹爪）
            control_left_modified[-1] = remote_left[-1]  # 左臂夹爪
            control_right_modified[-1] = remote_right[-1]  # 右臂夹爪

            # 获取当前时间戳
            current_time = rospy.Time.now()

            # 发布左臂动作数据 (远程控制数据)
            left_action_msg = JointState()
            left_action_msg.header.stamp = current_time
            left_action_msg.name = [f"joint_{i+1}" for i in range(len(remote_left))]
            left_action_msg.position = remote_left
            left_action_msg.velocity = []
            left_action_msg.effort = []
            self.left_arm_action_pub.publish(left_action_msg)

            # 发布右臂动作数据 (远程控制数据)
            right_action_msg = JointState()
            right_action_msg.header.stamp = current_time
            right_action_msg.name = [f"joint_{i+1}" for i in range(len(remote_right))]
            right_action_msg.position = remote_right
            right_action_msg.velocity = []
            right_action_msg.effort = []
            self.right_arm_action_pub.publish(right_action_msg)

            # 发布左臂观测数据 (控制系统实际数据)
            left_observation_msg = JointState()
            left_observation_msg.header.stamp = current_time
            left_observation_msg.name = [
                f"joint_{i+1}"
                for i in range(len(control_left_modified))  # 使用修改后的数据
            ]
            left_observation_msg.position = control_left_modified  # 使用修改后的数据
            left_observation_msg.velocity = []
            left_observation_msg.effort = []
            self.left_arm_observation_pub.publish(left_observation_msg)

            # 发布右臂观测数据 (控制系统实际数据)
            right_observation_msg = JointState()
            right_observation_msg.header.stamp = current_time
            right_observation_msg.name = [
                f"joint_{i+1}"
                for i in range(len(control_right_modified))  # 使用修改后的数据
            ]
            right_observation_msg.position = control_right_modified  # 使用修改后的数据
            right_observation_msg.velocity = []
            right_observation_msg.effort = []
            self.right_arm_observation_pub.publish(right_observation_msg)

            # 可选：调试日志
            rospy.logdebug_throttle(
                DEFAULT_LOG_PERIOD_S,
                f"单独发布关节数据: 左臂动作{len(remote_left)}维, 右臂动作{len(remote_right)}维, "
                f"左臂观测{len(control_left)}维, 右臂观测{len(control_right)}维",
            )

        except Exception as e:
            rospy.logwarn(f"单独发布关节数据失败: {str(e)}")

    def _setup_service(self):
        self.set_gripper_data_publish = None
        self.set_robot_data_publish = None
        self.set_servoj = None
        self.calibrate = None
        self.stop_robot = None
        self.enable = None
        self.disable = None

        try:
            rospy.wait_for_service(
                GripperInterface.GRIPPER_DATA_PUB.value,
                rospy.Duration(DEFAULT_SERVICE_TIMEOUT_S),
            )
            self.set_gripper_data_publish = rospy.ServiceProxy(
                GripperInterface.GRIPPER_DATA_PUB.value, PublishingSwitch
            )
        except Exception as e:
            rospy.logerr(f"Service call failed in remote system node: {e}")

        try:
            rospy.wait_for_service(
                RobotInterface.DATA_PUB.value,
                rospy.Duration(DEFAULT_SERVICE_TIMEOUT_S),
            )
            self.set_robot_data_publish = rospy.ServiceProxy(
                RobotInterface.DATA_PUB.value, PublishingSwitch
            )
        except Exception as e:
            rospy.logerr(f"Service call failed in remote system node: {e}")

        try:
            rospy.wait_for_service(
                RobotInterface.SERVOJ.value,
                rospy.Duration(DEFAULT_SERVICE_TIMEOUT_S),
            )
            self.set_servoj = rospy.ServiceProxy(
                RobotInterface.SERVOJ.value, PublishingSwitch
            )
        except Exception as e:
            rospy.logerr(f"Service call failed in remote system node: {e}")

        try:
            rospy.wait_for_service(
                RemoteControlInterface.CALIBRATE.value,
                rospy.Duration(DEFAULT_SERVICE_TIMEOUT_S),
            )
            self.calibrate = rospy.ServiceProxy(
                RemoteControlInterface.CALIBRATE.value, Trigger
            )
        except Exception as e:
            rospy.logerr(f"Service call failed in remote system node: {e}")

        try:
            rospy.wait_for_service(
                RobotInterface.EMERGENCY_STOP.value,
                rospy.Duration(DEFAULT_SERVICE_TIMEOUT_S),
            )
            self.stop_robot = rospy.ServiceProxy(
                RobotInterface.EMERGENCY_STOP.value, RobotBase
            )
        except Exception as e:
            rospy.logerr(f"Service call failed in remote system node: {e}")

        try:
            rospy.wait_for_service(
                RobotInterface.ENABLE.value,
                rospy.Duration(DEFAULT_SERVICE_TIMEOUT_S),
            )
            self.enable = rospy.ServiceProxy(RobotInterface.ENABLE.value, RobotBase)
        except Exception as e:
            rospy.logerr(f"Service call failed in remote system node: {e}")

        try:
            rospy.wait_for_service(
                RobotInterface.SETSPEED.value,
                rospy.Duration(DEFAULT_SERVICE_TIMEOUT_S),
            )
            self.setspeed = rospy.ServiceProxy(RobotInterface.SETSPEED.value, RobotBase)
        except Exception as e:
            rospy.logerr(f"Service call failed in remote system node: {e}")

        try:
            rospy.wait_for_service(
                RobotInterface.DISABLE.value,
                rospy.Duration(DEFAULT_SERVICE_TIMEOUT_S),
            )
            self.disable = rospy.ServiceProxy(RobotInterface.DISABLE.value, RobotBase)
        except Exception as e:
            rospy.logerr(f"Service call failed in remote system node: {e}")

        self.robot_long_jog_client = actionlib.SimpleActionClient(
            RobotInterface.MOVE_JOINTS.value, RobotLongJogJAction
        )

        self.status_publisher = rospy.Publisher(
            RemoteSystemInterface.STATUS.value,
            RemoteControlStatus,
            queue_size=self._publish_queue_size,
        )

    def _hardware_status_callback(self, msg: HardwareStatus):
        """硬件状态回调"""
        # 与上一状态一致则不触发回调
        if self._last_hardware_status == msg.overall_status:
            if not self._last_hardware_status:
                rospy.loginfo_throttle(
                    DEFAULT_LOG_PERIOD_S,
                    f"last_hardware_status: {self._last_hardware_status}",
                )
            return

        if msg.overall_status:
            self.hardware_self_test_passed()
        else:
            error_num_map = {
                "camera": msg.camera_error_num,
                "robot": msg.robot_error_num,
                "gripper": msg.gripper_error_num,
                "remote": msg.remote_error_num,
            }
            max_key = max(error_num_map, key=error_num_map.get)
            self.hardware_fault_detected(max_key)

        self._last_hardware_status = msg.overall_status

    def _control_status_callback(self, msg: RobotInfo):
        self.update_control_status(msg)
        self.update_io_state(msg)

        if self.is_lock_button_triggered():
            self.toggle_arm_lock()

        if self.is_control_status_error():
            self.software_fault_detected()
        else:
            self.maintenance_reset()

        if self.is_stop_button_pressed() and self.state != SystemState.STANDBY.name:
            self.stop_button_pressed()

        if self.is_record_button_triggered():
            if not self.recorder_client.is_recording:
                self.recorder_client.start_recording()
            else:
                self.recorder_client.cancel_recording()

        if (
            self.is_stop_button_pressed(DEFAULT_CALIBRATE_PRESSED_TIME_S)
            and self.state == SystemState.STANDBY.name
        ):
            self.calibrate()

    def _sync_pose_callback(
        self,
        remote_control_joints_msg: DualJointStates,
        control_system_gripper_msg: GripperData,
        control_system_joints_msg: RobotInfo,
        joint_error_threshold=5.0,
    ):
        self.update_gripper_state(remote_control_joints_msg.is_gripper_held)

        if self.is_gripper_held_long() and self.state == SystemState.STANDBY.name:
            rospy.loginfo("start_syncing")
            self.start_syncing()
        elif not self.is_gripper_held_long() and self.state == SystemState.SYNCING.name:
            rospy.loginfo("cancel_syncing")
            self.cancel_syncing()
        elif (
            self.is_gripper_released()
            and self.state == SystemState.SYNCED.name
            # and self._robots_state.left_robot.robotState == RbtFSM.enCPSState_StandBy
            # and self._robots_state.right_robot.robotState == RbtFSM.enCPSState_StandBy
        ):
            rospy.loginfo("start_following")
            self.start_following()

        # 计算主从臂误差
        self.remote_joints_left = list(remote_control_joints_msg.left_joints.position)
        self.remote_joints_right = list(remote_control_joints_msg.right_joints.position)

        # 确保远程关节数据有7个维度
        if len(self.remote_joints_left) == DEFAULT_AXIS_NUM - 1:
            self.remote_joints_left.append(0.0)
        if len(self.remote_joints_right) == DEFAULT_AXIS_NUM - 1:
            self.remote_joints_right.append(0.0)

        # self.control_system_joints_left = list(
        #     control_system_joints_msg.left_robot.joint_positions
        # ) + list(
        #     control_system_joints_msg.left_robot.eef_positions
        # )
        # self.control_system_joints_right = list(
        #     control_system_joints_msg.right_robot.joint_positions
        # ) + list(
        #     control_system_joints_msg.right_robot.eef_positions
        # )

        # 更新夹爪数据缓存
        self._last_gripper_left = control_system_gripper_msg.left_position
        self._last_gripper_right = control_system_gripper_msg.right_position
        self._has_gripper_data = True

        # 确保控制关节数据有7个维度
        # if len(self.control_system_joints_left) == DEFAULT_AXIS_NUM - 1:
        #     self.control_system_joints_left.append(
        #         control_system_gripper_msg.left_position
        #     )
        # if len(self.control_system_joints_right) == DEFAULT_AXIS_NUM - 1:
        #     self.control_system_joints_right.append(
        #         control_system_gripper_msg.right_position
        #     )

        # rospy.loginfo_throttle(DEFAULT_LOG_PERIOD_S, f"self.remote_joints_left: {self.remote_joints_left}")
        # rospy.loginfo_throttle(DEFAULT_LOG_PERIOD_S, f"self.remote_joints_right: {self.remote_joints_right}")

        # rospy.loginfo_throttle(DEFAULT_LOG_PERIOD_S, f"self.control_system_joints_left: {self.control_system_joints_left}")
        # rospy.loginfo_throttle(DEFAULT_LOG_PERIOD_S, f"self.control_system_joints_right: {self.control_system_joints_right}")

        if (
            len(self.remote_joints_left) != DEFAULT_AXIS_NUM
            or len(self.remote_joints_right) != DEFAULT_AXIS_NUM
            or len(self.control_system_joints_left) != self.arm_dimension
            or len(self.control_system_joints_right) != self.arm_dimension
        ):
            rospy.logerr_throttle(DEFAULT_LOG_PERIOD_S, f"joint nums error!")
            return

        joint_error_left = np.mean(
            np.abs(
                np.array(self.remote_joints_left[:6])
                - np.array(self.control_system_joints_left[:6])
            )
        )
        joint_error_right = np.mean(
            np.abs(
                np.array(self.remote_joints_right[:6])
                - np.array(self.control_system_joints_right[:6])
            )
        )
        joint_error = max(joint_error_left, joint_error_right)
        # rospy.loginfo_throttle(
        #     DEFAULT_LOG_PERIOD_S, f"joint_error_left: {joint_error_left}"
        # )
        # rospy.loginfo_throttle(
        #     DEFAULT_LOG_PERIOD_S, f"joint_error_right: {joint_error_right}"
        # )
        # rospy.loginfo_throttle(DEFAULT_LOG_PERIOD_S, f"joint_error: {joint_error}")

        if (
            joint_error <= joint_error_threshold
            and self.state == SystemState.SYNCING.name
        ):
            # 每次访问trigger接口刷新时效性, 只在条件符合时开始判断按钮trigger
            if self.is_follow_button_triggered():
                self.sync_completed()

    def _setup_transitions(self):
        """根据表格配置状态转换规则"""
        transitions = [
            # 1. 未初始化 → 待机 (硬件自检通过)
            {
                "trigger": "hardware_self_test_passed",
                "source": SystemState.UNINITIALIZED.name,
                "dest": SystemState.STANDBY.name,
            },
            # 2. 待机 → 同步中 (长握夹爪 + 关节限位通过)
            {
                "trigger": "start_syncing",
                "source": SystemState.STANDBY.name,
                "dest": SystemState.SYNCING.name,
            },
            # 3. 同步中 → 已同步 (姿态误差达标)
            {
                "trigger": "sync_completed",
                "source": SystemState.SYNCING.name,
                "dest": SystemState.SYNCED.name,
            },
            # 4. 同步中 → 待机 (松开夹爪)
            {
                "trigger": "cancel_syncing",
                "source": SystemState.SYNCING.name,
                "dest": SystemState.STANDBY.name,
            },
            # 5. 已同步 → 跟随中 (松开夹爪 + 符合跟随条件)
            {
                "trigger": "start_following",
                "source": SystemState.SYNCED.name,
                "dest": SystemState.FOLLOWING.name,
            },
            # 6. 同步中/已同步/跟随中 → 待机 (停止按钮)
            {
                "trigger": "stop_button_pressed",
                "source": [
                    SystemState.SYNCING.name,
                    SystemState.SYNCED.name,
                    SystemState.FOLLOWING.name,
                ],
                "dest": SystemState.STANDBY.name,
                "before": "stop_machine",
            },
            # 7. 跟随中 → 软件异常 (通信超时/碰撞检测)
            {
                "trigger": "software_fault_detected",
                "source": [
                    SystemState.STANDBY.name,
                    SystemState.SYNCING.name,
                    SystemState.FOLLOWING.name,
                ],
                "dest": SystemState.SOFTWARE_EXCEPTION.name,
                "before": "stop_machine",
                "after": "handle_software_fault",
            },
            # 8. 任意状态 → 硬件异常 (传感器失效)
            {
                "trigger": "hardware_fault_detected",
                "source": "*",
                "dest": SystemState.HARDWARE_EXCEPTION.name,
                "before": "stop_machine",
                "after": "handle_hardware_fault",
            },
            # 9. 任意状态 → 急停 (急停按钮按下)
            {
                "trigger": "emergency_stop_pressed",
                "source": "*",
                "dest": SystemState.EMERGENCY_STOP.name,
                "before": "activate_emergency_stop",
            },
            # 10. 急停 → 待机 (急停按钮再次按下)
            {
                "trigger": "emergency_stop_released",
                "source": SystemState.EMERGENCY_STOP.name,
                "dest": SystemState.STANDBY.name,
                "before": "deactivate_emergency_stop",
            },
            # 11. 异常恢复 (维护复位)
            {
                "trigger": "maintenance_reset",
                "source": [
                    SystemState.SOFTWARE_EXCEPTION.name,
                    SystemState.HARDWARE_EXCEPTION.name,
                ],
                "dest": SystemState.STANDBY.name,
                "conditions": "is_system_recoverable",
            },
        ]

        # 自动生成对应的trigger函数，一些短时操作可在状态切换或者转移函数前后进行
        self.machine.add_transitions(transitions)
        self.machine.before_state_change = self._before_state_change_callback
        self.machine.after_state_change = self._after_state_change_callback

    def _before_state_change_callback(self, *args):
        rospy.logwarn(f"before change state: {self._state}")

    def _after_state_change_callback(self, *args):
        rospy.logwarn(f"after change state: {self._state}")
        self._publish_status()

    # ------------------------
    # 条件检查方法
    # ------------------------
    def is_hardware_ready(self):
        if (
            self._robots_state.left_robot.robotState != RbtFSM.enCPSState_StandBy
            or not self._robots_state.left_robot.robotEnabled
        ):
            return False

        if (
            self._robots_state.right_robot.robotState != RbtFSM.enCPSState_StandBy
            or not self._robots_state.right_robot.robotEnabled
        ):
            return False

        return True

    def is_gripper_held_long(self, held_time=MIN_GRIPPER_HELD_TIME_S):
        """检查是否长握末端夹爪"""
        return self._is_gripper_held and (
            time.time() - self._gripper_hold_start >= held_time
        )

    def is_gripper_released(self):
        """是否松开末端夹爪"""
        return not self._is_gripper_held

    def is_system_recoverable(self):
        """系统是否可恢复（维护复位条件）"""
        return True

    def is_control_status_error(self):
        """控制是否出现软件异常"""
        state = self._robots_state
        # rospy.loginfo_throttle(DEFAULT_LOG_PERIOD_S, f"state.left_robot.robotState: {state.left_robot.robotState}")
        # rospy.loginfo_throttle(DEFAULT_LOG_PERIOD_S, f"state.right_robot.robotState: {state.right_robot.robotState}")
        if state.left_robot.robotState in ROBOT_ERROR_MAP:
            rospy.logerr_throttle(
                DEFAULT_LOG_PERIOD_S,
                f"left control status error: {state.left_robot.robotState}",
            )
            return True
        if state.right_robot.robotState in ROBOT_ERROR_MAP:
            rospy.logerr_throttle(
                DEFAULT_LOG_PERIOD_S,
                f"right control status error: {state.right_robot.robotState}",
            )
            return True

        return False

    def is_stop_button_pressed(self, check_time_s=MIN_BUTTON_PRESSED_TIME_S):
        """停止按钮是否被按下"""
        right_stop_io = self.io_map[IOFunction.RIGHT_STOP]
        if right_stop_io.current_state and right_stop_io.activate_time >= check_time_s:
            rospy.loginfo_throttle(
                DEFAULT_LOG_PERIOD_S, f"{IOFunction.RIGHT_STOP} button is pressed"
            )
            return True

        return False

    def is_record_button_triggered(self):
        """采集按钮是否被按下"""
        record_io = self.io_map[IOFunction.LEFT_RECORD]
        if record_io.high_trigger_count > 0:
            rospy.loginfo_throttle(
                DEFAULT_LOG_PERIOD_S, f"{IOFunction.LEFT_RECORD} button is pressed"
            )
            return True

        return False

    def is_lock_button_triggered(self):
        """自锁按钮是否被触发"""
        lock_io = self.io_map[IOFunction.RIGHT_LOCK]
        if lock_io.high_trigger_count > 0:
            rospy.loginfo(
                f"{IOFunction.RIGHT_LOCK} button is pressed - toggle arm lock"
            )
            return True
        return False

    def is_follow_button_triggered(self):
        """跟随按钮是否被触发"""
        left_stop_io = self.io_map[IOFunction.LEFT_STOP]
        if left_stop_io.high_trigger_count > 0:
            rospy.loginfo_throttle(
                DEFAULT_LOG_PERIOD_S, f"{IOFunction.LEFT_STOP} button is pressed"
            )
            return True

    def toggle_arm_lock(self):
        """切换自锁状态"""
        try:
            lock_service = rospy.ServiceProxy("/gello/lock_control", LockControl)
            response = lock_service()
            if response.success:
                rospy.loginfo(f"自锁状态切换成功: {response.message}")
            else:
                rospy.logwarn(f"自锁状态切换失败: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"调用自锁服务失败: {e}")

    # ------------------------
    # 状态生命周期回调方法
    # ------------------------
    def on_enter_STANDBY(self):
        """进入跟随中状态回调"""
        try:
            if self.disable:
                self.disable()

        except rospy.ServiceException as e:
            self.software_fault_detected()
            rospy.logerr("Service call failed in STANDBY: %s" % e)

    def on_exit_STANDBY(self, *args):
        """进入跟随中状态回调"""
        try:
            pass

        except rospy.ServiceException as e:
            self.software_fault_detected()
            rospy.logerr("Service call failed in STANDBY: %s" % e)

    def on_enter_SYNCING(self):
        """进入同步中状态回调"""
        try:
            self.setspeed()
            if self.enable:
                self.enable()
                self.start_enable_time = time.time()
            while True:
                if (
                    self._robots_state.left_robot.robotEnabled
                    and self._robots_state.right_robot.robotEnabled
                ):
                    rospy.loginfo("enabled sucessfully")
                    break
                # 检查超时
                if time.time() - self.start_enable_time > self.enable_timeout:
                    rospy.loginfo("等待机器人使能超时，强制退出")
                    break
                time.sleep(0.1)  # 避免CPU占用过高
            # 打开同步
            goal = RobotLongJogJGoal()
            goal.left_boxID = rospy.get_param("/fleximind_robot_node/boxid_left", 0)
            goal.left_rbtID = 0
            goal.left_joints = self.remote_joints_left[:6]
            goal.right_boxID = rospy.get_param("/fleximind_robot_node/boxid_right", 1)
            goal.right_rbtID = 0
            goal.right_joints = self.remote_joints_right[:6]
            rospy.loginfo("=== 同步状态关节信息 ===")
            rospy.loginfo(
                f"Left joints:  [{', '.join(f'{x:.6f}' for x in self.remote_joints_left[:6])}]"
            )
            rospy.loginfo(
                f"Right joints: [{', '.join(f'{x:.6f}' for x in self.remote_joints_right[:6])}]"
            )
            rospy.loginfo(
                f"Left boxID: {goal.left_boxID}, Right boxID: {goal.right_boxID}"
            )
            rospy.loginfo("========================")

            self.robot_long_jog_client.send_goal(goal)

        except rospy.ServiceException as e:
            self.software_fault_detected()
            rospy.logerr("Service call failed in SYNCING: %s" % e)

    def on_exit_SYNCING(self, *args):
        """退出同步中状态回调"""
        try:
            # 关闭同步
            self.robot_long_jog_client.cancel_goal()
            self.robot_long_jog_client.wait_for_result()

        except rospy.ServiceException as e:
            self.software_fault_detected()
            rospy.logerr("Service call failed in SYNCING: %s" % e)

    def on_enter_FOLLOWING(self):
        """进入跟随中状态回调"""
        try:
            # 打开跟随
            response = self.set_servoj(True)
            if response.code != 0:
                rospy.logwarn(f"Start servoj failed: {response.message}")

        except rospy.ServiceException as e:
            self.software_fault_detected()
            rospy.logerr("Service call failed in FOLLOWING: %s" % e)

    def on_exit_FOLLOWING(self, *args):
        """进入跟随中状态回调"""
        try:
            # 关闭跟随
            response = self.set_servoj(False)
            if response.code != 0:
                rospy.logwarn(f"Stop servoj failed: {response.message}")

        except rospy.ServiceException as e:
            self.software_fault_detected()
            rospy.logerr("Service call failed in FOLLOWING: %s" % e)

    # ------------------------
    # 异常处理方法
    # ------------------------
    def handle_software_fault(self, fault_type=SoftwareFault.UNEXPECTED):
        """软件异常处理"""
        if fault_type == SoftwareFault.COLLISION:
            pass
        elif fault_type == SoftwareFault.FOLLOW_ERROR:
            pass

    def handle_hardware_fault(self, component="Unknown"):
        """硬件异常处理"""
        rospy.logerr(f"硬件异常! 组件: {component}")

    # ------------------------
    # 硬件控制方法
    # ------------------------
    def set_robot_monitor(self, state: bool):
        if not self.set_robot_data_publish:
            rospy.logwarn(f"set robot data publish service not found!")
            return False

        response = self.set_robot_data_publish(state)
        if response.code != 0:
            rospy.logwarn(f"Set robot data publish failed: {response.message}")
            return False
        return True

    def set_gripper_monitor(self, state: bool):
        if not self.set_gripper_data_publish:
            rospy.logwarn(f"set gripper data publish service not found!")
            return False

        response = self.set_gripper_data_publish(state)
        if response.code != 0:
            rospy.logwarn(f"Set gripper data publish failed: {response.message}")
            return False
        return True

    def update_gripper_state(self, is_held: bool):
        if is_held != self._is_gripper_held:
            if is_held:
                self._gripper_hold_start = time.time()

            self._is_gripper_held = is_held

    def update_io_state(self, msg: RobotInfo):
        """刷新IO状态"""
        for io_key, io_data in self.io_map.items():
            if io_key in [IOFunction.LEFT_STOP, IOFunction.LEFT_RECORD]:
                io_data.current_state = msg.left_robot.BoxCI[io_data.bit]
            if io_key in [IOFunction.RIGHT_STOP, IOFunction.RIGHT_LOCK]:
                io_data.current_state = msg.right_robot.BoxCI[io_data.bit]

    def update_control_status(self, msg: RobotInfo):
        """刷新控制系统状态"""
        self._robots_state = msg

    def stop_machine(self, *args):
        if self.stop_robot:
            self.robot_long_jog_client.cancel_goal()
            self.robot_long_jog_client.wait_for_result()
            self.stop_robot()

    def activate_emergency_stop(self):
        """激活急停"""
        pass

    def deactivate_emergency_stop(self):
        """关闭急停"""
        pass


def main():
    remote_system = RemoteSystemNode()
    rospy.spin()


if __name__ == "__main__":
    main()
