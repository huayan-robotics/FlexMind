#!/usr/bin/env python
# -- coding: utf-8 --
import threading
import time
from enum import Enum, IntEnum

import actionlib
import rospy
from std_msgs.msg import Int32

from fleximind_bringup import DEFAULT_LOG_PERIOD_S
from fleximind_bringup.interface import RemoteControlInterface, RobotInterface
from fleximind_hardware import HardwareState, HardwareType
from fleximind_hardware.msg import ConnectionStatus, DualJointStates
from fleximind_robot.CPS import RbtFSM
from fleximind_robot.huayan_robot import CPScontrol
from fleximind_robot.move_plan import RealTimeMoveController
from fleximind_robot.msg import (
    Robot,
    RobotInfo,
    RobotLongJogJAction,
    RobotLongJogJFeedback,
    RobotLongJogJGoal,
    RobotLongJogJResult,
)
from fleximind_robot.robot_listener import RobotMonitor
from fleximind_robot.servo_plan import RealTimeServoController

# 业务相关依赖
from fleximind_robot.srv import (
    FollowingType,
    FollowingTypeRequest,
    FollowingTypeResponse,
    GetIP,
    GetIPResponse,
    PublishingSwitch,
    PublishingSwitchRequest,
    PublishingSwitchResponse,
    RobotBase,
    RobotBaseResponse,
    SingleArmControl,
    SingleArmControlRequest,
    SingleArmControlResponse,
)


class DualRobotState(IntEnum):
    enRobotState_Ready = 0  # 就绪状态
    enRobotState_PowerOff = 1  # 未上电状态
    enRobotState_NotReady = 2  # 未就绪状态（已上电但未达到就绪条件）
    enRobotState_Error = 3  # 错误状态
    enRobotState_EmergencyStop = 4  # 急停状态

    enRobotState_ResponseOK = 200  # 响应成功
    enRobotState_ConnectionFailed = 201  # 双臂连接失败
    enRobotState_ResetFailed = 202  # 双臂重置失败
    enRobotState_EmergencyStopFailed = 203  # 双臂紧急停止失败
    enRobotState_ElectricFailed = 204  # 双臂上电失败
    enRobotState_RobotStartFailed = 205  # 机械臂启动失败
    enRobotState_RobotPowerOutageFailed = 206  # 双臂断电失败
    enRobotState_RobotEnableFailed = 207  # 双臂使能失败
    enRobotState_RobotDisableFailed = 208  # 双臂去使能失败
    enRobotState_ResponseError = 500  # 响应错误


class FollowType(Enum):
    SERVOJ = 0  # servoj模式
    MOVEJ = 1  # movej模式


class DualRobotControlNode:
    def __init__(self):
        # 初始化节点
        rospy.init_node("fleximind_robot_node")
        self.lock = threading.Lock()
        self.record_control = 1
        self.gello_lock = 0
        self.device_state = ConnectionStatus()
        self.device_state.name = HardwareType.ROBOT.value
        self.device_state.status = HardwareState.OFFLINE.value
        self.is_publishing = False
        self.left_robot_name = "left_robot"
        self.right_robot_name = "right_robot"

        # 添加当前跟随模式状态 (0: servoj, 1: movej)
        self.current_following_type = FollowType.SERVOJ.value  # 默认servoj模式

        # 状态分类定义
        self._init_state()
        # 读取参数配置
        self._read_config()
        # 初始化双机械臂控制器
        self._init_dual_arm()
        # 创建服务
        self._create_services()
        # 创建发布器
        self._create_publishers()
        # 创建动作
        self._create_actions()

        self._create_state_publisher()

        # 定时器，若未配置则默认0.1s
        period = getattr(self, "timer_period", 0.1)
        rospy.Timer(rospy.Duration(period), self.timer_callback)
        rospy.loginfo("机械臂控制节点初始化完成")

        self.state_pub = rospy.Publisher(
            RobotInterface.DUAL_ROBOT_SYSTEM_STATE.value, Int32, queue_size=10
        )

    def _create_state_publisher(self):
        """创建状态发布器"""
        from fleximind_bringup.interface import RemoteSystemInterface
        from fleximind_remote.msg import RemoteControlStatus

        self.state_publisher = rospy.Publisher(
            RemoteSystemInterface.STATUS.value, RemoteControlStatus, queue_size=10
        )

    def __del__(self):
        """析构函数"""
        try:
            if hasattr(self, "robot_monitor_l") and self.robot_monitor_l:
                self.robot_monitor_l.close()
            if hasattr(self, "robot_monitor_r") and self.robot_monitor_r:
                self.robot_monitor_r.close()
        except Exception as e:
            rospy.logwarn(f"析构资源释放异常: {e}")

    def _init_dual_arm(self):
        """初始化双机械臂控制器"""
        try:
            self.arm_left = CPScontrol(
                name=self.left_robot_name,
                box_id=self.boxid_left,
                ip=self.ip_left,
                port=self.port,
            )
            self.arm_right = CPScontrol(
                name=self.right_robot_name,
                box_id=self.boxid_right,
                ip=self.ip_right,
                port=self.port,
            )
            with self.lock:
                left_code = self.arm_left.connect()
                right_code = self.arm_right.connect()
            self.device_state.status = (
                HardwareState.ONLINE.value
                if (left_code and right_code)
                else HardwareState.OFFLINE.value
            )

            self.robot_monitor_l = RobotMonitor(self.ip_left, self.monitor_port)
            self.robot_monitor_l.start_monitoring()
            self.robot_monitor_r = RobotMonitor(self.ip_right, self.monitor_port)
            self.robot_monitor_r.start_monitoring()

            try:
                self.servo_left = RealTimeServoController(
                    self.ip_left,
                    self.boxid_left,
                    self.left_robot_name,
                    self.interp_ratios,
                    self.max_velocity,
                    self.max_accel,
                    self.max_jerk,
                    self.serve_time,
                    self.dp,
                    self.lookahead_time,
                    self.buffer_size,
                )
                self.servo_right = RealTimeServoController(
                    self.ip_right,
                    self.boxid_right,
                    self.right_robot_name,
                    self.interp_ratios,
                    self.max_velocity,
                    self.max_accel,
                    self.max_jerk,
                    self.serve_time,
                    self.dp,
                    self.lookahead_time,
                    self.buffer_size,
                )
            except Exception as e:
                rospy.logwarn(f"伺服控制器启动失败: {e}")

            # 初始化MoveJ控制器
            try:
                self.move_left = RealTimeMoveController(
                    self.ip_left,
                    self.boxid_left,
                    self.left_robot_name,
                    port=self.port,
                    movej_frequency=self.movej_frequency,
                    log_dir=self.log_dir,
                    min_path_length=self.min_path_length,
                    max_angle_threshold=self.max_angle_threshold,
                    extreme_angle_threshold=self.extreme_angle_threshold,
                    curvature_threshold=self.curvature_threshold,
                    angle_diff_threshold=self.angle_diff_threshold,
                )
                self.move_right = RealTimeMoveController(
                    self.ip_right,
                    self.boxid_right,
                    self.right_robot_name,
                    port=self.port,
                    movej_frequency=self.movej_frequency,
                    log_dir=self.log_dir,
                    min_path_length=self.min_path_length,
                    max_angle_threshold=self.max_angle_threshold,
                    extreme_angle_threshold=self.extreme_angle_threshold,
                    curvature_threshold=self.curvature_threshold,
                    angle_diff_threshold=self.angle_diff_threshold,
                )
            except Exception as e:
                rospy.logwarn(f"MoveJ控制器启动失败: {e}")

            rospy.loginfo("机械臂控制器初始化成功")
        except Exception as e:
            rospy.logerr(f"机械臂初始化失败: {e}")
            raise

    def _init_state(self):
        """初始化状态"""
        self.POWER_OFF_STATES = {
            RbtFSM.enCPSState_ElectricBoxDisconnect,
            RbtFSM.enCPSState_ElectricBoxConnecting,
            RbtFSM.enCPSState_Blackouting48V,
            RbtFSM.enCPSState_Blackout48V,
            RbtFSM.enCPSState_ControllerDisconnecting,
            RbtFSM.enCPSState_Electrifying48V,
            RbtFSM.enCPSState_ControllerChecking,
            RbtFSM.enCPSState_ControllerDisconnect,
        }
        self.ERROR_STATES = {
            RbtFSM.enCPSState_EmergencyStopHandling,
            RbtFSM.enCPSState_EmergencyStop,
            RbtFSM.enCPSState_SafetyGuardErrorHandling,
            RbtFSM.enCPSState_SafetyGuardError,
            RbtFSM.enCPSState_ControllerVersionError,
            RbtFSM.enCPSState_EtherCATError,
            RbtFSM.enCPSState_ControllerDisconnecting,
            RbtFSM.enCPSState_SafetyGuardHandling,
            RbtFSM.enCPSState_SafetyGuard,
            RbtFSM.enCPSState_RobotOutofSafeSpace,
            RbtFSM.enCPSState_RobotCollisionStop,
            RbtFSM.enCPSState_Error,
            RbtFSM.enCPSState_Disabling,
            RbtFSM.enCPSState_RobotHolding,
            RbtFSM.enCPSState_HRAppDisconnected,
            RbtFSM.enCPSState_HRAppError,
            RbtFSM.enCPSState_TemperatureTooLow,
        }
        self.READY_STATES = {
            RbtFSM.enCPSState_StandBy,
            RbtFSM.enCPSState_Moving,
            RbtFSM.enCPSState_LongJogMoving,
            RbtFSM.enCPSState_RobotStopping,
            RbtFSM.enCPSState_RobotOpeningFreeDriver,
            RbtFSM.enCPSState_RobotClosingFreeDriver,
            RbtFSM.enCPSState_FreeDriver,
            RbtFSM.enCPSState_FTOpeningFreeDriver,
            RbtFSM.enCPSState_FTClosingFreeDriver,
            RbtFSM.enCPSState_FTFreeDriver,
            RbtFSM.enCPSState_ScriptRunning,
            RbtFSM.enCPSState_ScriptHoldHandling,
            RbtFSM.enCPSState_ScriptHolding,
            RbtFSM.enCPSState_ScriptStopping,
            RbtFSM.enCPSState_ScriptStopped,
        }
        self.UNENABLE_STATES = {
            RbtFSM.enCPSState_Initialize,
            RbtFSM.enCPSState_Initialize,
            RbtFSM.enCPSState_Reseting,
            RbtFSM.enCPSState_Enabling,
            RbtFSM.enCPSState_Disable,
            RbtFSM.enCPSState_RobotLoadIdentify,
            RbtFSM.enCPSState_Braking,
            RbtFSM.enCPSState_FTOpeningFreeDriver,
            RbtFSM.enCPSState_FTClosingFreeDriver,
            RbtFSM.enCPSState_FTFreeDriver,
        }

    def _read_config(self):
        """读取配置文件"""
        config_key = "fleximind_robot_node"
        self.boxid_left = rospy.get_param(f"~{config_key}/boxid_left")
        self.boxid_right = rospy.get_param(f"~{config_key}/boxid_right")
        self.ip_left = rospy.get_param(f"~{config_key}/ip_left")
        self.ip_right = rospy.get_param(f"~{config_key}/ip_right")
        self.port = rospy.get_param(f"~{config_key}/port")
        self.Armtype = rospy.get_param(f"~{config_key}/arm_type")
        self.monitor_port = rospy.get_param(f"~{config_key}/monitor_port")
        self.left_joint_names = rospy.get_param(f"~{config_key}/left_joint_names")
        self.right_joint_names = rospy.get_param(f"~{config_key}/right_joint_names")
        self.stop_status = rospy.get_param(f"~{config_key}/stop_status", False)
        self.timer_period = rospy.get_param(f"~{config_key}/timer_period", 0.1)

        servo_control_key = "servo_control"
        self.interp_ratios = rospy.get_param(
            f"~{config_key}/{servo_control_key}/interp_ratios", 3
        )
        self.max_velocity = rospy.get_param(
            f"~{config_key}/{servo_control_key}/max_velocity", 1.0
        )
        self.max_accel = rospy.get_param(
            f"~{config_key}/{servo_control_key}/max_accel", 0.5
        )
        self.max_jerk = rospy.get_param(
            f"~{config_key}/{servo_control_key}/max_jerk", 2.0
        )
        self.serve_time = rospy.get_param(
            f"~{config_key}/{servo_control_key}/serve_time", 0.004
        )
        self.dp = rospy.get_param(f"~{config_key}/{servo_control_key}/dp", 8000)
        self.lookahead_time = rospy.get_param(
            f"~{config_key}/{servo_control_key}/lookahead_time", 0.3
        )
        self.buffer_size = rospy.get_param(
            f"~{config_key}/{servo_control_key}/buffer_size", 1000
        )

        # 读取move_control参数
        move_control_key = "move_control"
        self.movej_frequency = rospy.get_param(
            f"~{config_key}/{move_control_key}/movej_frequency", 10
        )
        self.log_dir = rospy.get_param(
            f"~{config_key}/{move_control_key}/log_dir",
            "/home/robot/fleximind-ros1/servo_logs",
        )
        self.min_path_length = rospy.get_param(
            f"~{config_key}/{move_control_key}/min_path_length", 2.0
        )
        self.max_angle_threshold = rospy.get_param(
            f"~{config_key}/{move_control_key}/max_angle_threshold", 120.0
        )
        self.extreme_angle_threshold = rospy.get_param(
            f"~{config_key}/{move_control_key}/extreme_angle_threshold", 175.0
        )
        self.curvature_threshold = rospy.get_param(
            f"~{config_key}/{move_control_key}/curvature_threshold", 0.8
        )
        self.angle_diff_threshold = rospy.get_param(
            f"~{config_key}/{move_control_key}/angle_diff_threshold", 1.0
        )
        rospy.loginfo("配置文件读取成功")

    # === 创建服务 === #
    def _create_services(self):
        """创建服务"""
        services = [
            (GetIP, RobotInterface.GET_IP, self.get_ip_callback),
            (RobotBase, RobotInterface.RESET, self.reset_callback),
            (RobotBase, RobotInterface.EMERGENCY_STOP, self.emergency_stop_callback),
            (RobotBase, RobotInterface.START, self.start_callback),
            (RobotBase, RobotInterface.POWER_OUTAGE, self.power_outage_callback),
            (RobotBase, RobotInterface.ENABLE, self.enable_callback),
            (RobotBase, RobotInterface.DISABLE, self.disable_callback),
            (RobotBase, RobotInterface.SETSPEED, self.setoverride_callback),
            (PublishingSwitch, RobotInterface.SERVOJ, self.servoJ_callback),
            (
                PublishingSwitch,
                RobotInterface.DATA_PUB,
                self.data_publishing_switch_callback,
            ),
            (
                SingleArmControl,
                "/status/left_hand/reconnect",
                self.left_arm_reconnect_callback,
            ),
            (
                SingleArmControl,
                "/status/right_hand/reconnect",
                self.right_arm_reconnect_callback,
            ),
            (FollowingType, "/robot/following_type", self.following_type_callback),
        ]

        self.service_list = []
        for srv_type, srv_name, callback in services:
            # self.service_list.append(rospy.Service(srv_name.value, srv_type, callback))
            if hasattr(srv_name, "value"):
                service_name = srv_name.value
            else:
                service_name = srv_name
            self.service_list.append(rospy.Service(service_name, srv_type, callback))

    # === 创建服务 === #

    # === 服务回调方法 === #
    def left_arm_reconnect_callback(self, req):
        from fleximind_robot.srv import SingleArmControlResponse

        response = SingleArmControlResponse()
        try:
            rospy.loginfo("Received left arm reset request")
            success = self.arm_left.reset()
            if success:
                response.code = DualRobotState.enRobotState_ResponseOK
                response.message = "Left arm reset successfully"
                rospy.loginfo("Left arm reset successfully")
            else:
                response.code = DualRobotState.enRobotState_ResetFailed
                response.message = "Left arm reset failed"
                rospy.logwarn("Left arm reset failed")
        except Exception as e:
            response.code = DualRobotState.enRobotState_ResponseError
            response.message = f"Left arm reset exception: {str(e)}"
            rospy.logerr(f"Left arm reset exception: {str(e)}")
        return response

    def right_arm_reconnect_callback(self, req):
        from fleximind_robot.srv import SingleArmControlResponse

        response = SingleArmControlResponse()
        try:
            rospy.loginfo("Received right arm reset request")
            success = self.arm_right.reset()
            if success:
                response.code = DualRobotState.enRobotState_ResponseOK
                response.message = "Right arm reset successfully"
                rospy.loginfo("Right arm reset successfully")
            else:
                response.code = DualRobotState.enRobotState_ResetFailed
                response.message = "Right arm reset failed"
                rospy.logwarn("Right arm reset failed")
        except Exception as e:
            response.code = DualRobotState.enRobotState_ResponseError
            response.message = f"Right arm reset exception: {str(e)}"
            rospy.logerr(f"Right arm reset exception: {str(e)}")
        return response

    def get_ip_callback(self, req):
        response = GetIPResponse()
        try:
            response.code = 200
            response.left_ip = self.ip_left
            response.right_ip = self.ip_right
            response.port = self.port
        except Exception as e:
            response.code = 500
            response.message = str(e)
        return response

    def reset_callback(self, req):
        response = RobotBaseResponse()
        try:
            left_success = self.arm_left.reset()
            right_success = self.arm_right.reset()

            if left_success and right_success:
                response.code = DualRobotState.enRobotState_ResponseOK
                response.message = "双臂重置成功"
            else:
                response.code = DualRobotState.enRobotState_ResetFailed
                response.message = (
                    f"双臂重置失败,左臂:{left_success},右臂:{right_success}"
                )
        except Exception as e:
            response.code = DualRobotState.enRobotState_ResponseError
            response.message = str(e)
        return response

    def emergency_stop_callback(self, req):
        response = RobotBaseResponse()
        try:
            left_success = self.arm_left.stop()
            right_success = self.arm_right.stop()

            if left_success and right_success:
                response.code = DualRobotState.enRobotState_ResponseOK
                response.message = "双臂已紧急停止"
            else:
                response.code = DualRobotState.enRobotState_EmergencyStopFailed
                response.message = (
                    f"双臂紧急停止失败,左臂:{left_success},右臂:{right_success}"
                )
        except Exception as e:
            response.code = DualRobotState.enRobotState_ResponseError
            response.message = str(e)
        return response

    def start_callback(self, req):
        response = RobotBaseResponse()
        try:
            left_power = self.arm_left.electrify()
            right_power = self.arm_right.electrify()
            time.sleep(10.0)  # 等待电源启动
            if left_power and right_power:
                response.code = DualRobotState.enRobotState_ResponseOK
                response.message = "双臂上电成功"
            else:
                response.code = DualRobotState.enRobotState_ElectricFailed
                response.message = f"双臂上电失败 左臂:{left_power}, 右臂:{right_power}"
                return response

            # 分别连接控制器
            left_connect = self.arm_left.connect_controller()
            right_connect = self.arm_right.connect_controller()

            if left_connect and right_connect:
                response.code = DualRobotState.enRobotState_ResponseOK
                response.message = "机械臂启动成功"
                self._publish_uninitialized_status()

                rospy.loginfo("上电完成，检查系统状态...")

                max_retries = 3
                for i in range(max_retries):
                    time.sleep(8.0)  # 增加等待时间
                    current_state = self.read_dual_robot_state()
                    rospy.loginfo(f"第{i+1}次状态检查: {current_state}")

                    if current_state == DualRobotState.enRobotState_Ready:
                        rospy.loginfo("机械臂已进入就绪状态")
                        response.message = "机械臂启动成功并进入就绪状态"
                        break
                    elif current_state == DualRobotState.enRobotState_NotReady:
                        # 未就绪状态是正常的，继续等待
                        rospy.loginfo("机械臂正在初始化中...")
                        continue
                    elif current_state == DualRobotState.enRobotState_Error:
                        rospy.logwarn("检测到错误状态，尝试重置...")
                        left_reset = self.arm_left.reset()
                        right_reset = self.arm_right.reset()

                        if left_reset and right_reset:
                            rospy.loginfo("双臂重置成功")
                            response.message = (
                                "机械臂启动成功，检测到错误状态已自动重置"
                            )
                            break
                        else:
                            rospy.logwarn(
                                f"第{i+1}次重置失败 左臂:{left_reset}, 右臂:{right_reset}"
                            )
                            if i == max_retries - 1:  # 最后一次尝试
                                response.code = DualRobotState.enRobotState_ResetFailed
                                response.message = f"机械臂启动成功，但错误状态重置失败 左臂:{left_reset}, 右臂:{right_reset}"
                else:
                    rospy.loginfo("机械臂启动过程完成")
            else:
                response.code = DualRobotState.enRobotState_RobotStartFailed
                response.message = (
                    f"机械臂启动失败 左臂:{left_connect}, 右臂:{right_connect}"
                )
        except Exception as e:
            response.code = DualRobotState.enRobotState_ResponseError
            response.message = f"启动过程中发生错误: {str(e)}"
        return response

    def power_outage_callback(self, req):
        response = RobotBaseResponse()
        try:
            rospy.loginfo("开始执行断电操作...")

            # 读取当前状态
            left_fsm_result = []
            right_fsm_result = []
            self.arm_left.sdk.HRIF_ReadCurFSM(self.arm_left.box_id, 0, left_fsm_result)
            self.arm_right.sdk.HRIF_ReadCurFSM(
                self.arm_right.box_id, 0, right_fsm_result
            )

            rospy.loginfo(
                f"断电前状态 - 左臂FSM: {left_fsm_result}, 右臂FSM: {right_fsm_result}"
            )

            # 如果处于就绪状态（33），需要先禁用再断电
            left_need_disable = (
                left_fsm_result and int(left_fsm_result[0]) == RbtFSM.enCPSState_StandBy
            )
            right_need_disable = (
                right_fsm_result
                and int(right_fsm_result[0]) == RbtFSM.enCPSState_StandBy
            )

            if left_need_disable or right_need_disable:
                rospy.loginfo("机械臂处于就绪状态，先执行禁用操作...")

                # 执行禁用
                if left_need_disable:
                    rospy.loginfo("禁用左臂...")
                    self.arm_left.disable()
                if right_need_disable:
                    rospy.loginfo("禁用右臂...")
                    self.arm_right.disable()

                # 等待状态切换
                time.sleep(2.0)

                # 验证禁用状态
                left_fsm_after_disable = []
                right_fsm_after_disable = []
                self.arm_left.sdk.HRIF_ReadCurFSM(
                    self.arm_left.box_id, 0, left_fsm_after_disable
                )
                self.arm_right.sdk.HRIF_ReadCurFSM(
                    self.arm_right.box_id, 0, right_fsm_after_disable
                )

                rospy.loginfo(
                    f"禁用后状态 - 左臂FSM: {left_fsm_after_disable}, 右臂FSM: {right_fsm_after_disable}"
                )

            # 执行断电
            rospy.loginfo("执行断电操作...")
            left_success = self.arm_left.blackout()
            right_success = self.arm_right.blackout()

            rospy.loginfo(f"断电操作结果 - 左臂: {left_success}, 右臂: {right_success}")

            # 验证断电结果
            time.sleep(2.0)
            left_fsm_after = []
            right_fsm_after = []
            self.arm_left.sdk.HRIF_ReadCurFSM(self.arm_left.box_id, 0, left_fsm_after)
            self.arm_right.sdk.HRIF_ReadCurFSM(
                self.arm_right.box_id, 0, right_fsm_after
            )

            rospy.loginfo(
                f"断电后状态 - 左臂FSM: {left_fsm_after}, 右臂FSM: {right_fsm_after}"
            )

            if left_success and right_success:
                response.code = DualRobotState.enRobotState_ResponseOK
                response.message = "双臂断电成功"
                self._publish_uninitialized_status()
            else:
                response.code = DualRobotState.enRobotState_RobotPowerOutageFailed
                response.message = (
                    f"双臂断电失败 左臂:{left_success}, 右臂:{right_success}"
                )

        except Exception as e:
            rospy.logerr(f"断电操作异常: {str(e)}")
            response.code = DualRobotState.enRobotState_ResponseError
            response.message = f"断电过程中发生异常: {str(e)}"
        return response

    def enable_callback(self, req):
        response = RobotBaseResponse()
        try:
            left_success = self.arm_left.enable()
            right_success = self.arm_right.enable()

            if left_success and right_success:
                response.code = DualRobotState.enRobotState_ResponseOK
                response.message = "双臂使能成功"
                self._publish_standby_status()
            else:
                response.code = DualRobotState.enRobotState_RobotEnableFailed
                response.message = (
                    f"双臂使能失败 左臂:{left_success}, 右臂:{right_success}"
                )
        except Exception as e:
            response.code = DualRobotState.enRobotState_ResponseError
            response.message = str(e)
        return response

    def disable_callback(self, req):
        response = RobotBaseResponse()
        try:
            # 分别禁用左右臂
            left_success = self.arm_left.disable()
            right_success = self.arm_right.disable()

            if left_success and right_success:
                response.code = DualRobotState.enRobotState_ResponseOK
                response.message = "双臂去使能成功"
            else:
                response.code = DualRobotState.enRobotState_RobotDisableFailed
                response.message = (
                    f"双臂去使能失败 左臂:{left_success}, 右臂:{right_success}"
                )
        except Exception as e:
            response.code = DualRobotState.enRobotState_ResponseError
            response.message = str(e)
        return response

    def data_publishing_switch_callback(self, req: PublishingSwitchRequest):
        response = PublishingSwitchResponse()
        try:
            self.is_publishing = req.is_open
            response.code = 0
            response.message = f"数据发布开关设置为: {self.is_publishing}"
        except Exception as e:
            response.code = 500
            response.message = str(e)
        return response

    def servoJ_callback(self, req: PublishingSwitchRequest):
        response = PublishingSwitchResponse()
        try:
            # 根据跟随模式选择不同的控制器
            if hasattr(self, "current_following_type"):
                following_type = self.current_following_type
            else:
                # 默认使用servoj模式（保持向后兼容）
                following_type = 0

            if following_type == 0:
                # servoj模式 - 使用原有逻辑
                self.servo_left.set_sending(req.is_open)
                self.servo_right.set_sending(req.is_open)
                if req.is_open:
                    self.servo_left.start()
                    self.servo_right.start()
                    self._create_subscribers()
                else:
                    self.servo_left.stop()
                    self.servo_right.stop()
                    if hasattr(self, "subscription_remote_sensing"):
                        self.subscription_remote_sensing.unregister()

                response.code = 0
                response.message = "Servoj控制开关已设置"

            elif following_type == 1:
                # movej模式 - 使用新的movej控制器
                self.move_left.set_sending(req.is_open)
                self.move_right.set_sending(req.is_open)
                if req.is_open:
                    self.move_left.start()
                    self.move_right.start()
                    self._create_subscribers()
                else:
                    self.move_left.stop()
                    self.move_right.stop()
                    if hasattr(self, "subscription_remote_sensing"):
                        self.subscription_remote_sensing.unregister()

                response.code = 0
                response.message = "Movej控制开关已设置"
            else:
                response.code = 400
                response.message = f"不支持的跟随模式: {following_type}"

        except Exception as e:
            response.code = 500
            response.message = str(e)
        return response

    def following_type_callback(self, req):
        """跟随模式切换回调函数"""
        from fleximind_robot.srv import FollowingTypeResponse

        response = FollowingTypeResponse()

        try:
            following_type = req.type  # 0: servoj, 1: movej

            rospy.loginfo(f"接收到跟随模式切换请求: type={following_type}")

            if following_type not in [0, 1]:
                response.code = 400
                response.message = (
                    f"无效的跟随模式: {following_type}, 必须为 0 (servoj) 或 1 (movej)"
                )
                return response

            # 停止当前激活的控制器
            if (
                hasattr(self, "current_following_type")
                and self.current_following_type == 0
            ):
                # 当前是servoj模式，停止servoj控制器
                if hasattr(self, "servo_left") and hasattr(self, "servo_right"):
                    if (
                        hasattr(self.servo_left, "is_sending")
                        and self.servo_left.is_sending
                    ):
                        self.servo_left.stop()
                        self.servo_left.set_sending(False)
                    if (
                        hasattr(self.servo_right, "is_sending")
                        and self.servo_right.is_sending
                    ):
                        self.servo_right.stop()
                        self.servo_right.set_sending(False)
            elif (
                hasattr(self, "current_following_type")
                and self.current_following_type == 1
            ):
                # 当前是movej模式，停止movej控制器
                if hasattr(self, "move_left") and hasattr(self, "move_right"):
                    if (
                        hasattr(self.move_left, "is_sending")
                        and self.move_left.is_sending
                    ):
                        self.move_left.stop()
                        self.move_left.set_sending(False)
                    if (
                        hasattr(self.move_right, "is_sending")
                        and self.move_right.is_sending
                    ):
                        self.move_right.stop()
                        self.move_right.set_sending(False)

            # 更新模式
            self.current_following_type = following_type

            response.code = 200
            response.message = f"Following type changed to {'servoj' if following_type == 0 else 'movej'} successfully"
            rospy.loginfo(
                f"已切换到{'servoj' if following_type == 0 else 'movej'}跟随模式"
            )

        except Exception as e:
            rospy.logerr(f"跟随模式切换异常: {str(e)}")
            response.code = 500
            response.message = f"Following type change failed: {str(e)}"

        return response

    def setoverride_callback(self, req):
        response = RobotBaseResponse()
        try:
            left_success = self.arm_left.setspeed()
            right_success = self.arm_right.setspeed()

            if left_success and right_success:
                response.code = DualRobotState.enRobotState_ResponseOK
                response.message = "双臂设置速度成功"
            else:
                response.code = DualRobotState.enRobotState_RobotDisableFailed
                response.message = (
                    f"双臂设置速度失败 左臂:{left_success}, 右臂:{right_success}"
                )
        except Exception as e:
            response.code = DualRobotState.enRobotState_ResponseError
            response.message = str(e)
        return response

    # === 服务回调方法 === #

    # === 创建动作 === #
    def _create_actions(self):
        """创建动作"""
        self.long_jog_action = actionlib.SimpleActionServer(
            RobotInterface.MOVE_JOINTS.value,
            RobotLongJogJAction,
            execute_cb=self.long_jog_callback,
            auto_start=False,
        )
        self.long_jog_action.start()

    # === 创建动作 === #

    # === 动作回调方法 === #
    def long_jog_callback(self, goal: RobotLongJogJGoal):
        # 收到新目标，开始执行
        result = RobotLongJogJFeedback()
        feedback = RobotLongJogJResult()

        self.arm_left.longJogJ(goal.left_boxID, goal.left_rbtID, goal.left_joints)
        self.arm_right.longJogJ(goal.right_boxID, goal.right_rbtID, goal.right_joints)

        # 开始长点动循环
        rate = rospy.Rate(3)  # 3Hz

        while True:
            # 检查预占请求（取消信号）
            if self.long_jog_action.is_preempt_requested():
                rospy.logwarn("收到取消请求，终止长点动")
                self.long_jog_action.set_preempted()
                break

            # 1. 控制机器人按指定方向移动
            left_success = self.arm_left.longMoveEvent(goal.left_boxID, goal.left_rbtID)
            right_success = self.arm_right.longMoveEvent(
                goal.right_boxID, goal.right_rbtID
            )

            if not left_success or not right_success:
                rospy.logwarn(
                    f"Long move event failed: left_success={left_success}, right_success={right_success}"
                )

            # 2. 检查运动是否完成
            result_l, result_r = [], []
            left_done = self.arm_left.isArrivedJoints(
                goal.left_boxID, goal.left_rbtID, goal.left_joints, result_l
            )
            right_done = self.arm_right.isArrivedJoints(
                goal.right_boxID, goal.right_rbtID, goal.right_joints, result_r
            )

            if left_done and right_done:
                rospy.loginfo("Long move event completed successfully.")
                break

            # 3. 发布反馈
            feedback.code = DualRobotState.enRobotState_ResponseOK
            feedback.message = "Long move event in progress."
            self.long_jog_action.publish_feedback(feedback)
            rate.sleep()
        else:
            # 循环结束
            result.code = DualRobotState.enRobotState_ResponseOK
            result.message = "long jog J completed successfully."
            self.long_jog_action.set_succeeded(result)

        self.arm_left.stop()
        self.arm_right.stop()

    # === 动作回调方法 === #

    # === 创建发布者 === #
    def _create_publishers(self):
        """创建发布者"""
        self.robot_state_pub = rospy.Publisher(
            RobotInterface.STATUS.value, ConnectionStatus, queue_size=10
        )
        self.robot_info_pub = rospy.Publisher(
            RobotInterface.INFO.value, RobotInfo, queue_size=10
        )

    def timer_callback(self, event):
        """定时器回调（发布左右臂详细Robot信息）"""
        self.robot_state_pub.publish(self.device_state)

        if self.is_publishing:
            # 获取左右臂数据
            core_data_l = self.robot_monitor_l.get_core_data()
            core_data_r = self.robot_monitor_r.get_core_data()

            # 构造左臂Robot消息
            left_robot = Robot()
            left_robot.robotState = core_data_l.get("robotState", 0)
            left_robot.robotEnabled = core_data_l.get("robotEnabled", False)
            left_robot.robotMoving = core_data_l.get("robotMoving", False)
            left_robot.Error_AxisID = core_data_l.get("Error_AxisID", 0)
            left_robot.Error_Code = core_data_l.get("Error_Code", 0)
            left_robot.joint_positions = core_data_l.get("joint_positions", [0.0] * 6)
            left_robot.eef_positions = core_data_l.get("eef_positions", [0.0] * 6)
            left_robot.BoxCI = core_data_l.get("BoxCI", [0] * 8)
            left_robot.BoxCO = core_data_l.get("BoxCO", [0] * 8)
            left_robot.BoxDI = core_data_l.get("BoxDI", [0] * 8)
            left_robot.BoxDO = core_data_l.get("BoxDO", [0] * 8)
            left_robot.EndDI = core_data_l.get("EndDI", [0] * 4)
            left_robot.EndDO = core_data_l.get("EndDO", [0] * 4)
            left_robot.last_update = core_data_l.get("last_update", "")

            # 构造右臂Robot消息
            right_robot = Robot()
            right_robot.robotState = core_data_r.get("robotState", 0)
            right_robot.robotEnabled = core_data_r.get("robotEnabled", False)
            right_robot.robotMoving = core_data_r.get("robotMoving", False)
            right_robot.Error_AxisID = core_data_r.get("Error_AxisID", 0)
            right_robot.Error_Code = core_data_r.get("Error_Code", 0)
            right_robot.joint_positions = core_data_r.get("joint_positions", [0.0] * 6)
            right_robot.eef_positions = core_data_l.get("eef_positions", [0.0] * 6)
            right_robot.BoxCI = core_data_r.get("BoxCI", [0] * 8)
            right_robot.BoxCO = core_data_r.get("BoxCO", [0] * 8)
            right_robot.BoxDI = core_data_r.get("BoxDI", [0] * 8)
            right_robot.BoxDO = core_data_r.get("BoxDO", [0] * 8)
            right_robot.EndDI = core_data_r.get("EndDI", [0] * 4)
            right_robot.EndDO = core_data_r.get("EndDO", [0] * 4)
            right_robot.last_update = core_data_r.get("last_update", "")

            # 构造RobotInfo消息
            robot_info = RobotInfo()
            robot_info.header.stamp = rospy.Time.now()
            robot_info.left_robot = left_robot
            robot_info.right_robot = right_robot

            # 发布消息
            self.robot_info_pub.publish(robot_info)

        system_state = self.read_dual_robot_state()
        state_msg = Int32()
        state_msg.data = int(system_state)
        self.state_pub.publish(state_msg)

    # === 创建发布者 === #

    def read_dual_robot_state(self):
        """
        状态检测
        返回值:
            0 - 就绪状态(仅33)
            1 - 未上电状态
            2 - 未就绪状态（已上电但未达到就绪条件）
            3 - 错误/急停状态
        """
        try:
            if self.stop_status:
                self.stop_status = False
                return DualRobotState.enRobotState_Error  # 停止状态优先检测

            if self.Armtype == 0:  # 双臂模式
                state_l = self.robot_monitor_l.get_core_data().get("robotState", 0)
                state_r = self.robot_monitor_r.get_core_data().get("robotState", 0)
                # 空值检查
                if state_l is None or state_r is None:
                    return DualRobotState.enRobotState_Error

                # 1. 错误状态检测（最高优先级）
                if state_l in self.ERROR_STATES or state_r in self.ERROR_STATES:
                    return DualRobotState.enRobotState_Error

                # 2. 未上电状态检测
                if state_l in self.POWER_OFF_STATES or state_r in self.POWER_OFF_STATES:
                    return DualRobotState.enRobotState_PowerOff

                # 3. 就绪状态检测（严格匹配33）
                if state_l in self.READY_STATES and state_r in self.READY_STATES:
                    return DualRobotState.enRobotState_Ready

                # 4. 其他情况均为未就绪状态
                return DualRobotState.enRobotState_NotReady
            elif self.Armtype == 3:  # 不监控状态
                return DualRobotState.enRobotState_NotReady
            else:
                rospy.logerr(f"无效的Armtype值: {self.Armtype}")
                return DualRobotState.enRobotState_Error

        except Exception as e:
            rospy.logerr(f"状态读取异常: {str(e)}")
            return DualRobotState.enRobotState_Error

    # === 创建订阅者 === #
    def _create_subscribers(self):
        """创建订阅者,订阅遥感数据"""
        self.subscription_remote_sensing = rospy.Subscriber(
            RemoteControlInterface.JOINT_STATES.value,
            DualJointStates,
            self.remote_control_callback,
            queue_size=1,  # 缓冲区大会增加延时
        )

    def remote_control_callback(self, msg: DualJointStates):
        """遥控器控制回调 - 根据当前模式智能路由到对应的控制器"""
        try:
            if not hasattr(msg, "left_joints") or not hasattr(msg, "right_joints"):
                rospy.logwarn("遥控器消息缺少关节字段，忽略")
                return

            if len(msg.left_joints.position) < 6 or len(msg.right_joints.position) < 6:
                rospy.logwarn("遥控器关节数不足6个，忽略该消息")
                return

            # 取前六个关节数据
            left_pos = list(msg.left_joints.position)[:6]
            right_pos = list(msg.right_joints.position)[:6]

            # 根据当前跟随模式路由到对应的控制器
            if hasattr(self, "current_following_type"):
                following_type = self.current_following_type
            else:
                following_type = 0  # 默认servoj模式

            if (
                following_type == 0
                and hasattr(self, "servo_left")
                and hasattr(self, "servo_right")
            ):
                # servoj模式
                if (
                    hasattr(self.servo_left, "is_sending")
                    and self.servo_left.is_sending
                    and hasattr(self.servo_right, "is_sending")
                    and self.servo_right.is_sending
                ):
                    self.servo_left.add_point(left_pos)
                    self.servo_right.add_point(right_pos)

            elif (
                following_type == 1
                and hasattr(self, "move_left")
                and hasattr(self, "move_right")
            ):
                # movej模式
                if (
                    hasattr(self.move_left, "is_sending")
                    and self.move_left.is_sending
                    and hasattr(self.move_right, "is_sending")
                    and self.move_right.is_sending
                ):
                    self.move_left.add_point(left_pos)
                    self.move_right.add_point(right_pos)
            else:
                rospy.logwarn(f"当前模式{following_type}对应的控制器未就绪")

        except Exception as e:
            rospy.logerr(f"遥控器控制回调异常: {str(e)}")

    def _publish_standby_status(self):
        """发布待机状态"""
        try:
            from fleximind_remote import SystemState
            from fleximind_remote.msg import RemoteControlStatus

            status_msg = RemoteControlStatus()
            status_msg.system_status = SystemState.STANDBY.value

            self.state_publisher.publish(status_msg)
            rospy.loginfo("已发布待机状态")

        except Exception as e:
            rospy.logerr(f"发布待机状态失败: {e}")

    def _publish_uninitialized_status(self):
        """发布未初始化状态"""
        try:
            from fleximind_remote import SystemState
            from fleximind_remote.msg import RemoteControlStatus

            status_msg = RemoteControlStatus()
            status_msg.system_status = SystemState.UNINITIALIZED.value

            self.state_publisher.publish(status_msg)
            rospy.loginfo("已发布未初始化状态")

        except Exception as e:
            rospy.logerr(f"发布未初始化状态失败: {e}")


# === 创建订阅者 === #
def main():
    rospy.loginfo("Starting Dual Robot Control Node")
    node = DualRobotControlNode()

    try:
        rospy.spin()  # ROS1主线程阻塞spin
    except KeyboardInterrupt:
        rospy.loginfo("用户终止节点")
    except Exception as e:
        rospy.logerr(f"节点运行错误: {str(e)}")
    finally:
        node.__del__()
        rospy.signal_shutdown("Node shutdown")


if __name__ == "__main__":
    main()
