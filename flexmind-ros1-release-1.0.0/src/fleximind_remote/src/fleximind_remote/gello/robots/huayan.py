from typing import Dict
import numpy as np
from fleximind_remote.gello.robots.robot import Robot, BimanualRobot
from fleximind_remote.gello.robots.servo_plan import main
import logging
import time
from scipy.spatial.transform import Rotation
import os
import sys
import time
import math
import csv
import socket
import numpy as np
from fleximind_remote.gello.robots.CPS import CPSClient
from typing import List, Tuple, Union

LEFT_IP = "192.168.1.20"
RIGHT_IP = "192.168.1.30"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [%(threadName)s] %(message)s",
    handlers=[logging.StreamHandler()],
)


class CPSRobot(Robot):
    # Class constants
    JOINT_LIMITS = np.array(
        [
            [-np.radians(170), np.radians(170)],  # j1
            [-np.radians(90), np.radians(90)],  # j2
            [-np.radians(170), np.radians(170)],  # j3
            [-np.radians(190), np.radians(190)],  # j4
            [-np.radians(170), np.radians(170)],  # j5
            [-np.radians(360), np.radians(360)],  # j6
        ],
        dtype=np.float32,
    )

    DEFAULT_SERVO_TIME = 0.020  # 20ms cycle
    DEFAULT_LOOKAHEAD_TIME = 0.1  # 100ms lookahead

    def __init__(
        self,
        ip: str = LEFT_IP,
        port: int = 10003,
        servo_time: float = DEFAULT_SERVO_TIME,
        lookahead_time: float = DEFAULT_LOOKAHEAD_TIME,
    ):
        """
        Initialize CPS robot controller

        :param ip: Controller IP address
        :param port: Controller port
        :param servo_time: Servo control cycle in seconds (default 0.020)
        :param lookahead_time: Lookahead time in seconds (default 0.1)
        """
        self.logger = logging.getLogger("CPSRobot")
        self.logger.info(f"Initializing robot controller | IP: {ip} | Port: {port}")

        # Configuration
        self.ip = ip
        self.port = port
        self.servo_time = servo_time
        self.lookahead_time = lookahead_time
        self.read_error_count = 0
        self.servoj_error_count = 0

        # Control parameters
        if ip == LEFT_IP:
            self.box_id = 0  # Controller ID
        else:
            self.box_id = 1
        self.rbt_id = 0  # Robot ID

        # State variables
        self.joint_positions = np.zeros(6, dtype=np.float32)
        self.tcp_position = np.zeros(3, dtype=np.float32)
        self.tcp_orientation = np.zeros(3, dtype=np.float32)
        self.gripper_position = 0.0

        # Control flags
        self._running = True
        self._servo_active = False
        self.starting = 0
        self.starting_joint = None
        # Initialize SDK
        try:
            from fleximind_remote.gello.robots.CPS import CPSClient

            self.sdk = CPSClient()
            # self.sdk =SDK
            self.connected = self._connect_to_robot()
        except ImportError as e:
            self.logger.error(f"CPS SDK not found: {str(e)}")
            raise RuntimeError("Missing CPS SDK dependency")

    def _connect_to_robot(self) -> bool:
        # try:
        #    self.sdk.HRIF_DisConnect(self.box_id)
        # except Exception as e:
        #    print(str(e))
        #    pass
        """Connect to robot controller"""
        self._servo_active = False
        self.logger.info("Connecting to robot...")
        try:
            nRet = self.sdk.HRIF_Connect(self.box_id, self.ip, self.port)
            if nRet != 0:
                error_msg = f"{self.ip}HRIF_Connect failed (Error code: {nRet})"
                self.logger.error(error_msg)
                return False
            self.logger.info(f"{self.ip}Connection successful")
            return True
        except Exception as e:
            self.logger.error(f"{self.ip}Connection error: {str(e)}")
            return False

    def _GrpEnable(self) -> bool:
        # 使能
        self._servo_active = False
        nRet = self.sdk.HRIF_GrpEnable(self.box_id, self.rbt_id)
        if nRet != 0:
            return False
        return True

    def _Electrify(self) -> bool:
        # 上电
        self._servo_active = False
        nRet = self.sdk.HRIF_Electrify(self.box_id)
        if nRet != 0:
            return False
        return True

    def _init_servo_mode(self) -> bool:
        """Initialize servo mode"""
        self.logger.info(f"{self.ip} Initializing servo mode...")
        try:
            nRet = self.sdk.HRIF_StartServo(
                self.box_id, self.rbt_id, self.servo_time, self.lookahead_time
            )

            if nRet != 0:
                # self.read_robot_state()
                return False

            self._servo_active = True
            self.logger.info("Servo mode activated")
            return True
        except Exception as e:
            self.logger.error(f"Servo initialization error: {str(e)}")
            return False

    def read_robot_state(self):
        """
        调用 HRIF_ReadRobotState 接口读取机器人状态并详细打印结果

        参数:
            box_id: 电箱ID (0~5, 默认0)
            robot_id: 机器人ID (0~5, 默认0)
        """
        # 定义返回值空列表
        result = []

        # 调用接口
        try:
            nRet = self.sdk.HRIF_ReadRobotState(self.box_id, self.rbt_id, result)
        except Exception as e:
            print(f"调用接口失败: {str(e)}")
            return [nRet]

        if nRet != 0:
            print(f"警告: 接口调用失败，错误码: {nRet}")
            self.read_error_count += 1
            if self.read_error_count > 9:  # 连续读取错误，怀疑连接断开，尝试重连
                self._connect_to_robot()
            return [nRet]
        else:
            self.read_error_count = 0

        if len(result) < 13:
            print(
                f"错误: 返回结果长度不足，期望13，实际得到{len(result)},得到的结果是 {result}"
            )
            return result

        # 检查错误状态
        if result[2] == "1":
            print("\n====== 错误详情 ======")
            print(f"错误码: {result[3]}")
            print(f"错误轴ID: {result[4]}")

        print("\n====== 状态摘要 ======")
        if result[7] == "1":
            print("⚠️ 急停状态激活!")
        if result[2] == "1":
            print("⚠️ 机器人发生错误!")
        if result[5] == "1":
            print("⚠️ 抱闸松闸状态 - 注意轴可能掉落!")
        if result[10] == "0":
            print("⚠️ 电箱未连接!")
        if result[9] == "0":
            print("⚠️ 机器人未上电!")
        else:
            print("无机器人无明显异常状态，一切就绪")

        return result

    def _update_robot_state(self):
        """Update robot state from controller"""
        try:
            result = []
            nRet = self.sdk.HRIF_ReadActPos(self.box_id, self.rbt_id, result)

            if nRet != 0 or len(result) < 24:
                print(f"HRIF_ReadActPos failed (Error code: {nRet})")
                return

            # Parse joint positions (degrees to radians)
            self.joint_positions = np.deg2rad(
                np.array(
                    [
                        float(result[0]),
                        float(result[1]),
                        float(result[2]),
                        float(result[3]),
                        float(result[4]),
                        float(result[5]),
                    ],
                    dtype=np.float32,
                )
            )

            # TCP position (mm to meters)
            self.tcp_position = np.array(
                [
                    float(result[6]) / 1000.0,
                    float(result[7]) / 1000.0,
                    float(result[8]) / 1000.0,
                ],
                dtype=np.float32,
            )

            # TCP orientation (degrees to radians)
            self.tcp_orientation = np.deg2rad(
                np.array(
                    [float(result[9]), float(result[10]), float(result[11])],
                    dtype=np.float32,
                )
            )

        except Exception as e:
            self.logger.error(f"State update failed: {str(e)}")

    # --------------------- Robot Protocol ---------------------
    def num_dofs(self) -> int:
        return 7  # 6 real joints + 1 virtual gripper

    def get_joint_state(self) -> np.ndarray:
        self._update_robot_state()
        return np.append(self.joint_positions, self.gripper_position)

    def command_joint_state(self, joint_state: np.ndarray) -> bool:
        # if not self._servo_active:
        #    nRet=self._init_servo_mode()
        #    if not nRet:
        #        return False

        # if joint_state.size != 7:
        #    self.logger.warning(f"Invalid joint state dimension: {joint_state.size}, expected 7")
        #    return False

        target_joints = joint_state[:6].astype(np.float32)

        # if not np.all(np.isfinite(target_joints)):
        #    self.logger.error("Invalid joint positions (contains NaN or inf)")
        #    return False

        target_deg = np.degrees(target_joints).tolist()

        nRet = self.sdk.HRIF_PushServoJ(self.box_id, self.rbt_id, target_deg)

        if nRet != 0:
            print(f"HRIF_PushServoJ failed (Error code: {nRet})")
            #    self.servoj_error_count+=1
            #    if self.servoj_error_count>4:
            #        self._servo_active=False
            #        self.servoj_error_count=0
            return False
        # else:
        #    self.servoj_error_count=0

        return True

    def get_observations(self) -> Dict[str, np.ndarray]:
        self._update_robot_state()
        try:
            rot = Rotation.from_euler("xyz", self.tcp_orientation)
            quat = rot.as_quat()
            tcp_pose = np.zeros(7, dtype=np.float32)
            tcp_pose[0:3] = self.tcp_position
            tcp_pose[3] = quat[3]  # qw
            tcp_pose[4:7] = quat[0:3]  # qx, qy, qz
        except Exception:
            tcp_pose = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float32)

        return {
            "joint_positions": self.get_joint_state().copy(),
            "joint_velocities": np.zeros(7, dtype=np.float32),
            "ee_pos_quat": tcp_pose,
            "gripper_position": np.array([self.gripper_position], dtype=np.float32),
        }

    # New Interfaces
    def immediate_stop(self) -> bool:
        """触发机器人急停 (HRIF_GrpStop)"""
        try:
            self._servo_active = False
            self.logger.warning("Executing HRIF_GrpStop: motion stop!")
            nRet = self.sdk.HRIF_GrpStop(self.box_id, self.rbt_id)

            if nRet == 0:
                self.logger.info("Emergency stop executed successfully")
                return True

            self.logger.error(f"HRIF_GrpStop failed with error code: {nRet}")
            return False

        except Exception as e:
            self.logger.error(f"Exception in immediate_stop: {str(e)}")
            return False

    def move_j(self, joint_state: np.ndarray) -> bool:
        try:
            self._servo_active = False
            # joints=joint_state
            joint_state = joint_state.astype(np.float32)
            joints = np.rad2deg(joint_state)
            joints = [float(x) for x in joints]
            print(joints)
            points = [520, 0, 572, 180, 0, 180]  # start pose for the servo motion
            self.sdk.HRIF_SetOverride(
                0, 0, 0.5
            )  # set override, 50% etc.This command is used for the HRIF_MoveJ
            nret = self.sdk.HRIF_MoveJ(
                0, 0, points, joints, "TCP", "Base", 50, 50, 50, 1, 0, 0, 0, "0"
            )
            print(nret)
            time.sleep(0.1)
            result = []
            self.sdk.HRIF_IsMotionDone(0, 0, result)

            while result[0] != True:
                self.sdk.HRIF_IsMotionDone(0, 0, result)
                print(result[0])

            return True
        except Exception as e:
            self.logger.error(f"Exception in Move_j(Synchronising): {str(e)}")
            return False

    def blackout(self) -> bool:
        try:
            self._servo_active = False
            self.logger.warning("Executing black out")
            nRet = self.sdk.HRIF_BlackOut(self.box_id)

            if nRet == 0:
                self.logger.info("power off executed successfully")
                return True

            self.logger.error(f"HRIF_blackout failed with error code: {nRet}")
            return False

        except Exception as e:
            self.logger.error(f"Exception in HRIF_blackout: {str(e)}")
            return False

    def check_state(self) -> bool:
        """
        Initialize robot connection and ensure it's powered and enabled

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        try:
            # 1. Connect to robot
            # c_result = self._connect_to_robot()
            if not self.connected:
                return False

            # 2. Read current robot state
            state = self.read_robot_state()
            if not state or len(state) < 13:
                self.logger.error(f"Failed to read robot state,return value is {state}")
                return False

            # 3. Check power state (result[9])
            if state[9] == "0":
                self.logger.info("Robot is not powered ...")
                return False

            # 4. Check enable state (result[1])
            if state[1] == "0":
                self.logger.info("Robot is not enabled ...")
                return False

            # 5. Verify error state (result[2])
            if state[2] == "1":
                self.logger.error(
                    f"Robot has error state! Error code: {state[3]}, Axis ID: {state[4]}"
                )
                return False

            # 6. Check emergency stop state (result[7])
            if state[7] == "1":
                self.logger.error("Robot is in emergency stop state!")
                return False

            self.logger.info("Robot initialization completed successfully")
            return True

        except Exception as e:
            self.logger.error(f"Robot initialization failed: {str(e)}")
            return False

    def check_power(self) -> bool:
        try:
            # 1. Connect to robot
            # c_result = self._connect_to_robot()

            # 2. Read current robot state
            state = self.read_robot_state()
            if not state or len(state) < 13:
                self.logger.error("Failed to read robot state")
                return False

            # 3. Check power state (result[9])
            if state[9] == "0":
                self.logger.info("Robot is not powered -...")
                return False

            return True
        except Exception as e:
            self.logger.error(f"Robot check power failed: {str(e)}")
            return False

    def re_init(self) -> bool:
        """
        Initialize robot connection and ensure it's powered and enabled

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        try:
            self._servo_active = False
            # 1. Connect to robot
            c_result = self._connect_to_robot()
            if not c_result:
                print("机器人连接失败")
                return False
            # 2. Read current robot state
            state = self.read_robot_state()
            if not state or len(state) < 13:
                self.logger.error("Failed to read robot state")
                return False

            # 3. Check power state (result[9])
            if state[9] == "0":
                self.logger.info("Robot is not powered")
                return False

                # self.logger.info("Robot is not powered - attempting to power on...")
                # if not self._Electrify():
                #     self.logger.error("Failed to power on the robot")
                #     return False
                # self.logger.info("Robot powered on successfully")

            # 4. Check enable state (result[1])
            if state[1] == "0":
                self.logger.info("Robot is not enabled")
                return False

                # self.logger.info("Robot is not enabled - attempting to enable...")
                # if not self._GrpEnable():
                #     self.logger.error("Failed to enable the robot")
                #     return False
                # self.logger.info("Robot enabled successfully")

            # 5. Verify error state (result[2])
            if state[2] == "1":
                self.logger.error(
                    f"Robot has error state! Error code: {state[3]}, Axis ID: {state[4]}"
                )
                return False

            # 6. Check emergency stop state (result[7])
            if state[7] == "1":
                self.logger.error("Robot is in emergency stop state!")
                return False

            self.logger.info("Robot reinitialization completed successfully")
            return True

        except Exception as e:
            self.logger.error(f"Robot initialization failed: {str(e)}")
            return False

    def read_io(self, bit=0) -> list:
        """持续监控电箱状态的线程函数"""
        result = []  # 必须初始化为空列表

        # 调用接口读取状态
        nRet = self.sdk.HRIF_ReadBoxCI(self.box_id, bit, result)
        # nRet = self.sdk.HRIF_ReadBoxDO(self.box_id, bit, result)

        if nRet == 0:  # 调用成功
            pass
        else:
            print(f"读取电箱{self.box_id}位{bit}失败，错误码: {nRet}")
            result = []

        return result

    def group_reset(self) -> bool:
        try:
            nRet = self.sdk.HRIF_GrpReset(self.box_id, self.rbt_id)
            if nRet != 0:
                self.logger.error(f"HRIF_GrpReset failed (Error code: {nRet})")
                return False
            self.logger.info("Group reseted successful")
            return True
        except Exception as e:
            self.logger.error(f"Group reseted error: {str(e)}")
            return False

    def connect_controller(self) -> bool:
        try:
            nRet = self.sdk.HRIF_Connect2Controller(self.box_id)
            if nRet != 0:
                self.logger.error(
                    f"HRIF_Connect2Controller failed (Error code: {nRet})"
                )
                return False
            self.logger.info("Connect Controller successful")
            return True
        except Exception as e:
            self.logger.error(f"Connect Controller error: {str(e)}")
            return False

    def group_disable(self) -> bool:
        try:
            nRet = self.sdk.HRIF_GrpDisable(self.box_id, self.rbt_id)
            if nRet != 0:
                self.logger.error(f"HRIF_GrpDisable failed (Error code: {nRet})")
                return False
            self.logger.info("Group disabled successfully")
            return True
        except Exception as e:
            self.logger.error(f"Group disabled error: {str(e)}")
            return False


# 双臂控制类，符合BimanualRobot协议
class CPSDualArm(BimanualRobot):
    """双臂CPS机器人控制器，同时控制左右两个机械臂"""

    def __init__(self, _robot_l: CPSRobot, _robot_r: CPSRobot):
        """
        初始化双臂机器人控制器

        :param left_ip: 左臂控制器IP地址
        :param left_port: 左臂控制器端口
        :param right_ip: 右臂控制器IP地址
        :param right_port: 右臂控制器端口
        :param servo_time: 伺服周期时间(秒)
        :param lookahead_time: 轨迹前瞻时间(秒)
        """
        self.logger = logging.getLogger("CPSDualArm")
        self.logger.info("Initializing dual-arm CPS robot controller")

        # 分别创建左右臂实例
        self._robot_l = _robot_l

        self._robot_r = _robot_r
        # 初始化完成检查

        self.servoj_error_count = 0

        self.logger.info("Dual-arm CPS robot initialized successfully")

    def _init_servo_mode(self) -> bool:
        """Initialize servo mode"""
        try:
            nRet1 = self._robot_l.sdk.HRIF_StartServo(
                self._robot_l.box_id,
                self._robot_l.rbt_id,
                self._robot_l.servo_time,
                self._robot_l.lookahead_time,
            )

            nRet2 = self._robot_r.sdk.HRIF_StartServo(
                self._robot_r.box_id,
                self._robot_r.rbt_id,
                self._robot_r.servo_time,
                self._robot_r.lookahead_time,
            )
            if nRet1 != 0 or nRet2 != 0:
                # self.read_robot_state()
                return False

            self.logger.info("Servo mode activated")
            return True
        except Exception as e:
            self.logger.error(f"Servo initialization error: {str(e)}")
            return False

    def num_dofs(self) -> int:
        """获取双臂总自由度数量 (左臂7DOF + 右臂7DOF = 14DOF)"""
        return self._robot_l.num_dofs() + self._robot_r.num_dofs()

    def get_joint_state(self) -> np.ndarray:
        """获取双臂关节状态 [左臂1-6, 左爪, 右臂1-6, 右爪]"""
        left_state = self._robot_l.get_joint_state()
        right_state = self._robot_r.get_joint_state()
        return np.concatenate([left_state, right_state])

    def command_joint_state(self, joint_state: np.ndarray) -> bool:
        """
        同时控制左右臂关节状态

        :param joint_state: 14维数组[左臂1-6, 左爪, 右臂1-6, 右爪]
        """
        # if joint_state.size != 14:
        #    self.logger.error(f"Invalid joint state dimension: {joint_state.size}, expected 14")
        #    return False

        # 分割左右臂命令
        # dof_per_arm = self._robot_l.num_dofs()
        left_joints = joint_state[:6]
        right_joints = joint_state[7:13]

        # 同步命令左右臂
        result = self._robot_l.command_joint_state(
            left_joints
        ) and self._robot_r.command_joint_state(right_joints)

        # if not result:
        #    self.servoj_error_count+=1
        #    if self.servoj_error_count>4:
        #        self._robot_l._servo_active=False
        #        self._robot_r._servo_active=False
        #        self.servoj_error_count=0
        # else:
        #    self.servoj_error_count=0

        return result

    def get_observations(self) -> Dict[str, np.ndarray]:
        """获取双臂综合观测数据"""
        l_obs = self._robot_l.get_observations()
        r_obs = self._robot_r.get_observations()

        # 验证观测键一致性
        if set(l_obs.keys()) != set(r_obs.keys()):
            self.logger.error("Inconsistent observation keys between arms!")
            return {}

        # 合并双臂观测数据
        merged_obs = {}
        for key in l_obs.keys():
            try:
                # 合并数组维度
                merged_obs[key] = np.concatenate((l_obs[key], r_obs[key]))
            except ValueError as e:
                self.logger.error(f"Failed to merge '{key}': {str(e)}")
                merged_obs[key] = np.array([])

        return merged_obs

    def _connect_to_robot(self) -> bool:
        try:
            left_result = self._robot_l._connect_to_robot()
            right_result = self._robot_r._connect_to_robot()
            return left_result and right_result
        except Exception as e:
            print(f"双臂连接失败,错误信息为{e}")
            return False

    def immediate_stop(self) -> bool:
        """紧急停止双臂"""
        try:
            left_result = self._robot_l.immediate_stop()
            right_result = self._robot_r.immediate_stop()
            return left_result and right_result
        except Exception as e:
            print(f"双臂紧急停止失败,错误信息为{e}")
            return False

    def blackout(self) -> bool:
        try:
            """断电双臂"""
            left_result = self._robot_l.blackout()
            right_result = self._robot_r.blackout()
            return left_result and right_result
        except Exception as e:
            print(f"双臂断电失败,错误信息为{e}")
            return False

    def check_state(self) -> bool:
        """检查双臂状态，如果失败则报告具体是哪一侧手臂的问题"""
        try:
            left_result = self._robot_l.check_state()
            right_result = self._robot_r.check_state()

            if not left_result and not right_result:
                self.logger.error("双臂状态检查均失败")
                return False
            elif not left_result:
                self.logger.error("左臂状态检查失败")
                return False
            elif not right_result:
                self.logger.error("右臂状态检查失败")
                return False

            self.logger.info("双臂状态检查正常")
            return True

        except Exception as e:
            self.logger.error(f"双臂状态检查异常: {str(e)}")
            return False

    def re_init(self) -> bool:
        try:
            left_result = self._robot_l.re_init()
            right_result = self._robot_r.re_init()
            return left_result and right_result
        except Exception as e:
            print(f"双臂重新初始化失败,错误信息为{e}")
            return False

    def read_io(self, bit=0) -> list:
        """持续监控电箱状态的线程函数(双臂时默认为右臂)"""
        return self._robot_l.read_io(bit)

    def group_reset(self) -> bool:
        """双臂协同复位 (HRIF_GrpReset)"""
        try:
            # 左臂复位
            left_result = self._robot_l.group_reset()
            # 右臂复位
            right_result = self._robot_r.group_reset()

            if not left_result or not right_result:
                self.logger.error(
                    "双臂复位失败 (左臂: %s, 右臂: %s)",
                    "成功" if left_result else "失败",
                    "成功" if right_result else "失败",
                )
                return False

            self.logger.info("双臂协同复位完成")
            return True

        except Exception as e:
            self.logger.error("双臂复位过程中发生异常: %s", str(e))
            return False

    def connect_controller(self) -> bool:
        """双臂控制器连接 (HRIF_Connect2Controller)"""
        try:
            # 左臂控制器连接
            left_result = self._robot_l.connect_controller()
            # 右臂控制器连接
            right_result = self._robot_r.connect_controller()

            if not left_result or not right_result:
                self.logger.error(
                    "双臂控制器连接失败 (左臂: %s, 右臂: %s)",
                    "成功" if left_result else "失败",
                    "成功" if right_result else "失败",
                )
                return False

            self.logger.info("双臂控制器同步连接成功")
            return True

        except Exception as e:
            self.logger.error("双臂控制器连接异常: %s", str(e))
            return False

    def group_disable(self) -> bool:
        """双臂组态禁用 (HRIF_GrpDisable)"""
        try:
            # 禁用左臂组态
            left_result = self._robot_l.group_disable()
            # 禁用右臂组态
            right_result = self._robot_r.group_disable()

            if not left_result or not right_result:
                self.logger.error(
                    "双臂禁用失败 (左臂: %s, 右臂: %s)",
                    "成功" if left_result else "失败",
                    "成功" if right_result else "失败",
                )
                return False

            self.logger.warning("双臂组态已进入禁用状态")
            return True

        except Exception as e:
            self.logger.error("双臂禁用过程中发生异常: %s", str(e))
            return False

    def _GrpEnable(self) -> bool:
        """Enable both arm groups"""
        try:
            left_result = self._robot_l._GrpEnable()
            right_result = self._robot_r._GrpEnable()

            if not left_result or not right_result:
                self.logger.error(
                    f"Enable failed (Left: {'OK' if left_result else 'FAIL'}, "
                    f"Right: {'OK' if right_result else 'FAIL'})"
                )
                return False

            self.logger.info("Dual-arm enabled")
            return True
        except Exception as e:
            self.logger.error(f"Enable exception: {str(e)}")
            return False

    def _Electrify(self) -> bool:
        """Power on both arms"""
        try:
            left_result = self._robot_l._Electrify()
            right_result = self._robot_r._Electrify()

            if not left_result or not right_result:
                self.logger.error(
                    f"Power on failed (Left: {'OK' if left_result else 'FAIL'}, "
                    f"Right: {'OK' if right_result else 'FAIL'})"
                )
                return False

            self.logger.info("Dual-arm powered on")
            return True
        except Exception as e:
            self.logger.error(f"Power on exception: {str(e)}")
            return False

    def check_power(self) -> bool:
        try:
            left_result = self._robot_l.check_power()
            right_result = self._robot_r.check_power()

            if not left_result or not right_result:
                self.logger.error(
                    f"check_power failed (Left: {'OK' if left_result else 'FAIL'}, "
                    f"Right: {'OK' if right_result else 'FAIL'})"
                )
                return False

            self.logger.info("Dual-arm already power on")
            return True
        except Exception as e:
            self.logger.error(f"check_power exception: {str(e)}")
            return False


class RobotServer:

    def __init__(self, ip_left=LEFT_IP, ip_right=RIGHT_IP):
        self.arm_type = "not_connected"
        self.ip_dict = {"left": ip_left, "right": ip_right}
        self.robot_l = self._connect_robot(ip_left, "左臂")
        self.robot_r = self._connect_robot(ip_right, "右臂")
        self.robot = self._init_robot()

    def _connect_robot(self, ip, name):
        """尝试连接机器人并返回实例，失败返回None"""
        try:
            robot = CPSRobot(ip=ip)
            if not robot.connected:
                print(f"警告: {name}机器人连接失败 (IP: {ip})")
                return None
            print(f"{name}机器人连接成功 (IP: {ip})")
            return robot
        except Exception as e:
            print(f"连接{name}机器人时发生错误: {str(e)}")
            return None

    def _init_robot(self):
        """初始化机器人（单臂/双臂模式）"""
        if self.robot_l and self.robot_r:
            self.arm_type = "both"
            print("双臂机器人模式已激活")
            return CPSDualArm(self.robot_l, self.robot_r)
        elif self.robot_l:
            self.arm_type = "left"
            print("单臂模式: 使用左臂机器人")
            return self.robot_l
        elif self.robot_r:
            self.arm_type = "right"
            print("单臂模式: 使用右臂机器人")
            return self.robot_r
        else:
            self.arm_type = "not_connected"
            raise RuntimeError("所有机器人连接均失败，请检查网络和IP配置")


ROBOT = RobotServer()
ROBOT_TEST = RobotServer()
