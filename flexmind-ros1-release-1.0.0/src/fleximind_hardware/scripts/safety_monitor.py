#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading
import time
from collections import deque

import numpy as np
import psutil
import rospy
from sensor_msgs.msg import JointState

from fleximind_bringup.interface import RobotInterface
from fleximind_robot.CPS import CPSClient


class JointSafetyMonitor:
    def __init__(self):
        rospy.init_node("joint_safety_monitor", anonymous=True)

        # 初始化CPS客户端
        self.sdk = CPSClient()

        # 从参数服务器读取配置
        self.joint_count = self._get_param_safe(
            "~joint_count", default=6, param_type=int
        )

        # 需要监控的关节名称列表（根据实际话题数据）
        # self.monitored_joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.monitored_joints = self._get_param_safe(
            "~monitored_joints",
            default=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            param_type=list,
        )
        # 注意：joint_7 是夹爪，不进行安全监控

        # 系统参数配置（针对30Hz优化）
        self._configure_for_30hz()

        # 连接机械臂
        if not self._connect_arms():
            rospy.logerr("机械臂连接失败，节点将进入待机状态")
            return

        # 初始化数据结构
        self._init_data_structures()

        # 启动ROS订阅和监控线程
        self._start_monitoring()

    def _configure_for_30hz(self):
        """针对30Hz数据流的参数配置"""
        # 从参数服务器读取系统参数
        self.cpu_threshold = self._get_param_safe(
            "~cpu_threshold", default=85.0, param_type=float
        )
        self.cpu_overload_duration = self._get_param_safe(
            "~cpu_overload_duration", default=3.0, param_type=float
        )
        self.check_interval = self._get_param_safe(
            "~check_interval", default=0.05, param_type=float
        )

        # 导数计算参数（基于30Hz周期33.3ms）
        self.min_time_interval = self._get_param_safe(
            "~min_time_interval", default=0.03, param_type=float
        )  # 最小有效时间间隔（30Hz周期）
        self.min_position_change = self._get_param_safe(
            "~min_position_change", default=0.5, param_type=float
        )  # 更敏感的位置变化阈值（度）

        # 历史数据窗口大小（5个样本约166ms时间窗口）
        self.history_length = self._get_param_safe(
            "~history_length", default=5, param_type=int
        )

        # 监控频率（匹配30Hz输入）
        self.monitor_rate = self._get_param_safe(
            "~monitor_rate", default=30, param_type=int
        )  # Hz

        # 左右臂独立配置
        self.left_arm_config = self._load_arm_config("left_arm")
        self.right_arm_config = self._load_arm_config("right_arm")

    def _init_data_structures(self):
        """初始化数据存储结构"""
        self.left_arm_data = {}
        self.right_arm_data = {}

        # 初始化数据结构模板
        self.joint_data_template = {
            "positions": deque(maxlen=self.history_length),
            "times": deque(maxlen=self.history_length),
            "last_position": None,
            "last_time": None,
        }

        # 线程安全
        self.data_lock = threading.Lock()
        self.stop_monitor = False
        self.cpu_overload_start = None

    def _start_monitoring(self):
        """启动监控系统"""
        # 订阅ROS话题
        self.left_arm_sub = rospy.Subscriber(
            RobotInterface.DATA_ACTION_LEFT_ARM.value,
            JointState,
            self.left_arm_callback,
            queue_size=30,  # 匹配30Hz频率
        )

        self.right_arm_sub = rospy.Subscriber(
            RobotInterface.DATA_ACTION_RIGHT_ARM.value,
            JointState,
            self.right_arm_callback,
            queue_size=30,
        )

        # 启动监控线程（30Hz匹配）
        self.monitor_thread = threading.Thread(
            target=self.safety_monitor, name="SafetyMonitorThread"
        )
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

        # 启动CPU监控定时器
        self.timer = rospy.Timer(
            rospy.Duration(self.check_interval), self.cpu_check_callback
        )

        # rospy.loginfo("安全监控器已启动，运行频率: %dHz", self.monitor_rate)
        # rospy.loginfo("监控6个关节: %s", self.monitored_joints)

    def _load_arm_config(self, arm_prefix):
        """加载单臂配置"""
        config = {
            "boxid": self._get_param_safe(
                f"~{arm_prefix}_boxid",
                default=0 if arm_prefix == "left_arm" else 1,
                param_type=int,
            ),
            "ip": self._get_param_safe(
                f"~{arm_prefix}_ip",
                default="192.168.1.20" if arm_prefix == "left_arm" else "192.168.1.30",
                param_type=str,
            ),
            "port": self._get_param_safe(
                f"~{arm_prefix}_port", default=10003, param_type=int
            ),
            "robot_id": self._get_param_safe(
                f"~{arm_prefix}_robot_id", default=0, param_type=int
            ),
            "max_velocity": self._get_param_safe(
                f"~{arm_prefix}_max_joint_velocity",
                default=[500] * self.joint_count,  # 度数单位
                param_type=list,
            ),
            "max_acceleration": self._get_param_safe(
                f"~{arm_prefix}_max_joint_acceleration",
                default=[2000] * self.joint_count,  # 度数单位
                param_type=list,
            ),
            "max_jerk": self._get_param_safe(
                f"~{arm_prefix}_max_joint_jerk",
                default=[10000] * self.joint_count,  # 度数单位
                param_type=list,
            ),
        }

        # 验证参数长度
        for param_name in ["max_velocity", "max_acceleration", "max_jerk"]:
            if len(config[param_name]) != self.joint_count:
                rospy.logwarn("%s %s 长度不匹配，使用默认值", arm_prefix, param_name)
                config[param_name] = (
                    [500] * self.joint_count
                    if param_name == "max_velocity"
                    else (
                        [2000] * self.joint_count
                        if param_name == "max_acceleration"
                        else [10000] * self.joint_count
                    )
                )

        return config

    def _get_param_safe(self, param_name, default=None, param_type=None):
        """安全读取ROS参数"""
        try:
            value = rospy.get_param(param_name)
            if param_type == int:
                value = int(value)
            elif param_type == float:
                value = float(value)
            elif param_type == list:
                value = (
                    list(value)
                    if isinstance(value, (list, tuple, np.ndarray))
                    else [value]
                )
            elif param_type == str:
                value = str(value)
            return value
        except (KeyError, ValueError, TypeError) as e:
            rospy.logwarn("参数'%s'读取失败，使用默认值: %s", param_name, default)
            return default

    def _connect_arms(self):
        """连接左右机械臂"""
        arms = [
            ("left_arm", self.left_arm_config),
            ("right_arm", self.right_arm_config),
        ]

        connected = True
        for arm_name, config in arms:
            try:
                if self.sdk.HRIF_IsConnected(config["boxid"]):
                    rospy.loginfo("%s已连接（boxid: %d）", arm_name, config["boxid"])
                    continue

                rospy.loginfo(
                    "连接%s（IP: %s:%d, boxid: %d）",
                    arm_name,
                    config["ip"],
                    config["port"],
                    config["boxid"],
                )
                ret = self.sdk.HRIF_Connect(
                    config["boxid"], config["ip"], config["port"]
                )

                if ret != 0:
                    rospy.logerr("%s连接失败（错误码: %d）", arm_name, ret)
                    connected = False
                elif self.sdk.HRIF_IsConnected(config["boxid"]):
                    rospy.loginfo("%s连接成功", arm_name)
                else:
                    rospy.logerr("%s连接状态验证失败", arm_name)
                    connected = False

            except Exception as e:
                rospy.logerr("%s连接异常: %s", arm_name, str(e))
                connected = False

        return connected

    def left_arm_callback(self, msg):
        """左臂数据回调（30Hz）"""
        # 添加回调计数调试
        if not hasattr(self, "left_callback_count"):
            self.left_callback_count = 0
        self.left_callback_count += 1

        # if self.left_callback_count % 50 == 0:
        #     # 显示所有6个关节的位置
        #     positions_str = [f"{p:.1f}" for p in msg.position[:6]]
        #     rospy.loginfo("左臂回调第%d次，6个关节位置: %s",
        #                  self.left_callback_count,
        #                  positions_str)

        self.process_joint_data(msg, "left_arm")

    def right_arm_callback(self, msg):
        """右臂数据回调（30Hz）"""
        # 添加回调计数调试
        if not hasattr(self, "right_callback_count"):
            self.right_callback_count = 0
        self.right_callback_count += 1

        # if self.right_callback_count % 50 == 0:
        #     # 显示所有6个关节的位置
        #     positions_str = [f"{p:.1f}" for p in msg.position[:6]]
        #     rospy.loginfo("右臂回调第%d次，6个关节位置: %s",
        #                  self.right_callback_count,
        #                  positions_str)

        self.process_joint_data(msg, "right_arm")

    def process_joint_data(self, msg, arm_name):
        """处理关节数据（优化30Hz处理）"""
        with self.data_lock:
            current_time = msg.header.stamp.to_sec()
            data_dict = (
                self.left_arm_data if arm_name == "left_arm" else self.right_arm_data
            )

            # 初始化数据结构（仅需一次）
            if not data_dict:
                for joint_name in self.monitored_joints:
                    data_dict[joint_name] = {
                        "positions": deque(maxlen=self.history_length),
                        "times": deque(maxlen=self.history_length),
                        "last_position": None,
                        "last_time": None,
                    }
                # rospy.loginfo("%s初始化了%d个关节监控: %s", arm_name, len(data_dict), list(data_dict.keys()))

            # 处理每个关节数据
            processed_count = 0
            for i, joint_name in enumerate(msg.name):
                # 只处理前6个关节（joint_1 到 joint_6），跳过joint_7（夹爪）
                if joint_name in self.monitored_joints and joint_name in data_dict:
                    joint_data = data_dict[joint_name]
                    current_position = msg.position[i] if i < len(msg.position) else 0.0

                    # 保存数据（自动维护固定长度队列）
                    joint_data["positions"].append(current_position)
                    joint_data["times"].append(current_time)

                    # 更新最新值
                    joint_data["last_position"] = current_position
                    joint_data["last_time"] = current_time
                    processed_count += 1

    def calculate_derivatives(self, data_dict, arm_name):
        """优化的导数计算（适配30Hz）- 所有单位都是度数"""
        results = {}
        all_velocities = []  # 存储所有关节的速度信息

        for joint_name, joint_data in data_dict.items():
            # 只处理监控列表中的关节
            if joint_name not in self.monitored_joints:
                continue

            # 需要至少2个数据点
            if len(joint_data["times"]) < 2:
                continue

            times = list(joint_data["times"])
            positions = list(joint_data["positions"])

            # 初始化导数变量
            velocity = 0.0  # 度/秒
            acceleration = 0.0  # 度/秒²
            jerk = 0.0  # 度/秒³

            # 计算速度（当前点与前一个点）
            dt = times[-1] - times[-2]
            if dt >= self.min_time_interval:  # 有效时间间隔检查
                pos_change = positions[-1] - positions[-2]  # 位置变化（度）
                if abs(pos_change) >= self.min_position_change:
                    velocity = pos_change / dt  # 速度（度/秒）

                    # 计算加速度（需要至少3个点）
                    if len(times) >= 3:
                        prev_dt = times[-2] - times[-3]
                        if prev_dt >= self.min_time_interval:
                            prev_pos_change = positions[-2] - positions[-3]
                            if abs(prev_pos_change) >= self.min_position_change:
                                prev_velocity = prev_pos_change / prev_dt
                                acceleration = (velocity - prev_velocity) / dt

                                # 计算加加速度（需要至少4个点）
                                if len(times) >= 4:
                                    prev_prev_dt = times[-3] - times[-4]
                                    if prev_prev_dt >= self.min_time_interval:
                                        prev_prev_pos_change = (
                                            positions[-3] - positions[-4]
                                        )
                                        if (
                                            abs(prev_prev_pos_change)
                                            >= self.min_position_change
                                        ):
                                            prev_prev_velocity = (
                                                prev_prev_pos_change / prev_prev_dt
                                            )
                                            prev_acceleration = (
                                                prev_velocity - prev_prev_velocity
                                            ) / prev_dt
                                            jerk = (
                                                acceleration - prev_acceleration
                                            ) / dt

            # 记录所有关节的速度信息（已经是度/秒，不需要转换）
            all_velocities.append((joint_name, velocity, acceleration))

            results[joint_name] = {
                "velocity": velocity,  # 度/秒
                "acceleration": acceleration,  # 度/秒²
                "jerk": jerk,  # 度/秒³
            }

        return results

    def safety_monitor(self):
        """安全监控主循环（30Hz）"""
        rate = rospy.Rate(self.monitor_rate)
        monitor_cycle_count = 0

        while not rospy.is_shutdown() and not self.stop_monitor:
            start_time = time.time()
            monitor_cycle_count += 1

            with self.data_lock:
                # 计算并检查左臂状态
                left_arm_derivatives = self.calculate_derivatives(
                    self.left_arm_data, "left_arm"
                )
                self.check_arm_safety(
                    left_arm_derivatives, "left_arm", self.left_arm_config
                )

                # 计算并检查右臂状态
                right_arm_derivatives = self.calculate_derivatives(
                    self.right_arm_data, "right_arm"
                )
                self.check_arm_safety(
                    right_arm_derivatives, "right_arm", self.right_arm_config
                )

                # 每100个监控周期显示一次统计信息
                # if monitor_cycle_count % 100 == 0:
                #     left_active = len([v for v in left_arm_derivatives.values() if abs(v['velocity']) > 0.01])
                #     right_active = len([v for v in right_arm_derivatives.values() if abs(v['velocity']) > 0.01])
                #     rospy.loginfo("监控周期%d - 左臂活跃关节: %d/6, 右臂活跃关节: %d/6",
                #                  monitor_cycle_count, left_active, right_active)

            # 确保精确的30Hz循环
            cycle_time = time.time() - start_time
            if cycle_time < self.check_interval:
                rate.sleep()
            else:
                rospy.logwarn(
                    "监控循环超时: %.3fs > %.3fs", cycle_time, self.check_interval
                )

    def check_arm_safety(self, derivatives, arm_name, config):
        """检查关节安全状态 - 所有单位都是度数"""
        active_joints = []  # 记录有速度数据的关节

        for joint_name, values in derivatives.items():
            # 只检查监控列表中的关节
            if joint_name not in self.monitored_joints:
                continue

            try:
                # 从关节名称提取索引：joint_1 -> 0, joint_2 -> 1, ...
                joint_index = int(joint_name.split("_")[-1]) - 1
            except (ValueError, IndexError):
                joint_index = 0

            if joint_index < len(config["max_velocity"]):
                # 直接使用计算得到的速度值（已经是度/秒）
                velocity_deg = abs(values["velocity"])  # 度/秒
                acceleration_deg = abs(values["acceleration"])  # 度/秒²
                jerk_deg = abs(values["jerk"])  # 度/秒³

                # 记录有速度数据的关节
                if velocity_deg > 0.1:  # 忽略很小的速度值
                    active_joints.append((joint_name, velocity_deg))

                # 检查速度（单位一致：度/秒）
                if velocity_deg > config["max_velocity"][joint_index]:
                    # 修复：先格式化字符串，再传递
                    reason = "%s %s 速度超限: %.1f deg/s > %d deg/s" % (
                        arm_name,
                        joint_name,
                        velocity_deg,
                        config["max_velocity"][joint_index],
                    )
                    self.trigger_emergency(config["boxid"], config["robot_id"], reason)

                # 检查加速度（单位一致：度/秒²）
                if acceleration_deg > config["max_acceleration"][joint_index]:
                    # 修复：先格式化字符串，再传递
                    reason = "%s %s 加速度超限: %.1f deg/s² > %d deg/s²" % (
                        arm_name,
                        joint_name,
                        acceleration_deg,
                        config["max_acceleration"][joint_index],
                    )
                    self.trigger_emergency(config["boxid"], config["robot_id"], reason)

                # 检查加加速度（单位一致：度/秒³）
                if jerk_deg > config["max_jerk"][joint_index]:
                    # 修复：先格式化字符串，再传递
                    reason = "%s %s 加加速度超限: %.1f deg/s³ > %d deg/s³" % (
                        arm_name,
                        joint_name,
                        jerk_deg,
                        config["max_jerk"][joint_index],
                    )
                    self.trigger_emergency(config["boxid"], config["robot_id"], reason)

            # 每50次检查打印一次所有关节的速度状态
            if not hasattr(self, f"{arm_name}_safety_check_count"):
                setattr(self, f"{arm_name}_safety_check_count", 0)

            check_count = getattr(self, f"{arm_name}_safety_check_count")
            check_count += 1
            setattr(self, f"{arm_name}_safety_check_count", check_count)

            # if check_count % 50 == 0 and active_joints:
            #     rospy.loginfo("%s安全检查 - 有速度的关节: %s",
            #                 arm_name,
            #                 [f"{j}:{v:.1f}°/s" for j, v in active_joints])

    def cpu_check_callback(self, event):
        """CPU使用率检查"""
        try:
            now = rospy.Time.now()
            cpu_usage = psutil.cpu_percent(interval=None)

            if cpu_usage > self.cpu_threshold:
                if self.cpu_overload_start is None:
                    self.cpu_overload_start = now
                    rospy.logwarn("CPU超载开始: %.1f%%", cpu_usage)
                else:
                    duration = (now - self.cpu_overload_start).to_sec()
                    if duration >= self.cpu_overload_duration:
                        # 修复：先格式化字符串，再传递
                        reason = "CPU持续超载%.1f秒！当前: %.1f%%" % (
                            duration,
                            cpu_usage,
                        )
                        self.trigger_emergency_all(reason)
            else:
                if self.cpu_overload_start is not None:
                    rospy.loginfo("CPU恢复正常: %.1f%%", cpu_usage)
                self.cpu_overload_start = None

        except Exception as e:
            rospy.logerr("CPU监控异常: %s", str(e))

    def trigger_emergency(self, boxid, robot_id, reason):
        """触发单个机械臂急停"""
        rospy.logfatal("紧急停止！原因: %s", reason)

        try:
            ret = self.sdk.HRIF_GrpStop(boxid, robot_id)
            if ret != 0:
                rospy.logerr("急停指令发送失败（错误码: %d）", ret)
            else:
                rospy.loginfo("boxid %d 急停指令已执行", boxid)
        except Exception as e:
            rospy.logerr("急停异常: %s", str(e))

    def trigger_emergency_all(self, reason):
        """全局紧急停止"""
        rospy.logfatal("全局紧急停止！原因: %s", reason)

        arms = [
            ("left_arm", self.left_arm_config),
            ("right_arm", self.right_arm_config),
        ]

        for arm_name, config in arms:
            try:
                if self.sdk.HRIF_IsConnected(config["boxid"]):
                    ret = self.sdk.HRIF_GrpStop(config["boxid"], config["robot_id"])
                    if ret == 0:
                        rospy.loginfo("%s 已执行急停", arm_name)
                    else:
                        rospy.logerr("%s 急停失败，错误码: %d", arm_name, ret)
            except Exception as e:
                rospy.logerr("停止 %s 异常: %s", arm_name, str(e))

    def shutdown(self):
        """清理资源"""
        self.stop_monitor = True
        if hasattr(self, "timer"):
            self.timer.shutdown()

        if hasattr(self, "monitor_thread") and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=1.0)

        rospy.loginfo("节点关闭，保持机械臂连接状态")


if __name__ == "__main__":
    try:
        monitor = JointSafetyMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("节点启动失败: %s", str(e))
    finally:
        if "monitor" in locals():
            monitor.shutdown()
