#!/usr/bin/env python
import threading
import time
from typing import Dict

import rospy
from std_msgs.msg import Header

from fleximind_bringup.interface import HardwareInterface
from fleximind_hardware import HardwareState, HardwareType
from fleximind_hardware.msg import ConnectionStatus, HardwareStatus

OFFLINE_TIMEOUT = 3
WARNING_TIMEOUT = 1.5


class HardwareMonitor:
    def __init__(self):
        rospy.init_node("hardware_monitor")

        # 使用枚举类初始化设备类型
        self.device_types = [t.value for t in HardwareType]
        self.monitor_topics: Dict[str, dict] = {}
        for dev_type in self.device_types:
            self.monitor_topics[dev_type] = rospy.get_param(
                f"/{dev_type}_monitor_topics", {}
            )

        # 状态存储字典（使用枚举值作为键）
        self.status_dict = {t.value: {} for t in HardwareType}
        self.data_lock = threading.Lock()

        # 使用枚举值设置发布主题
        self.status_pub = rospy.Publisher(
            HardwareInterface.STATUS.value, HardwareStatus, queue_size=10
        )

        # 加载容错配置（使用枚举值作为键）
        self.fault_tolerance = {
            HardwareType.CAMERA.value: rospy.get_param(
                f"~fault_tolerance/{HardwareType.CAMERA.value}", 0
            ),
            HardwareType.ROBOT.value: rospy.get_param(
                f"~fault_tolerance/{HardwareType.ROBOT.value}", 0
            ),
            HardwareType.GRIPPER.value: rospy.get_param(
                f"~fault_tolerance/{HardwareType.GRIPPER.value}", 0
            ),
            HardwareType.REMOTE.value: rospy.get_param(
                f"~fault_tolerance/{HardwareType.REMOTE.value}", 0
            ),
        }

        # 定时器
        self.check_timer = rospy.Timer(
            rospy.Duration(0.5), self.publish_hardware_status
        )

        # 初始化设备订阅
        self._init_devices()

    def _init_devices(self):
        """从参数服务器加载设备配置（使用枚举值）"""
        for dev_type in self.device_types:
            devices = self.monitor_topics[dev_type]
            for name, topic in devices.items():
                rospy.Subscriber(
                    topic,
                    rospy.AnyMsg,
                    lambda msg, t=topic, n=name, d=dev_type: self.device_callback(
                        msg, t, n, d
                    ),
                )
                with self.data_lock:
                    self.status_dict[dev_type][name] = {
                        "status": HardwareState.OFFLINE.value,  # 初始状态
                        "last_msg_time": 0,
                    }

    def device_callback(self, msg, topic, name, dev_type):
        """更新设备最后活跃时间"""
        # rospy.loginfo_throttle(10.0, f"收到{dev_type}.{name}的消息，话题: {topic}")
        
        with self.data_lock:
            if dev_type in self.status_dict and name in self.status_dict[dev_type]:
                self.status_dict[dev_type][name][
                    "last_msg_time"
                ] = rospy.Time.now().to_sec()
                # 使用枚举值设置状态
                self.status_dict[dev_type][name]["status"] = HardwareState.ONLINE.value
                # rospy.loginfo_throttle(10.0, f"更新{dev_type}.{name}状态为在线")

    def _calculate_status(self, data):
        """计算设备当前状态（返回枚举成员）"""
        current_time = rospy.Time.now().to_sec()
        time_diff = current_time - data["last_msg_time"]

        if time_diff > OFFLINE_TIMEOUT:
            return HardwareState.OFFLINE
        elif time_diff > WARNING_TIMEOUT:
            return HardwareState.WARNING
        return HardwareState.ONLINE

    def publish_hardware_status(self, event):
        """发布聚合状态（考虑容错阈值）"""
        try:
            hw_status = HardwareStatus()
            hw_status.header = Header(stamp=rospy.Time.now())

            # 检查每类设备状态
            overall_ok = True

            for dev_type in self.device_types:
                dev_status_list = []
                error_count = 0

                if (
                    self.status_dict[dev_type] == {}
                    and self.monitor_topics[dev_type] != {}
                ):
                    overall_ok = False
                    error_count = len(self.monitor_topics[dev_type])

                    for name, topic in self.monitor_topics[dev_type].items():
                        # 创建连接状态消息
                        conn_status = ConnectionStatus()
                        conn_status.name = name
                        conn_status.status = HardwareState.OFFLINE.value
                        dev_status_list.append(conn_status)

                else:
                    for name, data in self.status_dict[dev_type].items():
                        # 计算当前状态（返回枚举成员）
                        current_state = self._calculate_status(data)

                        # 创建连接状态消息
                        conn_status = ConnectionStatus()
                        conn_status.name = name
                        conn_status.status = current_state.value
                        dev_status_list.append(conn_status)

                        # 统计故障设备数（非在线状态）
                        if current_state != HardwareState.ONLINE:
                            error_count += 1

                # 应用容错阈值判断
                if error_count > self.fault_tolerance[dev_type]:
                    overall_ok = False

                # 填充设备状态列表
                if dev_type == HardwareType.CAMERA.value:
                    hw_status.camera_error_num = error_count
                    hw_status.camera_status = dev_status_list
                elif dev_type == HardwareType.ROBOT.value:
                    hw_status.robot_error_num = error_count
                    hw_status.robot_status = dev_status_list
                elif dev_type == HardwareType.GRIPPER.value:
                    hw_status.gripper_error_num = error_count
                    hw_status.gripper_status = dev_status_list

            hw_status.overall_status = overall_ok
            self.status_pub.publish(hw_status)

        except Exception as e:
            rospy.logerr(f"状态发布失败: {str(e)}")


if __name__ == "__main__":
    monitor = HardwareMonitor()
    rospy.spin()
