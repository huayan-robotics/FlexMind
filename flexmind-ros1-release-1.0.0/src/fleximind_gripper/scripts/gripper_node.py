#!/usr/bin/env python

# 系统依赖
import asyncio
import os
import sys

path = os.path.abspath(".")
sys.path.insert(0, path + "/src/fleximind_gripper/scripts")
import threading
from dataclasses import dataclass, field
from enum import IntEnum

import rospy
from rospy import Time
from std_msgs.msg import Header

from fleximind_bringup import DEFAULT_AXIS_NUM, DEFAULT_LOG_PERIOD_S
from fleximind_bringup.interface import GripperInterface, RemoteControlInterface
from fleximind_gripper.huayan_plugin import HuayanPluginGripper

# 功能包依赖
from fleximind_gripper.msg import GripperData, GripperStatus
from fleximind_hardware.msg import DualJointStates
from fleximind_robot.srv import PublishingSwitch, PublishingSwitchResponse


@dataclass
class GripperState:
    """夹爪状态结构体"""

    last_publish_time: Time = field(default_factory=Time.now)
    position: int = 0
    is_busy: bool = False
    error_count: int = 0
    max_error_count: int = 5


class GripperType(IntEnum):
    enGripperType_Left = 0  # 左夹爪
    enGripperType_Right = 1  # 右夹爪
    enGripperType_last_axis = 6  # 末端轴


class GripperControlNode:
    def __init__(self):
        rospy.init_node("fleximind_gripper_node")

        self.is_publishing = False
        self.is_connecting = False
        self.last_publish_time_l = rospy.Time.now()
        self.last_publish_time_r = rospy.Time.now()
        self.control_timer_period = rospy.get_param("~control_timer_period")
        self.reader_timer_period = rospy.get_param("~reader_timer_period")

        # 重连相关参数
        self.reconnect_attempts = rospy.get_param("~reconnect_attempts", 0)
        self.reconnect_delay = rospy.get_param("~reconnect_delay", 2.0)  # 重连延迟秒数

        # 创建 asyncio 事件循环（在独立线程中运行）
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self._run_async_loop, daemon=True).start()

        # 初始化两个夹爪控制器
        self._initialize_gripper()

        # 创建独立的订阅者：分别订阅左、右夹爪的控制信号, 否则存在延时
        self.sub_left_gripper = rospy.Subscriber(
            RemoteControlInterface.JOINT_STATES.value,
            DualJointStates,
            self.left_gripper_callback,
            queue_size=1,
        )

        self.sub_right_gripper = rospy.Subscriber(
            RemoteControlInterface.JOINT_STATES.value,
            DualJointStates,
            self.right_gripper_callback,
            queue_size=1,
        )

        # 创建发布者
        self.gripper_data_pub = rospy.Publisher(
            GripperInterface.INFO.value, GripperData, queue_size=10
        )

        self.gripper_status_pub = rospy.Publisher(
            GripperInterface.STATUS.value, GripperStatus, queue_size=10
        )
        # 定时器，需要和夹爪控制间隔一段时间，否则存在数据截断
        self.timer = rospy.Timer(
            rospy.Duration(self.reader_timer_period), self.publish_gripper_info
        )

        self._create_services()

        # 分离左右夹爪的状态变量 - 改为结构体
        self.left_gripper_data = GripperState()
        self.right_gripper_data = GripperState()

        # 使用独立的锁
        self.left_lock = threading.Lock()
        self.right_lock = threading.Lock()

        rospy.loginfo("夹爪控制节点初始化完成")

    # === 创建服务 === #

    def _create_services(self):
        """创建服务"""
        services = [
            (
                PublishingSwitch,
                GripperInterface.GRIPPER_DATA_PUB.value,
                self.data_publishing_switch_callback,
            )
        ]

        for srv_type, srv_name, callback in services:
            rospy.Service(srv_name, srv_type, callback)

    def data_publishing_switch_callback(self, req):
        """数据发布开关服务回调"""
        response = PublishingSwitchResponse()
        try:
            self.is_publishing = req.is_open
            response.code = 0
            response.message = f"数据发布开关设置为: {self.is_publishing}"
            rospy.loginfo(response.message)
        except Exception as e:
            response.code = 500
            response.message = str(e)
        return response

    # === 初始化夹爪 === #

    def _set_gripper_param(self):
        cmd = "SetGripper"
        param = [
            self.gripper_configs[GripperType.enGripperType_Left]["gripper_type"],
            self.gripper_configs[GripperType.enGripperType_Left]["gripper_name"],
        ]
        result = []
        left_success = self.left_gripper._call_command(cmd, param, result)

        result = []
        param = [
            self.gripper_configs[GripperType.enGripperType_Right]["gripper_type"],
            self.gripper_configs[GripperType.enGripperType_Right]["gripper_name"],
        ]
        right_success = self.right_gripper._call_command(cmd, param, result)
        if left_success and right_success:
            rospy.loginfo(f"夹爪配置参数成功")
            return True
        else:
            rospy.loginfo(
                f"夹爪配置参数失败 left success={left_success} right success={right_success}"
            )
            return False

    def _init_gripper(self):
        cmd = "GripperInit"
        param = [
            self.gripper_configs[GripperType.enGripperType_Left]["gripper_id"],
            self.gripper_configs[GripperType.enGripperType_Left]["gripper_type"],
            self.gripper_configs[GripperType.enGripperType_Left]["connect_type"],
            self.gripper_configs[GripperType.enGripperType_Left]["uart_port"],
            self.gripper_configs[GripperType.enGripperType_Left]["slave_id"],
            self.gripper_configs[GripperType.enGripperType_Left]["baud_rate"],
            self.gripper_configs[GripperType.enGripperType_Left]["data_bit"],
            self.gripper_configs[GripperType.enGripperType_Left]["stop_bit"],
            self.gripper_configs[GripperType.enGripperType_Left]["parity"],
            self.gripper_configs[GripperType.enGripperType_Left]["connect_mode"],
        ]
        result = []
        left_success = self.left_gripper._call_command(cmd, param, result)

        result = []
        param = [
            self.gripper_configs[GripperType.enGripperType_Right]["gripper_id"],
            self.gripper_configs[GripperType.enGripperType_Right]["gripper_type"],
            self.gripper_configs[GripperType.enGripperType_Right]["connect_type"],
            self.gripper_configs[GripperType.enGripperType_Right]["uart_port"],
            self.gripper_configs[GripperType.enGripperType_Right]["slave_id"],
            self.gripper_configs[GripperType.enGripperType_Right]["baud_rate"],
            self.gripper_configs[GripperType.enGripperType_Right]["data_bit"],
            self.gripper_configs[GripperType.enGripperType_Right]["stop_bit"],
            self.gripper_configs[GripperType.enGripperType_Right]["parity"],
            self.gripper_configs[GripperType.enGripperType_Right]["connect_mode"],
        ]
        right_success = self.right_gripper._call_command(cmd, param, result)

        if left_success and right_success:
            rospy.loginfo(f"夹爪初始化成功")
            return True
        else:
            rospy.loginfo(
                f"夹爪初始化失败 left success={left_success} right success={right_success}"
            )
            return False

    def _connect_gripper(self):
        cmd = "Connect"
        param = [self.gripper_configs[GripperType.enGripperType_Left]["gripper_id"]]
        result = []
        left_success = self.left_gripper._call_command(cmd, param, result)

        param = [self.gripper_configs[GripperType.enGripperType_Right]["gripper_id"]]
        result = []
        right_success = self.right_gripper._call_command(cmd, param, result)
        if left_success and right_success:
            rospy.loginfo(f"夹爪连接成功")
            self.is_connecting = True
            return True
        else:
            rospy.loginfo(
                f"夹爪连接失败 left success={left_success} right success={right_success}"
            )
            self.is_connecting = False
            return False

    def _reconnect_gripper(self):
        """重连夹爪"""
        rospy.loginfo("尝试重新连接夹爪...")
        self.is_connecting = False

        # 重置错误计数
        with self.left_lock:
            self.left_gripper_data.error_count = 0
        with self.right_lock:
            self.right_gripper_data.error_count = 0

        success = self._connect_gripper()
        if success:
            rospy.loginfo("夹爪重连成功")
            self.reconnect_attempts = 0
        else:
            rospy.logwarn("夹爪重连失败")
            self.reconnect_attempts += 1

        return success

    def _check_and_reconnect(self, reconnect_reason):
        """检查并重连夹爪，无限重连直到成功"""
        rospy.loginfo("=== 开始夹爪重连过程 ===")

        while not rospy.is_shutdown():
            rospy.loginfo(f"重连原因: {reconnect_reason}")
            rospy.loginfo(
                f"第 {self.reconnect_attempts + 1} 次尝试重连，{self.reconnect_delay}秒后开始..."
            )
            rospy.sleep(self.reconnect_delay)

            success = self._reconnect_gripper()

            if success:
                rospy.loginfo("=== 夹爪重连成功 ===")
                break
            else:
                rospy.logwarn(f"=== 夹爪重连失败，将继续尝试 ===")
                # 继续循环，无限重试

    def _check_connection_status(self):
        """检查连接状态并触发重连"""
        need_reconnect = False
        reconnect_reason = ""

        with self.left_lock:
            if (
                self.left_gripper_data.error_count
                >= self.left_gripper_data.max_error_count
            ):
                need_reconnect = True
                reconnect_reason += "左夹爪连续错误次数超限; "
                self.left_gripper_data.error_count = 0  # 重置计数避免重复触发

        with self.right_lock:
            if (
                self.right_gripper_data.error_count
                >= self.right_gripper_data.max_error_count
            ):
                need_reconnect = True
                reconnect_reason += "右夹爪连续错误次数超限; "
                self.right_gripper_data.error_count = 0  # 重置计数避免重复触发

        if need_reconnect:
            rospy.logwarn(f"触发夹爪重连机制，原因: {reconnect_reason}")
            # 启动重连线程（无限重连）
            threading.Thread(
                target=self._check_and_reconnect, args=(reconnect_reason,), daemon=True
            ).start()

    def _initialize_gripper(self):
        """初始化夹爪"""
        # 获取夹爪的配置
        self.gripper_configs = rospy.get_param("~fleximind_gripper_node")

        # 初始化左右夹爪
        self.left_gripper = HuayanPluginGripper(
            self.gripper_configs[GripperType.enGripperType_Left]["box_id"],
            self.gripper_configs[GripperType.enGripperType_Left]["ip"],
            self.gripper_configs[GripperType.enGripperType_Left]["port"],
        )
        self.right_gripper = HuayanPluginGripper(
            self.gripper_configs[GripperType.enGripperType_Right]["box_id"],
            self.gripper_configs[GripperType.enGripperType_Right]["ip"],
            self.gripper_configs[GripperType.enGripperType_Right]["port"],
        )

        self.real_gripper_range_l = self.gripper_configs[
            GripperType.enGripperType_Left
        ]["range"]
        self.real_gripper_range_r = self.gripper_configs[
            GripperType.enGripperType_Right
        ]["range"]

        self._set_gripper_param()

        self._init_gripper()

        self._connect_gripper()

    # === 异步读取 === #

    def _publish_gripper_data(self):
        """发布夹爪数据"""
        current_time = rospy.Time.now()

        # 创建消息
        grippers_msg = GripperData()
        grippers_msg.header = Header()
        grippers_msg.header.stamp = current_time

        # 填充左夹爪数据
        with self.left_lock:
            grippers_msg.left_position = self.left_gripper_data.position
            grippers_msg.left_speed = self.gripper_configs[
                GripperType.enGripperType_Left
            ]["speed"]
            grippers_msg.left_force = self.gripper_configs[
                GripperType.enGripperType_Left
            ]["force"]

        # 填充右夹爪数据
        with self.right_lock:
            grippers_msg.right_position = self.right_gripper_data.position
            grippers_msg.right_speed = self.gripper_configs[
                GripperType.enGripperType_Right
            ]["speed"]
            grippers_msg.right_force = self.gripper_configs[
                GripperType.enGripperType_Right
            ]["force"]

        # 发布消息
        self.gripper_data_pub.publish(grippers_msg)

    async def _update_gripper_positions(self):
        """异步更新夹爪位置"""
        left_success, left_result = await self._get_left_gripper_position()
        right_success, right_result = await self._get_right_gripper_position()

        if left_success and left_result:
            with self.left_lock:
                self.left_gripper_data.position = int(left_result[1])
                self.left_gripper_data.error_count = 0  # 重置错误计数
        else:
            rospy.logwarn_throttle(DEFAULT_LOG_PERIOD_S, "获取左夹爪位置失败")
            with self.left_lock:
                self.left_gripper_data.error_count += 1

        if right_success and right_result:
            with self.right_lock:
                self.right_gripper_data.position = int(right_result[1])
                self.right_gripper_data.error_count = 0  # 重置错误计数
        else:
            rospy.logwarn_throttle(DEFAULT_LOG_PERIOD_S, "获取右夹爪位置失败")
            with self.right_lock:
                self.right_gripper_data.error_count += 1

        # 检查是否需要重连
        self._check_connection_status()

    def publish_gripper_info(self, event):
        """定时发布夹爪信息"""
        try:
            if self.is_publishing:
                # 异步更新位置并发布数据
                asyncio.run_coroutine_threadsafe(
                    self._update_gripper_positions(), self.loop
                ).add_done_callback(lambda _: self._publish_gripper_data())

            # 发布状态
            gripper_status = GripperStatus()
            gripper_status.is_connect = self.is_connecting
            self.gripper_status_pub.publish(gripper_status)

        except Exception as e:
            rospy.logerr_throttle(DEFAULT_LOG_PERIOD_S, f"发布夹爪信息异常: {str(e)}")

    async def _get_left_gripper_position(self):
        """异步获取左夹爪位置"""
        with self.left_lock:
            if self.left_gripper_data.is_busy:
                rospy.logdebug_throttle(
                    DEFAULT_LOG_PERIOD_S, "左夹爪正忙，跳过本次读取"
                )
                return
            self.left_gripper_data.is_busy = True

        cmd = "GetCatchPosition"
        param = [self.gripper_configs[GripperType.enGripperType_Left]["gripper_id"]]
        result = []

        try:
            success = await self.loop.run_in_executor(
                None, lambda: self.left_gripper._call_command(cmd, param, result)
            )
            return success, result
        except Exception as e:
            rospy.logerr(f"获取左夹爪位置异常: {str(e)}")
            return False, None
        finally:
            with self.left_lock:
                self.left_gripper_data.is_busy = False

    async def _get_right_gripper_position(self):
        """异步获取右夹爪位置"""
        with self.right_lock:
            if self.right_gripper_data.is_busy:
                rospy.logdebug_throttle(
                    DEFAULT_LOG_PERIOD_S, "右夹爪正忙，跳过本次读取"
                )
                return
            self.right_gripper_data.is_busy = True

        cmd = "GetCatchPosition"
        param = [self.gripper_configs[GripperType.enGripperType_Right]["gripper_id"]]
        result = []

        try:
            success = await self.loop.run_in_executor(
                None, lambda: self.right_gripper._call_command(cmd, param, result)
            )
            return success, result
        except Exception as e:
            rospy.logerr(f"获取右夹爪位置异常: {str(e)}")
            return False, None
        finally:
            with self.right_lock:
                self.right_gripper_data.is_busy = False

    # === 异步控制 === #

    def _run_async_loop(self):
        """在新线程中运行 asyncio 事件循环"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def left_gripper_callback(self, msg: DualJointStates):
        """左夹爪回调（触发异步任务）"""
        asyncio.run_coroutine_threadsafe(self._async_move_left_gripper(msg), self.loop)

    def right_gripper_callback(self, msg: DualJointStates):
        """右夹爪回调（触发异步任务）"""
        asyncio.run_coroutine_threadsafe(self._async_move_right_gripper(msg), self.loop)

    async def _async_move_left_gripper(self, msg: DualJointStates):
        """异步控制左夹爪"""
        current_time = rospy.Time.now()

        with self.left_lock:
            if (
                current_time - self.left_gripper_data.last_publish_time
            ) < rospy.Duration(self.control_timer_period):
                return

            if self.left_gripper_data.is_busy:
                rospy.logdebug_throttle(
                    DEFAULT_LOG_PERIOD_S, "左夹爪正忙，跳过本次控制"
                )
                return

            self.left_gripper_data.is_busy = True

        try:
            if msg and len(msg.left_joints.position) == DEFAULT_AXIS_NUM:
                param = [
                    self.gripper_configs[GripperType.enGripperType_Left]["gripper_id"],
                    int(msg.left_joints.position[GripperType.enGripperType_last_axis]),
                ]
                result = []

                # 异步执行阻塞操作
                success = await self.loop.run_in_executor(
                    None,
                    lambda: self.left_gripper._call_command(
                        "GripperCatchMoveTo", param, result
                    ),
                )

                if not success:
                    rospy.logwarn(f"左夹爪控制失败, param: {param}, result: {result}")
                    with self.left_lock:
                        self.left_gripper_data.error_count += 1
                else:
                    with self.left_lock:
                        self.left_gripper_data.error_count = 0  # 重置错误计数

        except Exception as e:
            rospy.logerr(f"左夹爪控制异常: {str(e)}")
            with self.left_lock:
                self.left_gripper_data.error_count += 1
        finally:
            with self.left_lock:
                self.left_gripper_data.last_publish_time = current_time
                self.left_gripper_data.is_busy = False

    async def _async_move_right_gripper(self, msg: DualJointStates):
        """异步控制右夹爪"""
        current_time = rospy.Time.now()

        with self.right_lock:
            if (
                current_time - self.right_gripper_data.last_publish_time
            ) < rospy.Duration(self.control_timer_period):
                return

            if self.right_gripper_data.is_busy:
                rospy.logdebug_throttle(
                    DEFAULT_LOG_PERIOD_S, "右夹爪正忙，跳过本次控制"
                )
                return

            self.right_gripper_data.is_busy = True

        try:
            if msg and len(msg.right_joints.position) == DEFAULT_AXIS_NUM:
                param = [
                    self.gripper_configs[GripperType.enGripperType_Right]["gripper_id"],
                    int(msg.right_joints.position[GripperType.enGripperType_last_axis]),
                ]
                result = []

                # 异步执行阻塞操作
                success = await self.loop.run_in_executor(
                    None,
                    lambda: self.right_gripper._call_command(
                        "GripperCatchMoveTo", param, result
                    ),
                )

                if not success:
                    rospy.logwarn(f"右夹爪控制失败, param: {param}, result: {result}")
                    with self.right_lock:
                        self.right_gripper_data.error_count += 1
                else:
                    with self.right_lock:
                        self.right_gripper_data.error_count = 0  # 重置错误计数

        except Exception as e:
            rospy.logerr(f"右夹爪控制异常: {str(e)}")
            with self.right_lock:
                self.right_gripper_data.error_count += 1
        finally:
            with self.right_lock:
                self.right_gripper_data.last_publish_time = current_time
                self.right_gripper_data.is_busy = False


def main():
    rospy.loginfo("启动夹爪控制节点")
    node = GripperControlNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        node.loop.stop()
        rospy.loginfo("用户终止节点")
    finally:
        node.loop.stop()
        rospy.loginfo("节点关闭")


if __name__ == "__main__":
    main()
