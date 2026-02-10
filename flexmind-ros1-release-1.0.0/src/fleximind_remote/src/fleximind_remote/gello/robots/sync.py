import socket
import threading
import time
import sys
import numpy as np
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [%(threadName)s] %(message)s",
    handlers=[logging.StreamHandler()],
)


class RobotSync:
    def __init__(
        self, host="10.20.200.85", port=10003, robot_id=0, target_positions=None
    ):
        self.logger = logging.getLogger("SINGLE_SYNC")
        self.host = host
        self.port = port
        self.robot_id = robot_id
        self.socket = None
        self.connected = False
        self.long_move_active = False
        self.long_move_interval = 0.05  # 150ms小于200ms
        if target_positions is None:
            target_positions = np.zeros(6)
        self.target_positions = np.array(
            target_positions, copy=True
        )  # 默认目标关节位置
        self.position_tolerance = 1.0  # 关节位置容差（度）
        self.read_interval = 0.1  # 读取关节位置的时间间隔
        self.position_reached = False  # 标记是否到达目标位置
        self.abort_flag = True  # 中止标志
        # self.keyboard_thread = None
        self.send_fail_count = 0  # 发送失败计数器

    def connect(self):
        """建立socket连接"""
        self.disconnect()  # 确保先断开已有连接
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(0.5)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"成功连接到服务器 {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """安全断开连接"""
        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except:
                pass
            finally:
                try:
                    self.socket.close()
                except:
                    pass
                self.socket = None

        self.connected = False
        if hasattr(self, "long_move_active"):
            self.long_move_active = False
        print("连接已断开")

    def send_command(self, command, *params):
        """发送指令并处理响应"""
        if not self.connected:
            print("未连接到服务器")
            return False, "未连接到服务器"

        try:
            # 构造命令字符串
            cmd_str = f"{command},{self.robot_id}"
            for param in params:
                cmd_str += f",{param}"
            cmd_str += ";"

            # print(f"发送: {cmd_str}")
            self.socket.sendall(cmd_str.encode("utf-8"))
            self.socket.settimeout(0.8)
            # 接收响应
            response = self.socket.recv(1024).decode("utf-8").strip()
            # print(f"接收: {response}")

            # 解析响应
            if response.startswith(f"{command},OK"):
                self.send_fail_count = 0  # 重置失败计数
                return True, response
            elif response.startswith(f"{command},Fail"):
                parts = response.split(",")
                if len(parts) >= 4:
                    error_code = parts[2]
                    error_msg = ",".join(parts[3:]).rstrip(";")
                    return False, f"错误 {error_code}: {error_msg}"
            return False, "未知响应格式"
        except Exception as e:
            print(f"通信错误: {e}")
            self.connected = False
            self.send_fail_count += 1  # 增加失败计数
            return False, str(e)

    def read_act_pos(self):
        """读取实际关节位置"""
        if not self.connected:
            return False, None, "未连接到服务器"

        success, response = self.send_command("ReadActPos")
        if not success:
            return False, None, response

        # 解析响应 - 提取前6个关节位置值
        parts = response.split(",")
        if len(parts) < 8:  # 命令,OK + 6个关节位置
            return False, None, f"响应格式无效: {response}"

        try:
            joint_positions = np.array(
                [
                    float(parts[2]),
                    float(parts[3]),
                    float(parts[4]),
                    float(parts[5]),
                    float(parts[6]),
                    float(parts[7]),
                ]
            )
            return True, joint_positions, "读取成功"
        except Exception as e:
            return False, None, f"解析位置失败: {e}"

    def movej_to(self):
        """发送MoveJTo指令"""
        # 构建并发送指令
        return self.send_command("MoveJTo", *self.target_positions)

    def start_movement(self, target_positions):
        """启动移动到目标位置的全过程"""
        # 设置目标位置
        self.target_positions = target_positions.copy()
        print(f"目标位置: {self.target_positions}")

        # 连接服务器
        if not self.connect():
            return False, "连接失败"

        # 发送MoveJTo指令
        success, response = self.movej_to()
        if not success:
            # self.disconnect()
            return False, f"MoveJTo失败: {response}"

        # 重置状态标志
        self.position_reached = False
        self.long_move_active = True
        self.send_fail_count = 0
        last_send_time = time.time()
        last_read_time = time.time()
        start_time = time.time()

        try:
            print("开始移动...")

            while True:
                current_time = time.time()

                # 1. 发送LongMoveEvent (主线程)
                if True:
                    # 发送长点动指令
                    success, error = self.send_command("LongMoveEvent")

                    if not success:
                        print(f"LongMoveEvent失败: {error}")
                        self.send_fail_count += 1
                        # 连续失败3次则重新发送MoveJTo(改为不重新发送直接退出同步)
                        if self.send_fail_count >= 3:
                            # print("连续失败，尝试重新发送MoveJTo...")
                            # success, response = self.movej_to()
                            # if not success:
                            # return False, f"重发MoveJTo失败: {response}"
                            # self.send_fail_count = 0  # 重置失败计数
                            return False, f"发送LongMoveEvent失败: {response}"
                    last_send_time = current_time

                    # 打印进度
                    elapsed = current_time - start_time
                    print(
                        f"\r持续发送同步指令: {elapsed:.1f}s | 失败计数: {self.send_fail_count}",
                        end="",
                        flush=True,
                    )

                # 2. 读取位置并检查
                if current_time - last_read_time >= self.read_interval:
                    # 读取实际位置
                    success, current_positions, message = self.read_act_pos()
                    if success:
                        # 计算位置差
                        position_diff = np.abs(
                            current_positions - self.target_positions
                        )
                        max_diff = np.max(position_diff)

                        # 检查是否到达目标
                        if np.all(position_diff <= self.position_tolerance):
                            self.position_reached = True
                            print(f"\n到达目标! 最终误差: {max_diff:.2f}°")
                            return True, "成功到达目标位置"

                    last_read_time = current_time

                # 3. 检查中止标志
                if self.abort_flag:
                    print("\n用户中止移动")
                    return False, "用户中止"

                # 短时间休眠以避免CPU过高
                time.sleep(0.01)

        finally:
            # 清理状态
            self.long_move_active = False
            self.disconnect()


class DualRobotSync:
    def __init__(
        self,
        hosts=("10.20.200.85", "10.20.200.86"),
        port=10003,
        robot_ids=(0, 0),
        target_positions1=None,
        target_positions2=None,
    ):
        if target_positions1 is None:
            target_positions1 = np.zeros(6)
        if target_positions2 is None:
            target_positions2 = np.zeros(6)
        self.logger = logging.getLogger("DUAL_SYNC")
        # 设置默认目标位置
        self.target_positions1 = np.array(target_positions1, copy=True)
        self.target_positions2 = np.array(target_positions2, copy=True)

        # 创建两个机器人实例
        self.robot1 = RobotSync(
            host=hosts[0],
            port=port,
            robot_id=robot_ids[0],
            target_positions=self.target_positions1.copy(),
        )
        self.robot2 = RobotSync(
            host=hosts[1],
            port=port,
            robot_id=robot_ids[1],
            target_positions=self.target_positions2.copy(),
        )

        # 双机器人控制参数
        self.position_tolerance = 1.0  # 位置容差
        self.long_move_interval = 0.05  # 发送同步指令间隔
        self.read_interval = 0.1  # 读取位置间隔
        self.abort_flag = True  # 中止标志
        self.sync_counter = 0  # 同步计数器

    def connect_both(self):
        """同时连接两个机器人"""
        success1 = self.robot1.connect()
        success2 = self.robot2.connect()
        return success1 and success2

    def disconnect_both(self):
        """同时断开两个机器人连接"""
        self.robot1.disconnect()
        self.robot2.disconnect()

    def send_to_both(self, command, *params):
        try:
            """向两个机器人发送指令，先发85，再发86"""
            # 发送到第一个机器人 (85)
            success1, resp1 = self.robot1.send_command(command, *params)
            if not success1:
                return False, f"机器人1({self.robot1.host})发送失败: {resp1}"

            # 立即发送到第二个机器人 (86)
            success2, resp2 = self.robot2.send_command(command, *params)
            if not success2:
                return False, f"机器人2({self.robot2.host})发送失败: {resp2}"

            return True, f"机器人1: {resp1}, 机器人2: {resp2}"
        except Exception as e:
            return False, f"同时发送两个机器臂指令失败，错误信息{e}"

    def movej_to_both(self):
        """向两个机器人发送MoveJTo指令"""
        # 为每个机器人设置目标位置
        try:
            # 先发85
            success1, resp1 = self.robot1.movej_to()
            if not success1:
                return False, f"机器人1 MoveJTo失败: {resp1}"

            # 立即发86
            success2, resp2 = self.robot2.movej_to()
            if not success2:
                return False, f"机器人2 MoveJTo失败: {resp2}"

            return True, "双机器人MoveJTo指令发送成功"
        except Exception as e:
            return False, f"同时发送两个机器臂MoveJTo指令失败，错误信息{e}"

    def start_sync_movement(self, target_positions1, target_positions2):
        """启动两个机器人的同步移动"""
        # 连接两个机器人
        if not self.connect_both():
            return False, "连接失败"

        self.robot1.target_positions = target_positions1.copy()
        self.robot2.target_positions = target_positions2.copy()
        # 发送初始MoveJTo指令
        success, response = self.movej_to_both()
        if not success:
            # self.disconnect_both()
            return False, f"双机器人MoveJTo失败: {response}"

        # 重置状态
        self.robot1.position_reached = False
        self.robot2.position_reached = False
        self.robot1.long_move_active = True
        self.robot2.long_move_active = True
        self.robot1.send_fail_count = 0
        self.robot2.send_fail_count = 0
        self.abort_flag = False
        self.sync_counter = 0

        last_send_time = time.time()
        last_read_time = time.time()
        start_time = time.time()

        try:
            print("开始双机器人同步移动...")

            while True:
                current_time = time.time()
                elapsed = current_time - start_time
                self.sync_counter += 1

                # 1. 发送同步指令
                if True:
                    # 先给85发
                    success1, resp1 = self.robot1.send_command("LongMoveEvent")
                    # 马上给86发
                    success2, resp2 = self.robot2.send_command("LongMoveEvent")
                    if not success1:
                        self.robot1.send_fail_count += 1
                    else:
                        self.robot1.send_fail_count = 0

                    if not success2:
                        self.robot2.send_fail_count += 1
                    else:
                        self.robot2.send_fail_count = 0

                    time.sleep(0.003)

                    # 检查连续失败
                    if (
                        self.robot1.send_fail_count >= 3
                        or self.robot2.send_fail_count >= 3
                    ):
                        failed_robot = (
                            "robot1" if self.robot1.send_fail_count >= 3 else "robot2"
                        )
                        print(
                            f"机器人 {failed_robot}（{getattr(self, failed_robot).host}）连续失败，尝试重新发送 MoveJTo..."
                        )
                        movej_success, movej_response = self.movej_to_both()
                        if not movej_success:
                            return (
                                False,
                                f"机器人 {failed_robot} 重发 MoveJTo 失败: {movej_response}",
                            )
                        self.send_fail_count = 0  # 重置失败计数

                    last_send_time = current_time

                    # 显示状态
                    status_line = (
                        f"\r同步次数: {self.sync_counter} | "
                        f"时间: {elapsed:.1f}s | "
                        f"机器人1失败: {self.robot1.send_fail_count} | "
                        f"机器人2失败: {self.robot2.send_fail_count}"
                    )
                    print(status_line, end="", flush=True)

                # 2. 读取位置状态
                if current_time - last_read_time >= self.read_interval:
                    # 读取机器人1位置
                    success1, pos1, msg1 = self.robot1.read_act_pos()
                    if success1:
                        diff1 = np.abs(pos1 - self.robot1.target_positions)
                        max_diff1 = np.max(diff1)
                        self.robot1.position_reached = np.all(
                            diff1 <= self.position_tolerance
                        )

                    # 读取机器人2位置
                    success2, pos2, msg2 = self.robot2.read_act_pos()
                    if success2:
                        diff2 = np.abs(pos2 - self.robot2.target_positions)
                        max_diff2 = np.max(diff2)
                        self.robot2.position_reached = np.all(
                            diff2 <= self.position_tolerance
                        )

                    # 检查是否都到达目标
                    if self.robot1.position_reached and self.robot2.position_reached:
                        print(
                            f"\n双机器人均已到达目标位置! 机器人1误差: {max_diff1:.2f}°, 机器人2误差: {max_diff2:.2f}°"
                        )
                        return True, "同步移动成功"

                    last_read_time = current_time
                    time.sleep(0.002)

                # 3. 检查中止标志
                if self.abort_flag:
                    print("\n用户中止同步移动")
                    return False, "用户中止"

        finally:
            # 清理状态
            self.robot1.long_move_active = False
            self.robot2.long_move_active = False
            self.disconnect_both()
