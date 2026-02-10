import time
import math
import socket
import queue
from typing import List
import threading
from fleximind_robot.huayan_robot import CPScontrol
import rospy
import logging
import os
from datetime import datetime
import rospy
import logging
import os
from datetime import datetime


class RealTimeServoController:
    def __init__(
        self,
        ip,
        box_id,
        name,
        interp_ratios=3,
        max_velocity=1.0,
        max_accel=0.5,
        max_jerk=2.0,
        serve_time=0.004,
        dp=8000,
        lookahead_time=0.3,
        buffer_size=1000,
        log_dir="/home/robot/fleximind-ros1/servo_logs",
    ):
        self.ip = ip
        self.box_id = box_id
        self.name = name
        self.input_queue = queue.Queue(maxsize=buffer_size)  # 原始点位输入队列
        self.running = False
        self.sending_thread = None
        self.planner = TrajectoryPlanner(
            interp_ratios, max_velocity, max_accel, max_jerk
        )
        self.last_input_point = None
        self.servo_client = None
        self.connected = False
        self.lock = threading.Lock()
        self.is_sending = False  # 控制发送开关,默认关闭

        self.servo_time = serve_time  # 控制周期
        self.dp = dp  # 柔顺度参数
        self.lookahead_time = lookahead_time  # 前瞻时间，实际周期由机器人控制器决定
        self.is_sdk = False  # 使用SDK的方式控制servoj

    def start(self):
        # 连接机器人
        if not self.connected:
            try:
                if self.is_sdk:
                    self.arm = CPScontrol(
                        name="arm",
                        box_id=self.box_id,
                        ip=self.ip,
                        port=8892,
                    )
                else:
                    self.servo_client = socket.socket()
                    self.servo_client.settimeout(3)
                    self.servo_client.connect((self.ip, 8892))

                self.connected = True
                # cmd = f"StartDebugLog,0,0,;"
                # self.servo_client.send(cmd.encode())
                print(f"[{self.name}] 连接成功: {self.ip}:8892")
            except Exception as e:
                self.connected = False
                print(f"[{self.name}] 连接失败: {e}")
                if self.servo_client:
                    try:
                        self.servo_client.close()
                    except Exception:
                        pass
                self.servo_client = None
                return
        if not self.running:
            self.running = True
            self.sending_thread = threading.Thread(
                target=self._sending_worker, daemon=True
            )
            self._setup_logging("/home/robot/fleximind-ros1/servo_logs")
            self.sending_thread.start()

    def stop(self):
        self.running = False
        if self.sending_thread:
            self.sending_thread.join(timeout=0.5)
        if self.servo_client:
            try:
                self.servo_client.close()
            except Exception as e:
                print(f"[{self.name}] 关闭socket异常: {e}")
            self.servo_client = None
        self.connected = False

    def set_sending(self, sending: bool):
        with self.lock:
            if sending:
                print(f"{self.name} 伺服控制已开启")
            else:
                print(f"{self.name} 伺服控制已关闭")
                with self.input_queue.mutex:
                    self.input_queue.queue.clear()  # 清空队列
                self.last_input_point = None  # 重置最后一个点
            self.is_sending = sending

    # def add_point(self, point):
    #     """优化版：仅两点插值，队列为空直接入队，否则插值后入队"""
    #     try:
    #         if self.last_input_point is None:
    #             # 队列为空，直接放入当前点
    #             self.input_queue.put_nowait(point)
    #             self.last_input_point = point
    #         else:
    #             # 上一个点不为空，取出上一个点和当前点做比较,差值绝对值小于0.1,使用上一个点形成两个点进行插值
    #             if all(
    #                 abs(point[i] - self.last_input_point[i]) < 0.1 for i in range(6)
    #             ):
    #                 # 差值小于0.1，直接使用上一个点
    #                 point = self.last_input_point
    #             prev_point = self.last_input_point
    #             if prev_point is None and not self.input_queue.empty():
    #                 prev_point = self.input_queue.queue[-1]  # 取队列最后一个点
    #             segment = [prev_point, point]
    #             # s-curve规划,优先使用
    #             interpolated_segment = self.planner.s_curve_interpolation(segment)
    #             # 跳过第一个点，避免重复
    #             for p in interpolated_segment[1:]:
    #                 try:
    #                     self.input_queue.put_nowait(p)
    #                 except queue.Full:
    #                     try:
    #                         self.input_queue.get_nowait()
    #                         self.input_queue.put_nowait(p)
    #                     except queue.Empty:
    #                         pass
    #             self.last_input_point = point
    #     except Exception as e:
    #         print(f"[{self.name}] 插值规划错误: {e}")
    
    def add_point(self, point):
        """优化版点位添加，确保队列始终有数据"""
        try:
            # 更新最后输入点
            self.last_input_point = point
            
            # 直接放入队列，不做过多的插值计算（避免阻塞）
            try:
                self.input_queue.put_nowait(point)
            except queue.Full:
                # 队列满时，丢弃最旧的点位
                try:
                    self.input_queue.get_nowait()
                    self.input_queue.put_nowait(point)
                except queue.Empty:
                    pass
                    
            # 每添加一定数量的点打印一次状态
            if self.input_queue.qsize() % 50 == 0:
                print(f"[{self.name}] 添加点位, 队列大小: {self.input_queue.qsize()}")
                
        except Exception as e:
            print(f"[{self.name}] 添加点位错误: {e}")

    def _setup_logging(self, log_dir):
        """设置日志系统"""
        # 创建日志目录
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        # 生成唯一的日志文件名（包含时间戳和控制器名称）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f"{self.ip}_{timestamp}.txt"
        log_path = os.path.join(log_dir, log_filename)

        # 创建logger
        self.logger = logging.getLogger(f"{self.ip}_{timestamp}")
        self.logger.setLevel(logging.INFO)

        # 防止日志重复
        if not self.logger.handlers:
            # 文件处理器
            file_handler = logging.FileHandler(log_path, encoding="utf-8")
            file_handler.setLevel(logging.INFO)

            # 控制台处理器（可选，只记录INFO及以上级别）
            console_handler = logging.StreamHandler()
            console_handler.setLevel(logging.WARNING)  # 只在终端显示警告和错误

            # 格式化器
            formatter = logging.Formatter(
                "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
            )
            file_handler.setFormatter(formatter)
            console_handler.setFormatter(formatter)

            self.logger.addHandler(file_handler)
            self.logger.addHandler(console_handler)

        self.logger.info(f"Servo controller {self.ip} started, logging to {log_path}")

    def _sending_worker(self):
        """点位发送线程"""
        try:
            if not self.servo_client or not self.connected:
                print(f"[{self.name}] 伺服socket未连接，无法发送点位")
                return
            try:
                if self.is_sdk:
                    result = self.arm.start_servo(
                        self.box_id, 0, self.dp, self.lookahead_time
                    )
                    if not result:
                        rospy.loginfo("start_servo failed")
                        return
                    print(f"[{self.name}] SDK伺服控制启动成功")
                else:
                    start_cmd = f"StartServo,0,{self.dp},{self.lookahead_time},;"
                    self.servo_client.send(start_cmd.encode())
            except Exception as e:
                print(f"[{self.name}] StartServo 指令发送失败: {e}")
                return

            print(f"[{self.name}] 开始伺服控制循环, 控制周期: {self.servo_time*1000}ms")
            
            # 超时保护相关变量
            last_successful_send_time = time.time()
            timeout_protection_threshold = 0.3  # 保护阈值
            emergency_point = None
            in_emergency_mode = False
            
            cycle_count = 0
            
            while self.running:
                cycle_count += 1
                
                if not self.is_sending:
                    time.sleep(0.05)
                    continue

                loop_start_time = time.perf_counter()
                current_time = time.time()
                
                # 超时保护检查
                time_since_last_success = current_time - last_successful_send_time
                if time_since_last_success > timeout_protection_threshold:
                    if not in_emergency_mode:
                        print(f"[{self.name}] 紧急: 超过{time_since_last_success*1000:.1f}ms未成功发送, 启动超时保护!")
                        in_emergency_mode = True
                    
                    # 紧急模式下，确保有点位可发送
                    if emergency_point is None and self.last_input_point:
                        emergency_point = self.last_input_point
                        print(f"[{self.name}] 设置紧急点位: {emergency_point}")
                
                # 优先尝试从队列获取点位
                point = None
                
                try:
                    point = self.input_queue.get_nowait()
                    # 从队列获取到点位，退出紧急模式
                    if in_emergency_mode:
                        print(f"[{self.name}] 恢复正常模式，从队列获取点位")
                        in_emergency_mode = False
                    
                except queue.Empty:
                    # 队列为空时，如果有紧急点位使用紧急点位，否则使用last_input_point
                    if emergency_point is not None:
                        point = emergency_point
                        if in_emergency_mode and cycle_count % 100 == 0:
                            print(f"[{self.name}] 紧急模式下使用紧急点位")
                    elif self.last_input_point:
                        point = self.last_input_point
                    else:
                        # 没有任何可用点位，跳过
                        time.sleep(0.001)
                        continue

                # 发送点位
                try:
                    if self.is_sdk:
                        result = self.arm.push_servo(self.box_id, 0, point)
                        if not result:
                            print(f"[{self.name}] SDK push_servo 返回失败")
                            continue
                    else:
                        cmd = f"PushServoJ,0,{point[0]:.6f},{point[1]:.6f},{point[2]:.6f},{point[3]:.6f},{point[4]:.6f},{point[5]:.6f},;"
                        self.servo_client.send(cmd.encode())

                    # 发送成功，更新时间戳
                    last_successful_send_time = time.time()
                    emergency_point = point   # 更新紧急点位为当前成功发送的点位
                    
                    # 退出紧急模式
                    if in_emergency_mode:
                        print(f"[{self.name}] 发送成功，退出紧急模式")
                        in_emergency_mode = False
                        
                except Exception as e:
                    print(f"[{self.name}] 发送失败: {e}")
                    # 发送失败时不更新last_successful_send_time，让超时保护机制处理

                # 频率控制
                elapsed = time.perf_counter() - loop_start_time
                sleep_time = max(0.001, self.servo_time - elapsed)
                time.sleep(sleep_time)

        except Exception as e:
            print(f"[{self.name}] 伺服控制线程错误: {e}")
        finally:
            print(f"[{self.name}] 伺服控制线程结束")
            self.connected = False


#####################servo#######################
class TrajectoryPlanner:
    def __init__(self, interp_ratio, max_velocity=1.0, max_accel=0.5, max_jerk=2.0):
        """
        :param interp_ratio: 插入点数数量
        """
        self.interp_ratio = interp_ratio  # 插值倍数

        # 运动参数（可根据实际调整）
        self.max_velocity = max_velocity  # 最大速度 (rad/s or m/s)
        self.max_accel = max_accel  # 最大加速度 (rad/s² or m/s²)
        self.max_jerk = max_jerk  # 最大加加速度 (rad/s³ or m/s³)

    def s_curve_interpolation(self, waypoints: List[List[float]]) -> List[List[float]]:
        """
        带固定插值点数的S曲线速度规划

        :param waypoints: 原始轨迹点列表
        :return: 插值后的轨迹点列表
        """
        if not waypoints:
            return []

        trajectory = []
        num_joints = len(waypoints[0])

        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]

            # 计算关节位移和最大位移
            displacements = [end[j] - start[j] for j in range(num_joints)]
            max_displacement = max(abs(d) for d in displacements)

            # 固定插值点数（如20ms→1ms就是20个点）
            num_points = self.interp_ratio

            # 微小位移处理（插入相同点）
            if max_displacement < 1e-6:
                for _ in range(num_points):
                    trajectory.append(start.copy())
                continue

            # S曲线参数计算（7段式）
            jerk = self.max_jerk
            t_acc = self.max_accel / jerk  # 加/减加速段时间
            v_max = min(
                self.max_velocity,
                jerk * t_acc**2
                + self.max_accel
                * (max_displacement - jerk * t_acc**3)
                / max_displacement,
            )
            # 打印加加速度、加速度、最大速度
            # print(f"[S曲线段{i}] jerk(加加速度): {jerk}, max_accel(加速度): {self.max_accel}, max_velocity(最大速度): {self.max_velocity}, v_max(本段最大速度): {v_max}")

            # 计算各段时间
            if v_max < self.max_velocity:
                # 无法达到最大速度（短距离运动）
                t_const = 0
                t_acc = (max_displacement / jerk) ** (1 / 3)  # 重新计算加速时间
            else:
                # 能达到最大速度
                t_const = (max_displacement - jerk * t_acc**3) / v_max

            total_time = 4 * t_acc + t_const  # 总时间

            # 生成固定数量的插值点
            for k in range(num_points):
                t = (k / num_points) * total_time  # 归一化时间

                # 计算S曲线路径参数s (0~1)
                if t < t_acc:
                    # 加加速段 (jerk > 0)
                    s = jerk * t**3 / 6
                elif t < 2 * t_acc:
                    # 减加速段 (jerk < 0)
                    dt = t - t_acc
                    s = (
                        jerk * t_acc**3 / 6
                        + 0.5 * jerk * t_acc**2 * dt
                        - jerk * dt**3 / 6
                    )
                elif t < 2 * t_acc + t_const:
                    # 匀速段
                    dt = t - 2 * t_acc
                    s = jerk * t_acc**3 / 3 + v_max * dt
                elif t < 3 * t_acc + t_const:
                    # 加减速段 (jerk < 0)
                    dt = t - (2 * t_acc + t_const)
                    s = (
                        jerk * t_acc**3 / 3
                        + v_max * t_const
                        + v_max * dt
                        - jerk * dt**3 / 6
                    )
                else:
                    # 减减速段 (jerk > 0)
                    dt = t - (3 * t_acc + t_const)
                    term = jerk * (t_acc - dt) ** 3 / 6
                    s = max_displacement - term

                # 归一化路径参数
                s_norm = min(s / max_displacement, 1.0)  # 确保不超过1

                # 计算各关节位置
                point = [
                    start[j] + s_norm * displacements[j] for j in range(num_joints)
                ]
                trajectory.append(point)

        # 添加最后一个点
        trajectory.append(waypoints[-1].copy())
        return trajectory

    def trapezoidal_interpolation(
        self, waypoints: List[List[float]]
    ) -> List[List[float]]:
        """
        带固定插值点数的梯形速度规划

        :param waypoints: 原始轨迹点列表(20ms间隔)
        :return: 插值后的轨迹点列表(1ms间隔)
        """
        if not waypoints:
            return []

        trajectory = []
        num_joints = len(waypoints[0])

        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]

            # 计算关节位移和最大位移
            displacements = [end[j] - start[j] for j in range(num_joints)]
            max_displacement = max(abs(d) for d in displacements)

            # 每个段固定插值点数（20ms→1ms就是20个点）
            num_points = self.interp_ratio

            # 微小位移处理（保持梯形规划但点数固定）
            if max_displacement < 1e-6:
                # 插入固定数量的相同点
                for _ in range(num_points):
                    trajectory.append(start.copy())
                continue

            # 计算梯形速度参数
            t_acc = self.max_velocity / self.max_accel  # 加速时间
            s_acc = 0.5 * self.max_accel * t_acc**2  # 加速段位移

            if 2 * s_acc <= max_displacement:
                # 完整梯形（加速+匀速+减速）
                s_const = max_displacement - 2 * s_acc
                t_const = s_const / self.max_velocity
                total_time = 2 * t_acc + t_const
            else:
                # 三角型（无法达到最大速度）
                t_acc = math.sqrt(max_displacement / self.max_accel)
                t_const = 0
                total_time = 2 * t_acc

            # 生成固定数量的插值点
            for k in range(num_points):
                t = (k / num_points) * total_time  # 按比例分配时间

                if t < t_acc:
                    # 加速段
                    s = 0.5 * self.max_accel * t**2
                elif t < t_acc + t_const:
                    # 匀速段
                    s = s_acc + self.max_velocity * (t - t_acc)
                else:
                    # 减速段
                    t_dec = t - (t_acc + t_const)
                    s = max_displacement - 0.5 * self.max_accel * (t_acc - t_dec) ** 2

                # 计算各关节位置
                ratio = s / max_displacement if max_displacement > 0 else 0
                point = [start[j] + ratio * displacements[j] for j in range(num_joints)]
                trajectory.append(point)

        # 添加最后一个点
        trajectory.append(waypoints[-1].copy())
        return trajectory
