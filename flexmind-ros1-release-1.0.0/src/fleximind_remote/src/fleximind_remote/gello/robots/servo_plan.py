import os
import sys
import time
import math
import csv
import socket
import numpy as np
from fleximind_remote.gello.robots.CPS import CPSClient
from typing import List, Tuple, Union

#####Specify robot pose#####
robot_pose_list = []
robot_joint_list = []
robot_ip = "192.168.189.129"


#####################Test#######################
def load_joint_trajectory(file_path, arm="left"):
    """
    从文件加载关节轨迹数据并转换为标准格式

    参数:
        file_path: 数据文件路径
        arm: 要加载的手臂('left'或'right')

    返回:
        waypoints列表，格式如:
        [
            [j1, j2, j3, j4, j5, j6],
            [j1, j2, j3, j4, j5, j6],
            ...
        ]
    """
    waypoints = []

    try:
        with open(file_path, "r") as file:
            for line in file:
                line = line.strip()
                if not line:
                    continue

                # 检查是否是目标手臂的数据行
                if f"{arm}_joints" in line:
                    # 提取关节数据部分
                    joints_str = line.split(":")[1].strip()

                    # 转换字符串为浮点数列表
                    joints = [
                        float(x.strip()) for x in joints_str.strip("[]").split(",")
                    ]

                    # 确保有6个关节值
                    if len(joints) == 6:
                        waypoints.append(joints)
                    else:
                        print(f"警告: 忽略无效数据行(关节数不为6): {line}")

    except FileNotFoundError:
        print(f"错误: 文件 {file_path} 未找到")
        return None
    except Exception as e:
        print(f"处理文件时发生错误: {str(e)}")
        return None

    return waypoints


#####################servo 1#######################
class MotionPlanner:
    def __init__(self, robot_interface):
        """
        运动轨迹规划与控制器

        :param robot_interface: 机器人控制接口对象
        """
        self.robot = robot_interface
        self.servo_time = 0.02  # 默认伺服周期(s)
        self.lookahead_time = 0.1  # 默认前瞻时间(s)
        self.max_accel = 360.0  # 默认最大加速度(rad/s²)
        self.max_velocity = 60.0  # 默认最大速度(rad/s)

    def trapezoidal_velocity_planning(
        self, waypoints: List[List[float]]
    ) -> List[List[float]]:
        """
        梯形速度规划（修改版：微小位移段保持原有点数）

        :param waypoints: 原始轨迹点列表
        :return: 插值后的轨迹点列表
        """
        trajectory = []
        num_joints = len(waypoints[0])

        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]

            # 计算关节位移和最大位移
            displacements = [end[j] - start[j] for j in range(num_joints)]
            max_displacement = max(abs(d) for d in displacements)

            # 计算默认插值点数（保持与正常段相同的密度）
            default_points = int(self.default_segment_time / self.servo_time) + 1

            # 微小位移处理（不再跳过，而是保持点数）
            if max_displacement < 1e-6:
                # 保持默认点数，所有点相同
                for _ in range(default_points):
                    trajectory.append(start.copy())
                continue

            # 正常梯形速度规划
            t_acc = self.max_velocity / self.max_accel  # 加速时间
            s_acc = 0.5 * self.max_accel * t_acc**2  # 加速段位移

            if 2 * s_acc <= max_displacement:
                # 可达到最大速度
                s_const = max_displacement - 2 * s_acc  # 匀速段位移
                t_const = s_const / self.max_velocity  # 匀速段时间
                total_time = 2 * t_acc + t_const
            else:
                # 无法达到最大速度
                t_acc = math.sqrt(max_displacement / self.max_accel)
                t_const = 0
                total_time = 2 * t_acc

            # 确保最小点数（与微小位移段保持一致）
            num_points = max(int(total_time / self.servo_time) + 1, default_points)

            # 生成轨迹点
            for k in range(num_points):
                t = min(k * self.servo_time, total_time)  # 防止最后一步超限

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
                ratio = s / max_displacement
                point = [start[j] + ratio * displacements[j] for j in range(num_joints)]
                trajectory.append(point)

        # 添加最后一个点
        if waypoints:
            trajectory.append(waypoints[-1].copy())

        return trajectory

    def s_curve_velocity_planning(
        self, waypoints: List[List[float]]
    ) -> List[List[float]]:
        """
        S形速度规划（7段S曲线）

        :param waypoints: 原始轨迹点列表
        :return: 插值后的轨迹点列表
        """
        trajectory = []
        num_joints = len(waypoints[0])

        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]

            # 计算关节位移和最大位移
            displacements = [end[j] - start[j] for j in range(num_joints)]
            max_displacement = max(abs(d) for d in displacements)

            # 忽略微小移动
            if max_displacement < 1e-6:
                continue

            # S曲线参数计算
            jerk = 0.5 * self.max_accel**2 / self.max_velocity  # 加加速度
            t1 = self.max_accel / jerk  # 加加速段时间
            t2 = max(
                0, (max_displacement - self.max_accel**2 / jerk) / self.max_velocity
            )  # 匀速段时间

            # 各段时间计算
            t3 = t1  # 减加速段时间
            t4 = 0  # 匀速段时间（已包含在t2）
            t5 = t1  # 加减速段时间
            t6 = t2  # 匀减速段时间
            t7 = t1  # 减减速段时间

            total_time = t1 + t2 + t3 + t5 + t6 + t7

            # 生成轨迹点
            num_points = int(total_time / self.servo_time) + 1
            for k in range(num_points):
                t = k * self.servo_time

                # 计算S曲线路径参数s (0~1)
                if t < t1:
                    # 加加速段
                    s = jerk * t**3 / 6
                elif t < t1 + t2:
                    # 匀速段1
                    s1 = jerk * t1**3 / 6
                    s = s1 + 0.5 * self.max_accel * (t - t1) ** 2
                elif t < t1 + t2 + t3:
                    # 减加速段
                    s1 = jerk * t1**3 / 6
                    s2 = 0.5 * self.max_accel * t2**2
                    dt = t - t1 - t2
                    s = (
                        s1
                        + s2
                        + self.max_accel * t2 * dt
                        + 0.5 * self.max_accel * dt**2
                        - jerk * dt**3 / 6
                    )
                elif t < t1 + t2 + t3 + t5:
                    # 匀速段2
                    s1 = jerk * t1**3 / 6
                    s2 = 0.5 * self.max_accel * t2**2
                    s3 = (
                        self.max_accel * t2 * t3
                        + 0.5 * self.max_accel * t3**2
                        - jerk * t3**3 / 6
                    )
                    dt = t - t1 - t2 - t3
                    s = s1 + s2 + s3 + self.max_velocity * dt
                elif t < t1 + t2 + t3 + t5 + t6:
                    # 加减速段
                    s1 = jerk * t1**3 / 6
                    s2 = 0.5 * self.max_accel * t2**2
                    s3 = (
                        self.max_accel * t2 * t3
                        + 0.5 * self.max_accel * t3**2
                        - jerk * t3**3 / 6
                    )
                    s5 = self.max_velocity * t5
                    dt = t - t1 - t2 - t3 - t5
                    s = s1 + s2 + s3 + s5 + self.max_velocity * dt - jerk * dt**3 / 6
                else:
                    # 减减速段
                    s1 = jerk * t1**3 / 6
                    s2 = 0.5 * self.max_accel * t2**2
                    s3 = (
                        self.max_accel * t2 * t3
                        + 0.5 * self.max_accel * t3**2
                        - jerk * t3**3 / 6
                    )
                    s5 = self.max_velocity * t5
                    s6 = self.max_velocity * t6 - jerk * t6**3 / 6
                    dt = t - t1 - t2 - t3 - t5 - t6
                    term = jerk * (t7 - dt) ** 3 / 6
                    s = max_displacement - term

                # 归一化路径参数
                s_norm = s / max_displacement

                # 计算各关节位置
                point = [
                    start[j] + s_norm * displacements[j] for j in range(num_joints)
                ]
                trajectory.append(point)

        return trajectory


#####################servo 2#######################
class TrajectoryPlanner:
    def __init__(self, input_interval_ms=20, output_interval_ms=1):
        """
        :param input_interval_ms: 输入轨迹采样间隔(ms)
        :param output_interval_ms: 输出轨迹采样间隔(ms)
        """
        self.input_interval = input_interval_ms / 1000.0  # 转换为秒
        self.servo_time = output_interval_ms / 1000.0  # 伺服周期
        self.interp_ratio = int(input_interval_ms / output_interval_ms)  # 插值倍数

        # 运动参数（可根据实际调整）
        self.max_velocity = 1.0  # 最大速度 (rad/s or m/s)
        self.max_accel = 0.5  # 最大加速度 (rad/s² or m/s²)
        self.max_jerk = 2.0  # 最大加加速度 (rad/s³ or m/s³)

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


def main():
    # 连接机器人,设置速度比
    cps_8892 = socket.socket()
    IP = "192.168.1.30"  # Han's robot IP
    nRet = cps_8892.connect(
        (IP, 8892)
    )  # connect to port 8892, where only servo related commands are acceptable
    print("test1")

    cps_10003 = CPSClient()  # create object for connection to 10003
    nRet = cps_10003.HRIF_Connect(
        0, IP, 10003
    )  # connect to port 10003, where other regular commands are acceptable
    cps_10003.HRIF_SetOverride(
        0, 0, 0.5
    )  # set override, 10% etc.This command is used for the HRIF_MoveJ

    # 创建原始轨迹点,获取采集频率
    # 加载左臂轨迹数据
    left_waypoints = load_joint_trajectory("joints_data.txt", arm="left")

    if left_waypoints:
        print("左臂轨迹点(前5个):")
        for i, point in enumerate(left_waypoints[:5]):
            print(f"{i+1}: {point}")

        print(f"\n总轨迹点数: {len(left_waypoints)}")

        # 可以直接使用left_waypoints作为轨迹数据
        # 例如: robot.move_along_trajectory(left_waypoints)
    else:
        print("未能加载轨迹数据")

    # Todo 轨迹过滤跳点,并取前后两点的平均值.

    # 轨迹插值
    # 创建规划器（20ms→1ms）
    planner = TrajectoryPlanner(input_interval_ms=20, output_interval_ms=1)

    # 执行梯形速度规划方法插值
    # trajectory_data = planner.trapezoidal_interpolation(left_waypoints)
    # 执行S曲线规划方法插值
    trajectory_data = planner.s_curve_interpolation(left_waypoints)

    if trajectory_data:
        print("左臂轨迹点(前5个):")
        for i, point in enumerate(trajectory_data[:5]):
            print(f"{i+1}: {point}")

        print(f"\n总轨迹点数: {len(trajectory_data)}")

        # 可以直接使用left_waypoints作为轨迹数据
        # 例如: robot.move_along_trajectory(left_waypoints)
    else:
        print("未能加载轨迹数据")

    startTime = time.perf_counter()
    servoTime = 0.001  # parameter for servo interval, 1ms, no less than the robot controller EtherCAT bus cycle (1ms, or 4ms)
    # to set the look-ahead time. The larger the look-ahead time, the smoother the actual motion trajectory, but the phase lag will also increase accordingly.
    # lookaheadTime = 0.004 # parameter for servo lookahead time, 4ms.
    lookaheadTime = 0.2  # parameter for servo lookahead time, 4ms.
    cps_8892.send(
        f"StartServo,0,{servoTime},{lookaheadTime},;".encode()
    )  # start servoJ task
    count = 0
    # invoke "PushServoJ" intervally to drive robot move
    while True:
        if count > 6000:
            break  # 6000 points in this case.
        cps_8892.send(
            f"PushServoJ,0,{trajectory_data[count][0]},{trajectory_data[count][1]},{trajectory_data[count][2]},{trajectory_data[count][3]},{trajectory_data[count][4]},{trajectory_data[count][5]},;".encode()
        )  # send a point
        # ensure that "PushServoJ" is inovked intervally
        while True:
            currentTime = time.perf_counter()
            if (currentTime - startTime) > (servoTime * (count + 1)):
                break
            time.sleep(0.0001)
        count += 1
        # change the joints a little
        print(trajectory_data[count])
    end = time.perf_counter()
    print(end - startTime)
