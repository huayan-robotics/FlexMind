import time
import math
import socket
import queue
from typing import List, Tuple, Optional
import threading
from fleximind_robot.huayan_robot import CPScontrol
import rospy
import logging
import os
from datetime import datetime
import numpy as np


class SmoothMoveJFilter:
    """MoveJ平滑过渡筛选算法"""
    
    def __init__(self, min_path_length=2.0, max_angle_threshold=120.0, 
                 extreme_angle_threshold=175.0, curvature_threshold=0.8):
        self.min_path_length = min_path_length
        self.max_angle_threshold = math.radians(max_angle_threshold)
        self.extreme_angle_threshold = math.radians(extreme_angle_threshold)
        self.curvature_threshold = curvature_threshold
        
        self.point_history = []
        self.max_history_size = 5
        self.consecutive_rejects = 0
        self.max_consecutive_rejects = 3
        
        self.total_points_processed = 0
        self.points_filtered = 0
        
    def _safe_add_point(self, point):
        if point is not None and len(point) == 6:
            self.point_history.append(point.copy())
            if len(self.point_history) > self.max_history_size:
                self.point_history.pop(0)
    
    def calculate_path_length(self, point1: List[float], point2: List[float]) -> float:
        if len(point1) != 6 or len(point2) != 6:
            return 0.0
            
        joint_weights = [150.0, 120.0, 100.0, 30.0, 20.0, 10.0]
        
        weighted_distance = 0.0
        for i in range(6):
            angle_diff_rad = math.radians(abs(point2[i] - point1[i]))
            weighted_distance += (joint_weights[i] * angle_diff_rad) ** 2
            
        return math.sqrt(weighted_distance)
    
    def calculate_angle_between_paths(self, points: List[List[float]]) -> float:
        if len(points) < 3:
            return 0.0
            
        p1, p2, p3 = points[-3], points[-2], points[-1]
        
        vec1 = [p2[i] - p1[i] for i in range(6)]
        vec2 = [p3[i] - p2[i] for i in range(6)]
        
        mag1 = math.sqrt(sum(v*v for v in vec1))
        mag2 = math.sqrt(sum(v*v for v in vec2))
        
        if mag1 < 1e-6 or mag2 < 1e-6:
            return 0.0
            
        dot_product = sum(vec1[i] * vec2[i] for i in range(6))
        cos_angle = dot_product / (mag1 * mag2)
        cos_angle = max(-1.0, min(1.0, cos_angle))
        
        return math.acos(cos_angle)
    
    def calculate_curvature(self, points: List[List[float]]) -> float:
        if len(points) < 3:
            return 0.0
            
        p1, p2, p3 = points[-3], points[-2], points[-1]
        
        curvature = 0.0
        for i in range(6):
            diff1 = p2[i] - p1[i]
            diff2 = p3[i] - p2[i]
            accel = diff2 - diff1
            curvature += abs(accel)
            
        return curvature / 6.0
    
    def check_path_validity(self, new_point: List[float]) -> Tuple[bool, str, float]:
        self.total_points_processed += 1
        
        if len(self.point_history) < 2:
            self._safe_add_point(new_point)
            return True, "Insufficient history", 50.0
        
        historical_points = self.point_history[-2:]
        issues = []
        recommended_radius = 50.0
        
        if len(historical_points) >= 2:
            last_path_length = self.calculate_path_length(historical_points[-2], historical_points[-1])
            new_path_length = self.calculate_path_length(historical_points[-1], new_point)
            
            if new_path_length < self.min_path_length:
                issues.append(f"short_path_{new_path_length:.1f}mm")
                recommended_radius = max(recommended_radius, 30.0)
        
        if len(historical_points) >= 2:
            test_points = historical_points + [new_point]
            path_angle = self.calculate_angle_between_paths(test_points)
            
            if path_angle > self.extreme_angle_threshold:
                issues.append("extreme_angle")
                if self.consecutive_rejects >= self.max_consecutive_rejects:
                    issues.remove("extreme_angle")
                    recommended_radius = 200.0
                else:
                    self.consecutive_rejects += 1
                    self.points_filtered += 1
                    return False, f"Extreme angle: {math.degrees(path_angle):.1f}°", 0.0
            elif path_angle > self.max_angle_threshold:
                issues.append("large_angle")
                angle_ratio = (path_angle - self.max_angle_threshold) / \
                            (self.extreme_angle_threshold - self.max_angle_threshold)
                recommended_radius = 50.0 + angle_ratio * 100.0
        
        curvature = self.calculate_curvature(historical_points + [new_point])
        if curvature > self.curvature_threshold:
            issues.append("high_curvature")
            recommended_radius = max(recommended_radius, 80.0)
        
        self._safe_add_point(new_point)
        self.consecutive_rejects = 0
        
        if issues:
            return True, f"Adjusted for: {', '.join(issues)}", recommended_radius
        else:
            return True, "Path valid", recommended_radius
    
    def get_filter_stats(self):
        return {
            'total_points_processed': self.total_points_processed,
            'points_filtered': self.points_filtered,
            'filter_rate': self.points_filtered / max(1, self.total_points_processed)
        }


class RealTimeMoveController:
    def __init__(
        self,
        ip,
        box_id,
        name,
        port=10003,
        movej_frequency=10,
        log_dir="/home/robot/fleximind-ros1/servo_logs",
        min_path_length=2.0,
        max_angle_threshold=120.0,
        extreme_angle_threshold=175.0,
        curvature_threshold=0.8,
        angle_diff_threshold=1.0
    ):
        self.ip = ip
        self.box_id = box_id
        self.name = name
        self.port = port
        
        # 🚨 关键修复1: 使用无界队列，但添加队列清空机制
        self.input_queue = queue.Queue(maxsize=3)
        self.running = False
        self.sending_thread = None
        self.last_input_point = None
        self.arm = None
        self.connected = False
        self.lock = threading.Lock()
        self.is_sending = False
        
        self.movej_interval = 1.0 / movej_frequency
        self.is_sdk = True
        self.angle_diff_threshold = angle_diff_threshold
        
        # 初始化平滑过滤器
        self.smooth_filter = SmoothMoveJFilter(
            min_path_length=min_path_length,
            max_angle_threshold=max_angle_threshold,
            extreme_angle_threshold=extreme_angle_threshold,
            curvature_threshold=curvature_threshold
        )
        
        # 动态过渡半径管理
        self.current_radius = 50.0
        self.radius_adjustment_factor = 1.0
        
        # 性能监控
        self.performance_stats = {
            'success_count': 0,
            'error_count': 0,
            'last_status_log_time': time.time(),
            'cycle_count': 0
        }
        
        # 连接状态监控
        self.last_connection_check = 0
        self.connection_check_interval = 5.0
        
        # 🚨 关键修复2: 使用更简单的停止控制，避免复杂的事件机制
        self._stop_flag = threading.Event()
        self._stop_timeout = 3.0
        
        # 初始化日志系统
        self._setup_logging(log_dir)
        self.logger.info(f"{self.name} MoveJ控制器初始化完成 - IP: {ip}, BoxID: {box_id}")

    def start(self):
        """启动控制器 - 保持原有接口不变"""
        self.logger.info(f"{self.name} 开始启动控制器")
        
        self.logger.info(f"{self.name} 清除之前的点位")
        self._clear_queue_safely()
        
        if not self.connected:
            try:
                self.logger.info(f"{self.name} 创建CPScontrol实例")
                self.arm = CPScontrol(
                    name=self.name,
                    box_id=self.box_id,
                    ip=self.ip,
                    port=self.port,
                )
                
                self.logger.info(f"{self.name} 连接机器人控制器")
                if not self.arm.connect():
                    self.logger.error(f"{self.name} 连接机器人控制器失败")
                    self.connected = False
                    return False
                
                self.connected = True
                self.logger.info(f"{self.name} 连接成功: {self.ip}:{self.port}")

            except Exception as e:
                self.logger.error(f"{self.name} 连接过程发生异常: {e}", exc_info=True)
                self.connected = False
                if self.arm:
                    try:
                        self.arm.disconnect()
                    except Exception as disconn_error:
                        self.logger.error(f"{self.name} 断开连接异常: {disconn_error}")
                self.arm = None
                return False
                
        if not self.running:
            self.running = True
            self._stop_flag.clear()
            self.sending_thread = threading.Thread(
                target=self._sending_worker, 
                name=f"MoveJSender-{self.name}",
                daemon=True
            )
            self.sending_thread.start()
            self.logger.info(f"{self.name} 发送线程启动成功")
        
        try:    
            self.arm.set_override(boxID=self.box_id,dOverride=1)
            self.logger.info(f"{self.name} 速度调整为100%")
        except Exception as e:
            self.logger.error(f"{self.name} 速度设置异常: {e}")
            
        return True

    def stop(self):
        """安全停止控制器 - 修复死锁和队列清空问题"""
        self.logger.info(f"{self.name} 开始安全停止控制器")
        
        if not self.running:
            self.logger.info(f"{self.name} 控制器已处于停止状态")
            return True
        
        # 🚨 关键修复3: 简化停止逻辑，避免死锁
        self.running = False
        self.is_sending = False
        self._stop_flag.set()
        
        # 🚨 关键修复4: 使用非阻塞方式唤醒线程
        self._wakeup_thread_safely()
        
        # 🚨 关键修复5: 等待线程退出，但不在等待期间持有任何锁
        if self.sending_thread and self.sending_thread.is_alive():
            self.logger.info(f"{self.name} 等待发送线程退出")
            
            # 使用分段等待，避免长时间阻塞
            wait_start = time.time()
            while time.time() - wait_start < self._stop_timeout:
                if not self.sending_thread.is_alive():
                    break
                time.sleep(0.1)  # 短暂等待，定期检查
                
                # 定期尝试唤醒线程
                if int((time.time() - wait_start) * 10) % 3 == 0:  # 每0.3秒唤醒一次
                    self._wakeup_thread_safely()
            
            if self.sending_thread.is_alive():
                self.logger.warning(f"{self.name} 发送线程未在超时时间内退出")
            else:
                self.logger.info(f"{self.name} 发送线程已退出")
        
        # 🚨 关键修复6: 线程退出后安全清空队列
        self._clear_queue_safely()
        
        # 重置状态
        self.last_input_point = None
        self.sending_thread = None
        self._stop_flag.clear()
        
        self.logger.info(f"{self.name} 控制器安全停止完成")
        
        self.arm.set_override(boxID=self.box_id,dOverride=0.15)
        self.logger.info(f"{self.name} 速度恢复为15%")
        return True

    def _wakeup_thread_safely(self):
        """安全唤醒线程 - 完全不阻塞"""
        try:
            # 尝试放入唤醒标记，但使用非阻塞方式
            for _ in range(2):  # 尝试2次
                try:
                    self.input_queue.put_nowait("WAKEUP_FOR_STOP")
                    break  # 成功放入一个就退出
                except:
                    break  # 队列满或其他异常时直接退出
        except:
            pass  # 忽略所有异常

    def _clear_queue_safely(self):
        """安全清空队列 - 在线程退出后执行"""
        try:
            current_size = self.input_queue.qsize()
            cleared_count = 0
            # 使用非阻塞方式清空队列
            while cleared_count<=current_size:
                try:
                    self.input_queue.get_nowait()
                    cleared_count += 1
                except queue.Empty:
                    break
            
            if cleared_count > 0:
                self.logger.info(f"{self.name} 安全清空队列，移除点数: {cleared_count}")
            else:
                self.logger.debug(f"{self.name} 队列已为空")
                
        except Exception as e:
            self.logger.error(f"{self.name} 清空队列时异常: {e}")

    def set_sending(self, sending: bool):
        """设置发送状态 - 保持原有接口不变"""
        with self.lock:
            if sending:
                self.logger.info(f"{self.name} MoveJ控制开启")
                print(f"{self.name} MoveJ控制已开启")
            else:
                self.logger.info(f"{self.name} MoveJ控制关闭")
                print(f"{self.name} MoveJ控制已关闭")
                # 🚨 关键修复7: 不在set_sending中清空队列，避免锁竞争
                self.last_input_point = None
            self.is_sending = sending

    def add_point(self, point):
        """添加点位到队列 - 保持原有接口不变"""
        try:
            if point is None:
                self.logger.warning(f"{self.name} 尝试添加空点位，已忽略")
                return
                
            if len(point) != 6:
                self.logger.error(f"{self.name} 点位格式错误，期望6个关节值，实际得到{len(point)}个: {point}")
                return
            
            # 1. 角度差过滤
            if self.last_input_point is not None:
                angle_diffs = [abs(point[i] - self.last_input_point[i]) for i in range(6)]
                max_angle_diff = max(angle_diffs)
                
                if max_angle_diff < self.angle_diff_threshold:
                    self.logger.debug(f"{self.name} 角度差过小({max_angle_diff:.3f}度)，已跳过")
                    return
            
            # 2. 平滑过渡检查
            is_valid, reason, recommended_radius = self.smooth_filter.check_path_validity(point)
            
            if not is_valid:
                self.logger.warning(f"{self.name} 点位被过滤 - {reason}")
                return
            
            # 3. 更新动态过渡半径
            self._update_radius(recommended_radius, reason)
            
            # 4. 添加到队列
            self.last_input_point = point
            
            try:
                self.input_queue.put_nowait(point)
                current_size = self.input_queue.qsize()
                
                if "Large angle" in reason or "Adjusted for" in reason:
                    self.logger.info(f"{self.name} {reason}, 使用半径: {self.current_radius:.1f}mm")
                elif current_size % 20 == 0:
                    self.logger.debug(f"{self.name} 点位添加成功，队列大小: {current_size}")
                    
            except Exception as e:
                self.logger.error(f"{self.name} 添加点位到队列异常: {e}")
                    
        except Exception as e:
            self.logger.error(f"{self.name} 添加点位异常: {e}", exc_info=True)

    def _update_radius(self, recommended_radius: float, reason: str):
        """更新动态过渡半径"""
        smoothing_factor = 0.3
        self.current_radius = (smoothing_factor * recommended_radius + 
                              (1 - smoothing_factor) * self.current_radius)
        self.current_radius = max(10.0, min(200.0, self.current_radius))

    def _check_connection(self):
        """检查连接状态"""
        current_time = time.time()
        if current_time - self.last_connection_check > self.connection_check_interval:
            self.last_connection_check = current_time
            
            if not self.connected and self.arm:
                try:
                    self.logger.info(f"{self.name} 尝试重新连接机器人")
                    if self.arm.connect():
                        self.connected = True
                        self.logger.info(f"{self.name} 重新连接成功")
                    else:
                        self.logger.warning(f"{self.name} 重新连接失败")
                except Exception as e:
                    self.logger.error(f"{self.name} 重新连接异常: {e}")

    def _setup_logging(self, log_dir):
        """设置日志系统"""
        try:
            if not os.path.exists(log_dir):
                os.makedirs(log_dir)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_filename = f"{self.ip}_movej_{timestamp}.log"
            log_path = os.path.join(log_dir, log_filename)

            self.logger = logging.getLogger(f"{self.name}_{timestamp}")
            self.logger.setLevel(logging.DEBUG)
            
            if not self.logger.handlers:
                file_handler = logging.FileHandler(log_path, encoding="utf-8")
                file_handler.setLevel(logging.DEBUG)
                
                console_handler = logging.StreamHandler()
                console_handler.setLevel(logging.INFO)

                formatter = logging.Formatter(
                    "%(asctime)s.%(msecs)03d - %(name)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s",
                    datefmt="%Y-%m-%d %H:%M:%S"
                )
                file_handler.setFormatter(formatter)
                console_handler.setFormatter(formatter)

                self.logger.addHandler(file_handler)
                self.logger.addHandler(console_handler)

            self.logger.info(f"MoveJ控制器日志系统初始化完成 - 日志文件: {log_path}")
            
        except Exception as e:
            print(f"日志系统初始化失败: {e}")
            self.logger = logging.getLogger(f"{self.name}_fallback")
            if not self.logger.handlers:
                handler = logging.StreamHandler()
                formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
                handler.setFormatter(formatter)
                self.logger.addHandler(handler)
                self.logger.setLevel(logging.INFO)

    def _sending_worker(self):
        """发送线程 - 完全非阻塞版本"""
        self.logger.info(f"{self.name} MoveJ发送线程启动")
        
        last_send_time = 0
        consecutive_failures = 0
        max_consecutive_failures = 5
        
        try:
            while self.running:
                current_time = time.time()
                
                # 🎯 关键修改: 使用完全非阻塞方式获取点位
                point = None
                try:
                    point = self.input_queue.get_nowait()  # 非阻塞获取
                    
                    # 检查停止信号
                    if point == "WAKEUP_FOR_STOP" or not self.running:
                        self.logger.debug(f"{self.name} 收到停止信号，退出线程")
                        break
                        
                except queue.Empty:
                    # 队列为空，短暂休眠避免CPU占用过高
                    time.sleep(0.01)  # 10ms休眠
                    continue
                
                # 频率控制 - 使用非阻塞方式检查时间间隔
                time_since_last_send = current_time - last_send_time
                if time_since_last_send < self.movej_interval:
                    # 分段休眠，定期检查停止信号
                    sleep_time = self.movej_interval - time_since_last_send
                    sleep_chunks = max(1, int(sleep_time / 0.05))  # 每50ms检查一次
                    
                    for i in range(sleep_chunks):
                        if not self.running:
                            break
                        chunk_sleep = min(0.05, sleep_time - i * 0.05)
                        if chunk_sleep > 0:
                            time.sleep(chunk_sleep)
                    continue
                
                # 在处理点位前再次检查运行状态
                if not self.running:
                    break
                
                # 处理有效点位
                if (point is not None and 
                    isinstance(point, (list, tuple)) and 
                    len(point) == 6 and 
                    self.is_sending and 
                    self.arm and 
                    self.connected):
                    
                    try:
                        success = self.arm.move_j(
                            boxID=self.box_id,
                            rbtID=0,
                            dJ1_dJ6=point,
                            dX_dRz=None,
                            sTcpName="TCP",
                            sUcsName="Base",
                            dVelocity=180.0,
                            dAcc=360.0,
                            dRadius=self.current_radius,
                            nIsUseJoint=1,
                            nIsSeek=0,
                            nIOBit=0,
                            nIOState=0,
                            strCmdID="0"
                        )
                        
                        if success:
                            self.performance_stats['success_count'] += 1
                            last_send_time = current_time
                            consecutive_failures = 0
                        else:
                            self.performance_stats['error_count'] += 1
                            consecutive_failures += 1
                            self.logger.error(f"{self.name} MoveJ发送失败")
                            self.connected = False
                            
                    except Exception as e:
                        self.performance_stats['error_count'] += 1
                        consecutive_failures += 1
                        self.logger.error(f"{self.name} MoveJ发送异常: {e}")
                        self.connected = False
                        
                        if consecutive_failures >= max_consecutive_failures:
                            self.logger.error(f"{self.name} 连续失败超过限制，停止发送")
                            break
                
                # 定期检查连接状态 - 使用非阻塞时间检查
                if current_time - self.last_connection_check > self.connection_check_interval:
                    self._check_connection()
                    
        except Exception as e:
            self.logger.error(f"{self.name} 发送线程发生未捕获异常: {e}", exc_info=True)
        finally:
            # 🎯 关键修改: 使用非阻塞方式清空线程可能持有的队列项目
            try:
                cleared_count = 0
                while True:
                    try:
                        self.input_queue.get_nowait()
                        cleared_count += 1
                    except queue.Empty:
                        break
                if cleared_count > 0:
                    self.logger.debug(f"{self.name} 线程退出前清空 {cleared_count} 个点位")
            except:
                pass
                
            self.logger.info(f"{self.name} 发送线程安全退出")

    def get_status(self):
        """获取控制器状态信息"""
        filter_stats = self.smooth_filter.get_filter_stats()
        total_attempts = max(1, self.performance_stats['success_count'] + self.performance_stats['error_count'])
        success_rate = self.performance_stats['success_count'] / total_attempts
        
        thread_alive = self.sending_thread.is_alive() if self.sending_thread else False
        
        return {
            'connected': self.connected,
            'sending': self.is_sending,
            'running': self.running,
            'queue_size': self.input_queue.qsize(),
            'current_radius': self.current_radius,
            'success_rate': success_rate,
            'filter_stats': filter_stats,
            'thread_alive': thread_alive
        }


# 保持原有类结构完全兼容
class TrajectoryPlanner:
    def __init__(self, interp_ratio=3, max_velocity=1.0, max_accel=0.5, max_jerk=2.0):
        self.interp_ratio = interp_ratio
        self.max_velocity = max_velocity
        self.max_accel = max_accel
        self.max_jerk = max_jerk

    def s_curve_interpolation(self, waypoints: List[List[float]]) -> List[List[float]]:
        return waypoints

    def trapezoidal_interpolation(self, waypoints: List[List[float]]) -> List[List[float]]:
        return waypoints