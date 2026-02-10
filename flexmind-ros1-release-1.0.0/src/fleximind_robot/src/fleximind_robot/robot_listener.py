import json
import socket
import threading
import time
from datetime import datetime

import rospy


class RobotMonitor:
    def __init__(self, host, port=10006):
        self.host = host
        self.port = port
        self.socket = None
        self.buffer = b""
        self._running = False
        self._thread = None
        self.data_lock = threading.Lock()  # 数据访问锁
        self.connection_lock = threading.Lock()  # 连接操作锁

        # 网络参数配置
        self.socket_timeout = 5.0
        self.reconnect_interval = 3.0
        self.max_buffer_size = 10 * 1024 * 1024  # 10MB

        # 状态数据初始化
        self.core_data = {
            "robotState": 0,
            "robotEnabled": False,
            "robotMoving": False,
            "Error_AxisID": 0,
            "Error_Code": 0,
            "joint_positions": [0.0] * 6,
            "BoxCI": [0] * 8,
            "BoxCO": [0] * 8,
            "BoxDI": [0] * 8,
            "BoxDO": [0] * 8,
            "EndDI": [0] * 4,
            "EndDO": [0] * 4,
            "last_update": datetime.now().isoformat(),
        }

    def connect(self):
        """建立socket连接（线程安全）"""
        with self.connection_lock:
            if self.socket:
                self.socket.close()

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.socket_timeout)

            try:
                self.socket.connect((self.host, self.port))
                print(f"成功连接到 {self.host}:{self.port}")
                return True
            except Exception as e:
                print(f"连接失败: {str(e)}")
                self.socket = None
                return False

    def start_monitoring(self):
        """启动监控线程"""
        if self._running:
            print("监控已在运行中")
            return False

        if not self.connect():
            return False

        self._running = True
        self._thread = threading.Thread(
            target=self._monitor_loop, daemon=True, name="RobotMonitorThread"
        )
        self._thread.start()
        print("监控线程已启动")
        return True

    def stop_monitoring(self):
        """停止监控线程"""
        if not self._running:
            return

        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self.close()
        print("监控线程已停止")

    def _monitor_loop(self):
        """监控主循环"""
        last_data_time = time.time()

        while self._running:
            try:
                # 检查连接状态
                if not self.socket:
                    if not self._reconnect():
                        time.sleep(self.reconnect_interval)
                        continue

                # 接收数据（非阻塞）
                try:
                    chunk = self.socket.recv(4096)
                    if not chunk:
                        print("连接已关闭")
                        self._reconnect()
                        continue

                    last_data_time = time.time()
                    self._process_incoming_data(chunk)

                except socket.timeout:
                    # 检查数据超时
                    if time.time() - last_data_time > 30:
                        print("数据接收超时，尝试重连")
                        self._reconnect()
                    continue

                except ConnectionResetError:
                    print("连接被重置")
                    self._reconnect()
                    continue

            except Exception as e:
                print(f"监控循环异常: {str(e)}")
                time.sleep(1)

    def _reconnect(self):
        """尝试重新连接"""
        with self.connection_lock:
            print("尝试重新连接...")
            if self.connect():
                self.buffer = b""  # 清空缓冲区
                return True
            return False

    def _process_incoming_data(self, chunk):
        """处理接收到的数据块"""
        # 缓冲区检查
        if len(self.buffer) + len(chunk) > self.max_buffer_size:
            print("警告：缓冲区溢出，清空缓冲")
            self.buffer = b""

        self.buffer += chunk

        # 尝试解析完整消息
        while True:
            try:
                start_idx = self.buffer.find(b"{")
                if start_idx == -1:
                    break  # 没有完整消息

                # 尝试解析JSON
                data, end_idx = self._parse_json(self.buffer[start_idx:])
                if not data:
                    break  # 需要更多数据

                # 处理有效数据
                with self.data_lock:
                    self._update_core_data(data)

                # 移动缓冲区
                self.buffer = self.buffer[start_idx + end_idx :]

            except Exception as e:
                print(f"数据处理异常: {str(e)}")
                break

    def _parse_json(self, raw_data):
        """安全解析JSON数据"""
        try:
            # 尝试直接解析
            data = json.loads(raw_data.decode("utf-8"))
            return data, len(raw_data)
        except UnicodeDecodeError:
            # 处理二进制前缀
            start_idx = raw_data.find(b"{")
            if start_idx > 0:
                try:
                    data = json.loads(raw_data[start_idx:].decode("utf-8"))
                    return data, len(raw_data)
                except:
                    pass
        except json.JSONDecodeError:
            # 尝试找到完整消息
            end_idx = raw_data.rfind(b"}")
            if end_idx > 0:
                try:
                    data = json.loads(raw_data[: end_idx + 1].decode("utf-8"))
                    return data, end_idx + 1
                except:
                    pass
        return None, 0

    def _update_core_data(self, data):
        """更新核心数据（线程安全）"""
        try:
            # 提取基础状态
            state_info = data.get("StateAndError", {})
            pos_info = data.get("PosAndVel", {})
            io_info = data.get("ElectricBoxIO", {})
            end_io = data.get("EndIO", {})

            # 更新数据字典
            new_data = {
                "robotState": state_info.get("robotState", 0),
                "robotEnabled": bool(state_info.get("robotEnabled", 0)),
                "robotMoving": bool(state_info.get("robotMoving", 0)),
                "Error_AxisID": state_info.get("Error_AxisID", 0),
                "Error_Code": state_info.get("Error_Code", 0),
                "joint_positions": [
                    float(x) for x in pos_info.get("Actual_Position", [])[:6]
                ],
                "eef_positions":[
                    float(x) for x in pos_info.get("Actual_Position", [])[-6:]
                ],
                "BoxCI": io_info.get("BoxCI", [0] * 8)[:8],
                "BoxCO": io_info.get("BoxCO", [0] * 8)[:8],
                "BoxDI": io_info.get("BoxDI", [0] * 8)[:8],
                "BoxDO": io_info.get("BoxDO", [0] * 8)[:8],
                "EndDI": end_io.get("EndDI", [0] * 4)[:4],
                "EndDO": end_io.get("EndDO", [0] * 4)[:4],
                "last_update": datetime.now().isoformat(),
            }
            # print(new_data)
            # 原子性更新
            self.core_data.update(new_data)

        except Exception as e:
            print(f"数据更新异常: {str(e)}")

    def get_core_data(self):
        """获取当前核心数据（线程安全）"""
        with self.data_lock:
            return self.core_data.copy()  # 返回副本避免竞态

    def close(self):
        """安全关闭连接"""
        with self.connection_lock:
            if self.socket:
                try:
                    self.socket.shutdown(socket.SHUT_RDWR)
                except:
                    pass
                finally:
                    self.socket.close()
                    self.socket = None
            self._running = False

    def __del__(self):
        """析构函数确保资源释放"""
        self.stop_monitoring()
