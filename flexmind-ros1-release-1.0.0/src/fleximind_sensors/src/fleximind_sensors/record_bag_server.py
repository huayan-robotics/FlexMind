#!/usr/bin/env python
import os
import queue
import threading
import time
from threading import Lock

import actionlib
import rosbag
import rospy
from mcap_ros1.writer import Writer as MCAPWriter
from std_msgs.msg import Bool, String

from fleximind_bringup.interface import RecordServer
from fleximind_sensors.base import DEFAULT_CHUNK_THRESHOLD, StorageFormat
from fleximind_sensors.msg import (
    RecordBagAction,
    RecordBagFeedback,
    RecordBagGoal,
    RecordBagResult,
)


class RecordBagServer:
    def __init__(self, name):
        # 初始化Action服务器
        self.server = actionlib.SimpleActionServer(
            name,
            RecordBagAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo(f"记录服务器 [{name}] 已启动")

        # 参数
        self.record_frequency = 10
        self.storage_format = rospy.get_param(
            "/record_bag/storage_format", StorageFormat.BAG.value
        )
        rospy.loginfo(f"当前存储格式: {self.storage_format.upper()}")

        # 记录状态变量
        self.record_lock = Lock()
        self.multi_thread_lock = Lock()
        self._is_recording = False
        self.cancel_requested = False
        self.subscribers = []  # 保存所有订阅器
        self.writer = None  # 通用写入器实例
        self.mcap_file = None  # MCAP专用文件对象

        self.msg_queue = queue.Queue()
        self.process_thread = None
        self._start_processing_thread()

        # 默认存储目录
        self.output_dir = rospy.get_param("/record_bag/output_dir", "~/rosbags")
        os.makedirs(os.path.expanduser(self.output_dir), exist_ok=True)

        self.status_pub = rospy.Publisher(
            RecordServer.RECORDING_STATUS_DETAIL.value, String, queue_size=10
        )
        self.recording_status_pub = rospy.Publisher(
            RecordServer.RECORDING_STATE.value, Bool, queue_size=10
        )

        self._publish_status("initialized")

    def _start_processing_thread(self):
        """启动顺序处理线程"""
        self.process_thread = threading.Thread(
            target=self._process_messages, daemon=True
        )
        self.process_thread.start()
        rospy.loginfo("消息顺序处理线程已启动")

    def _process_messages(self):
        """顺序处理消息的主循环"""
        while not rospy.is_shutdown():
            try:
                # 从队列获取消息（阻塞式）
                topic, msg, timestamp = self.msg_queue.get(timeout=1.0)

                # 处理消息
                self._write_message_safely(topic, msg, timestamp)

                # 标记任务完成
                self.msg_queue.task_done()

            except queue.Empty:
                # 超时，继续循环
                continue
            except Exception as e:
                rospy.logwarn(f"处理消息时出错: {str(e)}")

    def _msg_callback(self, msg, topic):
        """消息回调 - 仅将消息放入队列"""
        if not self.is_recording or not self.writer:
            return

        # 获取时间戳
        timestamp = msg.header.stamp if hasattr(msg, "header") else rospy.Time.now()

        try:
            # 非阻塞方式放入队列
            self.msg_queue.put((topic, msg, timestamp), block=False)
        except queue.Full:
            # 队列已满，丢弃最旧的消息（可选策略）
            try:
                self.msg_queue.get_nowait()  # 丢弃一个旧消息
                self.msg_queue.put((topic, msg, timestamp), block=False)
                rospy.logwarn(f"队列已满，丢弃一条旧消息，加入新消息: {topic}")
            except queue.Empty:
                # 如果队列突然变空，重新尝试
                self.msg_queue.put((topic, msg, timestamp), block=False)

    def _write_message_safely(self, topic, msg, timestamp):
        """安全写入消息（在单线程中顺序执行）"""
        try:
            if self.storage_format == StorageFormat.BAG.value:
                self.writer.write(topic, msg, timestamp)
            elif self.storage_format == StorageFormat.MCAP.value:
                self.writer.write_message(topic, msg, timestamp.to_nsec())

        except Exception as e:
            rospy.logwarn(f"写入 {topic} 失败: {str(e)}")

    @property
    def is_recording(self):
        return self._is_recording

    @is_recording.setter
    def is_recording(self, _is_recording):
        with self.record_lock:
            self._is_recording = _is_recording

    def execute_cb(self, goal):
        """执行回调函数"""
        self.is_recording = True
        self._publish_status("started")

        # 创建唯一文件名（根据格式添加扩展名）
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        ext = self.storage_format
        bag_path = os.path.join(
            os.path.expanduser(self.output_dir), f"recording_{timestamp}.{ext}"
        )

        # 初始化状态标志
        cancelled = False
        disk_full = False
        time_expired = False

        try:
            self._setup_recording(goal.topics, goal.topics_type, bag_path)
            start_time = time.time()
            feedback = RecordBagFeedback()

            # 主循环实时检查取消请求
            rate = rospy.Rate(self.record_frequency)
            while True:
                # 检查预占请求（取消信号）
                if self.server.is_preempt_requested() or self.cancel_requested:
                    rospy.loginfo("收到取消请求，终止录制")
                    self.server.set_preempted()
                    self.cancel_requested = False
                    cancelled = True
                    break

                # 原有检查逻辑
                if not self._check_disk_space(min_space_gb=1):
                    rospy.logwarn("磁盘空间不足，停止录制")
                    disk_full = True
                    break

                elapsed = time.time() - start_time
                if goal.duration > 0 and elapsed >= goal.duration:
                    time_expired = True
                    break

                # 发布反馈
                if goal.duration > 0:
                    feedback.progress = min(100.0, (elapsed / goal.duration) * 100)
                feedback.elapsed_time = elapsed
                self.server.publish_feedback(feedback)
                rate.sleep()

        except Exception as e:
            rospy.logerr(f"记录出错: {str(e)}")
        finally:
            self._stop_recording()
            self.is_recording = False
            self._publish_status("stopped")

            # 根据终止原因设置最终状态
            result = RecordBagResult()
            result.bag_path = bag_path

            if cancelled:
                result.success = False
                self.server.set_preempted(result, "记录被取消")
            elif disk_full:
                result.success = False
                self.server.set_aborted(result, "磁盘空间不足")
            elif time_expired:
                result.success = True
                self.server.set_succeeded(result, "记录完成")
            else:
                result.success = False
                self.server.set_aborted(result, "录制异常终止")

    def _setup_recording(self, topics, topics_type, bag_path):
        """初始化录制 - 支持rosbag和MCAP格式"""
        # ===== 通用文件检查逻辑 =====
        if os.path.exists(bag_path):
            rospy.logwarn(f"文件已存在: {bag_path}, 将被覆盖")
            try:
                os.remove(bag_path)
            except Exception as e:
                rospy.logerr(f"删除文件失败: {e}")
                raise

        # ===== 根据格式初始化写入器 =====
        if self.storage_format == StorageFormat.BAG.value:
            # 原生rosbag模式
            try:
                self.writer = rosbag.Bag(
                    bag_path,
                    "w",
                    chunk_threshold=DEFAULT_CHUNK_THRESHOLD,
                )
                rospy.loginfo(f"创建ROS Bag文件: {bag_path}")
            except rosbag.ROSBagException as e:
                rospy.logerr(f"打开ROS Bag失败: {e}")
                raise

        elif self.storage_format == StorageFormat.MCAP.value:
            try:
                # 使用 mcap-ros1-support 的 Writer
                self.mcap_file = open(bag_path, "wb")
                self.writer = MCAPWriter(self.mcap_file)
                rospy.loginfo(f"创建MCAP文件: {bag_path}")

            except ImportError:
                rospy.logerr(
                    "未安装mcap-ros1-support! 请执行: pip install mcap-ros1-support"
                )
                raise
            except Exception as e:
                rospy.logerr(f"打开MCAP文件失败: {e}")
                raise
        else:
            rospy.logerr(f"不支持的存储格式: {self.storage_format}")
            raise ValueError("Invalid storage format")

        # ===== 消息类型处理 =====
        self.msg_classes = {}
        if len(topics) != len(topics_type):
            rospy.logerr(
                f"参数错误: topics数量({len(topics)}) != 类型数量({len(topics_type)})"
            )
            raise ValueError("Topic/type count mismatch")

        # ===== 创建订阅器 =====
        self.subscribers = []

        for i, (topic, topic_type) in enumerate(zip(topics, topics_type)):
            try:
                # 动态导入消息类型
                module_name, class_name = topic_type.split("/")
                exec(f"from {module_name}.msg import {class_name}")
                msg_class = locals()[class_name]
                self.msg_classes[topic] = msg_class

                # 创建订阅器
                sub = rospy.Subscriber(
                    topic, msg_class, self._msg_callback, callback_args=topic
                )
                self.subscribers.append(sub)
                rospy.loginfo(f"订阅话题 {i+1}/{len(topics)}: {topic} ({topic_type})")

            except ImportError:
                rospy.logwarn(f"动态导入失败，尝试监听话题: {topic}")
                try:
                    # 回退方案：通过实际消息获取类型
                    msg_sample = rospy.wait_for_message(topic, rospy.AnyMsg, timeout=2)
                    msg_class = msg_sample.__class__
                    self.msg_classes[topic] = msg_class
                except Exception as e:
                    rospy.logerr(f"监听话题 {topic} 失败: {str(e)}")
                    continue
            except Exception as e:
                rospy.logerr(f"初始化话题 {topic} 失败: {str(e)}")
                continue

        rospy.loginfo(f"开始记录 {len(topics)} 个话题到 {bag_path}")

    # def _msg_callback(self, msg, topic):
    #     """通用消息回调 - 支持两种存储格式"""
    #     with self.multi_thread_lock:
    #         if not self.is_recording or not self.writer:
    #             return

    #         try:
    #             # 获取时间戳（优先使用消息头）
    #             timestamp = msg.header.stamp if hasattr(msg, "header") else rospy.Time.now()

    #             if self.storage_format == StorageFormat.BAG.value:
    #                 # 原生rosbag写入
    #                 self.writer.write(topic, msg, timestamp)

    #             elif self.storage_format == StorageFormat.MCAP.value:
    #                 # MCAP写入：直接使用write_message方法
    #                 self.writer.write_message(topic, msg, timestamp.to_nsec())

    #         except Exception as e:
    #             rospy.logwarn(f"写入 {topic} 失败: {str(e)}")

    def _stop_recording(self):
        """停止记录并关闭资源"""
        if hasattr(self, "msg_queue") and self.msg_queue:
            try:
                self.msg_queue.join()  # 等待所有任务完成
                rospy.loginfo("所有消息已处理完成")
            except Exception as e:
                rospy.logwarn(f"等待消息队列完成时出错: {e}")

        # 取消所有订阅
        for sub in self.subscribers:
            try:
                sub.unregister()
            except Exception as e:
                rospy.logwarn(f"取消订阅失败: {str(e)}")
        self.subscribers = []

        # 关闭写入器
        if self.writer:
            try:
                if self.storage_format == StorageFormat.BAG.value:
                    self.writer.close()  # 确保正确关闭
                elif self.storage_format == StorageFormat.MCAP.value:
                    self.writer.finish()  # MCAP格式完成写入
                    if self.mcap_file:
                        self.mcap_file.close()

                rospy.loginfo(f"成功关闭 {self.storage_format.upper()} 写入器")
            except Exception as e:
                rospy.logerr(f"关闭写入器失败: {str(e)}")
            finally:
                self.writer = None
                self.mcap_file = None

    def _check_disk_space(self, min_space_gb=1):
        """检查磁盘空间是否充足"""
        try:
            stat = os.statvfs(self.output_dir)
            free_space = stat.f_bavail * stat.f_frsize
            return free_space >= min_space_gb * 1e9
        except Exception as e:
            rospy.logerr(f"磁盘检查失败: {str(e)}")
            return False

    def cancel_recording(self):
        """外部调用取消记录"""
        if self.is_recording:
            self.cancel_requested = True
            return True
        return False

    # def _publish_status(self, status):
    #     """发布录制状态"""
    #     self.status_pub.publish(status)
    #     self.recording_status_pub.publish(Bool(data=(status == "started")))

    def _publish_status(self, status):
        """发布录制状态"""
        try:
            status_msg = String()
            status_msg.data = status
            self.status_pub.publish(status_msg)

            recording_msg = Bool()
            recording_msg.data = status == "started"
            self.recording_status_pub.publish(recording_msg)

            rospy.loginfo(f"发布录制状态: {status}, 录制中: {recording_msg.data}")
        except Exception as e:
            rospy.logwarn(f"发布状态失败: {e}")
