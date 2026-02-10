#!/usr/bin/env python
from threading import Lock

import actionlib
import rospy
from std_msgs.msg import Bool, String

from fleximind_bringup import DEFAULT_LOG_PERIOD_S
from fleximind_bringup.interface import RecordServer
from fleximind_sensors.msg import (
    RecordBagAction,
    RecordBagFeedback,
    RecordBagGoal,
    RecordBagResult,
)


class RecordBagClient:
    def __init__(self, server_name="record_bag"):
        self.client = actionlib.SimpleActionClient(server_name, RecordBagAction)
        self._is_recording = False  # 唯一状态跟踪变量
        self.recording_lock = Lock()
        rospy.Subscriber(RecordServer.RECORDING_STATE.value, Bool, self._status_callback)
        rospy.Subscriber(
            RecordServer.RECORDING_STATUS_DETAIL.value, String, self._detailed_status_callback
        )

    @property
    def is_recording(self):
        with self.recording_lock:
            return self._is_recording

    @is_recording.setter
    def is_recording(self, state: bool):
        with self.recording_lock:
            self._is_recording = state

    def start_recording(self, duration=0):
        """开始当前录制任务"""
        goal = RecordBagGoal()
        goal.topics = rospy.get_param(
            "/record_bag/topics", ["/camera/left_hand/color/image_raw"]
        )
        goal.topics_type = rospy.get_param(
            "/record_bag/topics_type", ["sensor_msgs/Image"]
        )
        goal.duration = rospy.get_param("/record_bag/duration", 10.0)

        self.client.send_goal(
            goal,
            done_cb=self._done_callback,
            active_cb=self._active_callback,
            feedback_cb=self._feedback_callback,
        )
        rospy.loginfo(f"已发送录制请求: {goal.topics}")

    def cancel_recording(self):
        if self.is_recording:
            rospy.loginfo("发送取消请求...")
            self.client.cancel_goal()  # 直接调用取消接口
            return True
        rospy.logwarn("没有正在进行的录制任务可取消")
        return False

    def _active_callback(self):
        """服务器开始处理目标时的回调"""
        self.is_recording = True
        rospy.loginfo("服务器已接受录制请求，开始录制...")

    def _feedback_callback(self, feedback):
        """进度反馈回调"""
        progress = feedback.progress
        elapsed = feedback.elapsed_time

        if progress > 0:
            rospy.loginfo_throttle(
                DEFAULT_LOG_PERIOD_S,
                f"录制进度: {progress:.1f}% | 已录制: {elapsed:.1f}s",
            )
        else:
            rospy.loginfo_throttle(
                DEFAULT_LOG_PERIOD_S, f"持续录制中 | 已录制: {elapsed:.1f}s"
            )

    def _done_callback(self, status, result):
        """任务完成回调"""
        self.is_recording = False

        # 解析最终状态
        status_text = {
            actionlib.GoalStatus.SUCCEEDED: "成功完成",
            actionlib.GoalStatus.PREEMPTED: "已被取消",
            actionlib.GoalStatus.ABORTED: "异常终止",
        }.get(status, "未知状态")

        # 处理结果
        if result.success:
            rospy.loginfo(f"录制{status_text} | 文件保存至: {result.bag_path}")
        else:
            rospy.logwarn(f"录制{status_text} | 未生成有效文件")

    def _status_callback(self, msg):
        self.is_recording = msg.data
        rospy.loginfo(f"录制状态: {'正在录制' if self.is_recording else '未在录制'}")

    def _detailed_status_callback(self, msg):
        rospy.loginfo(f"录制详细状态: {msg.data}")
