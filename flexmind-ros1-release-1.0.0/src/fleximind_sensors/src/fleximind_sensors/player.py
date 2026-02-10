import os
from typing import Any, Iterator, Tuple

import rosbag
from mcap_ros1.reader import read_ros1_messages as read_mcap_messages  # MCAP读取器

from fleximind_sensors.base import BaseBagReader


class RosbagReader(BaseBagReader):
    """ROS Bag格式读取器[3,5](@ref)"""

    def __init__(self, file_path: str):
        super().__init__(file_path)
        self.bag = rosbag.Bag(file_path, "r")

    def read_messages(self) -> Iterator[Tuple[str, Any, float]]:
        """迭代读取ROS Bag消息"""
        for topic, msg, t in self.bag.read_messages():
            yield topic, msg, t.to_sec()  # 时间戳转换为秒

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.bag.close()


class McapReader(BaseBagReader):
    """MCAP格式读取器[1](@ref)"""

    def __init__(self, file_path):
        super().__init__(file_path)
        self.file = open(self.file_path, "rb")

    def __enter__(self):
        # self.file = open(self.file_path, "rb")
        # return read_mcap_messages(self.file)  # 返回生成器
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.file:
            self.file.close()
        # 返回False让异常继续传播
        return False

    def read_messages(self) -> Iterator[Tuple[str, Any, float]]:
        """迭代读取MCAP消息"""
        with open(self.file_path, "rb") as f:
            for mcap_message in read_mcap_messages(f):
                # 时间戳转换为秒
                yield mcap_message.topic, mcap_message.ros_msg, mcap_message.publish_time_ns / 1e9


def create_bag_reader(file_path: str) -> BaseBagReader:
    """工厂方法创建合适的读取器实例"""
    ext = os.path.splitext(file_path)[1].lower()

    if ext == ".bag":
        return RosbagReader(file_path)
    elif ext == ".mcap":
        return McapReader(file_path)
    else:
        raise ValueError(f"不支持的格式: {ext}")


# 使用示例
if __name__ == "__main__":
    bag_file = "/home/robot/fleximind-ros1/data/bags/recording_20250815-161840.bag"
    mcap_file = "/home/robot/fleximind-ros1/data/bags/recording_20250815-161738.mcap"

    # 自动检测格式并创建读取器
    for file_path in [bag_file, mcap_file]:
        try:
            with create_bag_reader(file_path) as reader:
                print(f"\n读取文件: {file_path} ({reader.file_format.name}格式)")

                # 读取并处理消息
                for i, (topic, msg, timestamp) in enumerate(reader.read_messages()):
                    print(f"消息 #{i+1}")
                    print(f"  话题: {topic}")
                    print(f"  时间戳: {timestamp:.6f} 秒")

                    # 提取图像消息的特殊处理
                    if "Image" in str(type(msg)):
                        print(f"  图像尺寸: {msg.width}x{msg.height}")
                        print(f"  编码格式: {msg.encoding}")

                    # 只显示前5条消息示例
                    if i >= 4:
                        print("... (更多消息省略)")
                        break

        except Exception as e:
            import traceback

            traceback.print_exc()
            print(f"处理文件 {file_path} 时出错: {str(e)}")
