import os
import time
import yaml
import rosbag
import shutil
from concurrent.futures import ThreadPoolExecutor
from fleximind_interaction.msg import PlaybackItem


class PlaybackManager:
    def __init__(self, max_workers=4):
        self.func_executor = ThreadPoolExecutor(max_workers=max_workers)

    def get_playback_list(self, target_dir="data/bags/"):
        if not os.path.isdir(target_dir):
            return False, []

        # 递归扫描 .bag/.db3 文件
        bag_files = []
        for root, _, files in os.walk(target_dir):
            for file in files:
                if file.endswith((".bag", ".db3")):
                    bag_files.append(os.path.join(root, file))

        if not bag_files:
            return True, []

        # 并行解析 metadata
        futures = [
            self.func_executor.submit(self.extract_bag_metadata, path)
            for path in bag_files
        ]
        playback_list = [f.result() for f in futures if f.result()]

        return True, playback_list

    def extract_bag_metadata(self, bag_path):
        try:
            # 解析 ROS 1 .bag 文件
            if bag_path.endswith(".bag"):
                return self.extract_ros1_bag_metadata(bag_path)

            # 解析 ROS 2 .db3 文件
            elif bag_path.endswith(".db3"):
                return self.extract_ros2_bag_metadata(bag_path)

            return None
        except Exception as e:
            print(f"Error processing {bag_path}: {e}")
            return None

    def extract_ros1_bag_metadata(self, bag_path):
        try:
            bag = rosbag.Bag(bag_path)
            item = PlaybackItem()
            item.name = os.path.basename(bag_path)
            item.full_path = bag_path

            # 获取话题信息
            topics = bag.get_type_and_topic_info()[1]
            item.topics = list(topics.keys())
            item.message_count = sum(info.message_count for info in topics.values())

            # 获取开始时间
            start_time = bag.get_start_time()
            item.timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(start_time))

            bag.close()
            return item
        except Exception as e:
            print(f"Error extracting ROS 1 metadata from {bag_path}: {e}")
            return None

    def extract_ros2_bag_metadata(self, bag_path):
        try:
            bag_dir = os.path.dirname(bag_path)
            metadata_path = os.path.join(bag_dir, "metadata.yaml")
            if not os.path.exists(metadata_path):
                return None

            with open(metadata_path, "r") as f:
                metadata = yaml.safe_load(f)

            parent_dir = os.path.basename(os.path.dirname(bag_path))
            filename = os.path.basename(bag_path)

            item = PlaybackItem()
            item.name = f"{parent_dir}/{filename}"
            item.full_path = bag_path

            rosbag_info = metadata.get("rosbag2_bagfile_information", {})
            item.topics = [
                t["topic_metadata"]["name"]
                for t in rosbag_info.get("topics_with_message_count", [])
            ]
            item.message_count = sum(
                t.get("message_count", 0)
                for t in rosbag_info.get("topics_with_message_count", [])
            )
            if "duration" in rosbag_info:
                item.duration_sec = rosbag_info["duration"]["nanoseconds"] * 1e-9
            if "starting_time" in rosbag_info:
                start_time = rosbag_info["starting_time"]["nanoseconds_since_epoch"]
                item.timestamp = time.strftime(
                    "%Y-%m-%d %H:%M:%S", time.gmtime(start_time * 1e-9)
                )

            return item
        except Exception as e:
            print(f"Error extracting ROS 2 metadata from {bag_path}: {e}")
            return None

    def delete_playback_files(self, file_paths):
        try:
            deleted_files = []
            failed_deletions = []
            
            for file_path in file_paths:
                if not os.path.exists(file_path):
                    failed_deletions.append((file_path, "File not found"))
                    continue
                
                try:
                    os.remove(file_path)
                    deleted_files.append(file_path)
                    
                    if file_path.endswith('.db3'):
                        file_dir = os.path.dirname(file_path)
                        metadata_path = os.path.join(file_dir, 'metadata.yaml')
                        if os.path.exists(metadata_path):
                            os.remove(metadata_path)
                            
                except Exception as e:
                    failed_deletions.append((file_path, str(e)))
            
            result = {
                'deleted_files': deleted_files,
                'failed_deletions': failed_deletions,
                'total_attempted': len(file_paths),
                'successful': len(deleted_files),
                'failed': len(failed_deletions)
            }
            
            return True, result
            
        except Exception as e:
            return False, f"Delete failed: {str(e)}"