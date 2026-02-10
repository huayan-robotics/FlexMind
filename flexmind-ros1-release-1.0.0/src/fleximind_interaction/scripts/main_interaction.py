#!/usr/bin/env python
import os
import subprocess
import threading

import rospy

from fleximind_bringup.interface import MainInteracting
from fleximind_bringup.ros1_logger_utils import ros_service_logger
from fleximind_interaction.playback_manager import PlaybackManager
from fleximind_interaction.srv import (
    DeletePlaybackFiles,
    DeletePlaybackFilesRequest,
    DeletePlaybackFilesResponse,
    ExportPlaybackFiles,
    ExportPlaybackFilesRequest,
    ExportPlaybackFilesResponse,
    GetPlaybackList,
    GetPlaybackListRequest,
    GetPlaybackListResponse,
    GetVersion,
    GetVersionRequest,
    GetVersionResponse,
    ImportPlaybackFiles,
    ImportPlaybackFilesRequest,
    ImportPlaybackFilesResponse,
)


class FlexiMindInteraction:
    def __init__(self):
        rospy.init_node("fleximind_interaction")
        rospy.loginfo("FlexiMind服务已启动...")

        self.playback_manager = PlaybackManager()
        self.playback_lock = threading.Lock()
        self.current_playback_process = None

        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.workspace_dir = os.path.abspath(os.path.join(current_dir, "../../.."))
        # 验证路径是否存在
        if not os.path.exists(self.workspace_dir):
            raise FileNotFoundError(f"工作目录不存在: {self.workspace_dir}")
        # 验证是否是预期的目录（可选）
        expected_dir_name = "fleximind-ros1"
        if os.path.basename(self.workspace_dir) != expected_dir_name:
            raise ValueError(
                f"工作目录名称不符，期望: {expected_dir_name}，实际: {os.path.basename(self.workspace_dir)}"
            )
        self.default_bags_directory = os.path.join(self.workspace_dir, "data/bags")

        # 服务接口
        self._version_srv = rospy.Service(
            MainInteracting.VERSION.value, GetVersion, self.get_version
        )
        self._playback_srv = rospy.Service(
            MainInteracting.GET_PLAYBACK_LIST.value,
            GetPlaybackList,
            self.get_playback_list,
        )
        self._export_files_srv = rospy.Service(
            MainInteracting.EXPORT_PLAYBACK_FILES.value,
            ExportPlaybackFiles,
            self.export_playback_files,
        )
        self._import_srv = rospy.Service(
            MainInteracting.IMPORT_PLAYBACK_FILES.value,
            ImportPlaybackFiles,
            self.import_playback_files,
        )
        self._delete_srv = rospy.Service(
            MainInteracting.DELETE_PLAYBACK.value,
            DeletePlaybackFiles,
            self.delete_playback_files,
        )

    @ros_service_logger
    def get_version(self, request: GetVersionRequest):
        response = GetVersionResponse()
        response.version = "v1.0.0"
        response.code = 200
        return response

    @ros_service_logger
    def get_playback_list(self, request: GetPlaybackListRequest):
        target_dir = request.target_directory or self.default_bags_directory
        success, playback_list = self.playback_manager.get_playback_list(target_dir)

        response = GetPlaybackListResponse()
        response.success = success
        response.playback_list = playback_list
        return response

    @ros_service_logger
    def export_playback_files(self, request: ExportPlaybackFilesRequest):
        response = ExportPlaybackFilesResponse()
        file_names = list(request.file_paths)
        if not file_names:
            response.success = False
            response.message = "The file name cannot be empty"
            return response

        try:
            export_path = []
            fail_path = []

            for file_name in file_names:
                file_path = os.path.join(self.default_bags_directory, file_name)
                if os.path.exists(file_path):
                    export_path.append(file_path)
                    rospy.loginfo(f"Found the file: {file_path}")
                else:
                    fail_path.append(f"{file_name}: File does not exist")
                    rospy.loginfo(f"File does not exist: {file_path}")

            response.success = True
            response.export_path = export_path
            response.message = f"Export query completed: {len(export_path)} file(s) found, {len(fail_path)} file(s) not found"
        except Exception as e:
            response.success = False
            response.message = (
                f"An error occurred during the export query process: {str(e)}"
            )
        return response

    @ros_service_logger
    def import_playback_files(self, request: ImportPlaybackFilesRequest):
        response = ImportPlaybackFilesResponse()
        try:
            response.success = True
            response.import_path = [self.default_bags_directory]
            response.message = f"Import folder path: {self.default_bags_directory}"
            rospy.loginfo(f"Return import folder path: {self.default_bags_directory}")
        except Exception as e:
            response.success = False
            response.message = (
                f"An error occurred while retrieving the import folder path: {str(e)}"
            )
        return response

    @ros_service_logger
    def delete_playback_files(self, request: DeletePlaybackFilesRequest):
        response = DeletePlaybackFilesResponse()
        video_name = request.video_name

        if not video_name:
            response.success = False
            response.message = "Video name cannot be empty"
            return response

        video_path = os.path.join(self.default_bags_directory, video_name)

        try:
            if not os.path.exists(video_path):
                response.success = False
                response.message = f"File does not exist: {video_path}"
                return response

            rospy.loginfo(f"Attempting to delete file: {video_path}")
            os.remove(video_path)

            if not os.path.exists(video_path):
                response.success = True
                response.message = f"Successfully deleted file: {video_name}"
                rospy.loginfo(f"Successfully deleted file: {video_path}")
            else:
                response.success = False
                response.message = f"Failed to delete file: {video_name}"
                rospy.logwarn(f"File still exists after deletion attempt: {video_path}")

        except Exception as e:
            response.success = False
            response.message = f"Error occurred while deleting file: {str(e)}"
            rospy.logerr(f"Error deleting file {video_path}: {str(e)}")

        return response


if __name__ == "__main__":
    fleximind_interaction = FlexiMindInteraction()
    rospy.spin()
