#!/usr/bin/env python
import json
from pathlib import Path
from threading import Lock

import rospy

from fleximind_bringup.interface import ManagingConfig

try:
    from ruamel.yaml import YAML
    from ruamel.yaml.comments import CommentedMap, CommentedSeq
except ImportError:
    rospy.logerr("ruamel.yaml not installed. Please run: pip install ruamel.yaml")
    exit(1)

from fleximind_bringup.ros1_logger_utils import ros_service_logger
from fleximind_hardware.srv import (
    ConfigService,
    ConfigServiceRequest,
    ConfigServiceResponse,
)


class ConfigManager:
    def __init__(self):
        rospy.init_node("config_manager")
        rospy.loginfo("配置管理服务已启动...")

        # 初始化ruamel.yaml - 关键配置
        self.yaml = YAML()
        self.yaml.preserve_quotes = True  # 保留引号
        self.yaml.indent(mapping=2, sequence=4, offset=2)  # 设置缩进

        # 从参数服务器获取ROS工作空间路径
        self.workspace_path = rospy.get_param(
            "~workspace_path", "/home/robot/fleximind-ros1"
        )

        # 配置文件路径映射和格式偏好
        self.config_files = {
            "gripper": {
                "package": "fleximind_gripper",
                "file": "gripper_config.yaml",
                "path": f"{self.workspace_path}/src/fleximind_gripper/config/gripper_config.yaml",
                "list_format": "flow",  # 内联格式 [a, b, c]
            },
            "remote": {
                "package": "fleximind_remote",
                "file": "remote.yaml",
                "path": f"{self.workspace_path}/src/fleximind_remote/config/remote.yaml",
                "list_format": "block",  # 块格式
            },
            "robot": {
                "package": "fleximind_robot",
                "file": "config.yaml",
                "path": f"{self.workspace_path}/src/fleximind_robot/config/config.yaml",
                "list_format": "flow",  # 默认使用内联格式
            },
            "record": {
                "package": "fleximind_sensors",
                "file": "record.yaml",
                "path": f"{self.workspace_path}/src/fleximind_sensors/config/record.yaml",
                "list_format": "flow",  # 默认使用内联格式
            },
        }

        # 线程安全锁
        self.config_lock = Lock()

        # 只保存一份配置数据，使用ruamel.yaml对象直接操作
        self.config_data = {}

        # 加载初始配置
        self.load_all_configs()

        # 配置管理服务
        self._config_service = rospy.Service(
            ManagingConfig.CONFIG.value, ConfigService, self.handle_config_service
        )

    def load_all_configs(self):
        """加载所有配置文件到内存"""
        with self.config_lock:
            for config_type, info in self.config_files.items():
                file_path = Path(info["path"])
                try:
                    if file_path.exists():
                        with open(file_path, "r", encoding="utf-8") as f:
                            # 直接加载ruamel.yaml对象，保持所有格式
                            yaml_content = self.yaml.load(f)
                            if yaml_content is None:
                                yaml_content = CommentedMap()
                            self.config_data[config_type] = yaml_content
                        rospy.loginfo(f"成功加载配置文件: {info['file']}")
                    else:
                        rospy.logwarn(f"配置文件不存在: {file_path}")
                        self.config_data[config_type] = self.get_default_config(
                            config_type
                        )
                except Exception as e:
                    rospy.logerr(f"加载配置文件 {info['file']} 失败: {str(e)}")
                    self.config_data[config_type] = self.get_default_config(config_type)

    def get_default_config(self, config_type):
        """获取默认配置"""
        defaults = {
            "gripper": CommentedMap(
                [
                    ("control_timer_period", 0.2),
                    ("reader_timer_period", 0.4),
                    ("fleximind_gripper_node", CommentedSeq()),
                ]
            ),
            "remote": CommentedMap(
                [
                    ("arm_type", "bimanual"),
                    ("hostname", "127.0.0.1"),
                    ("hz", 100),
                    ("left_arm", CommentedMap()),
                    ("right_arm", CommentedMap()),
                ]
            ),
            "robot": CommentedMap(
                [
                    ("robot_name", "dual_robot"),
                    ("max_speed", 1.0),
                    ("fleximind_robot_node", CommentedMap()),
                ]
            ),
            "record": CommentedMap(
                [
                    (
                        "record_bag",
                        CommentedMap(
                            [
                                ("test_mode", False),
                                ("storage_format", "bag"),
                                ("output_dir", "/home/robot/fleximind-ros1/data/bags"),
                                ("topics", CommentedSeq()),
                                ("topics_type", CommentedSeq()),
                                ("duration", 0.0),
                            ]
                        ),
                    )
                ]
            ),
        }
        return defaults.get(config_type, CommentedMap())

    @ros_service_logger
    def handle_config_service(self, request: ConfigServiceRequest):
        """处理配置服务请求"""
        response = ConfigServiceResponse()

        try:
            config_type = request.config_type.lower()
            operation = request.operation.lower()

            # 验证配置类型
            if config_type not in self.config_files:
                response.success = False
                response.message = f"不支持的配置类型: {config_type}。支持的类型: {list(self.config_files.keys())}"
                return response

            # 根据操作类型处理
            if operation == "get_all":
                self._get_all_config(config_type, response)
            elif operation == "get_param":
                self._get_parameter(config_type, request.param_path, response)
            elif operation == "set_and_save":  # 修改：整合set和save操作
                self._set_and_save_parameter(
                    config_type, request.param_path, request.param_value, response
                )
            else:
                response.success = False
                response.message = f"不支持的操作: {operation}。支持的操作: get_all, get_param, set_and_save"

        except Exception as e:
            response.success = False
            response.message = f"配置操作失败: {str(e)}"
            rospy.logerr(f"配置操作异常: {str(e)}")

        return response

    def _get_all_config(self, config_type: str, response: ConfigServiceResponse):
        """获取指定类型的所有配置"""
        with self.config_lock:
            config_content = self.config_data.get(config_type, {})

            # 转换为普通字典用于JSON序列化
            def commented_to_dict(obj):
                if isinstance(obj, (CommentedMap, dict)):
                    return {k: commented_to_dict(v) for k, v in obj.items()}
                elif isinstance(obj, (CommentedSeq, list)):
                    return [commented_to_dict(item) for item in obj]
                else:
                    return obj

            dict_content = commented_to_dict(config_content)

            # 添加文件路径信息
            dict_content["_file_info"] = {
                "package": self.config_files[config_type]["package"],
                "file_path": self.config_files[config_type]["path"],
                "file_name": self.config_files[config_type]["file"],
                "list_format": self.config_files[config_type]["list_format"],
            }

            # 转换为JSON字符串供前端显示
            try:
                json_content = json.dumps(dict_content, ensure_ascii=False, indent=2)
                response.success = True
                response.config_content = json_content
                response.message = f"获取 {config_type} 配置成功"
            except Exception as e:
                response.success = False
                response.message = f"配置序列化失败: {str(e)}"

    def _get_parameter(
        self, config_type: str, param_path: str, response: ConfigServiceResponse
    ):
        """获取特定参数值"""
        if not param_path:
            response.success = False
            response.message = "参数路径不能为空"
            return

        with self.config_lock:
            config = self.config_data.get(config_type, {})
            value = self._get_nested_value(config, param_path)

            if value is not None:
                response.success = True
                response.config_content = str(value)
                response.message = f"获取参数 {param_path} 成功"
            else:
                response.success = False
                response.message = f"参数不存在: {param_path}"

    def _set_and_save_parameter(
        self,
        config_type: str,
        param_path: str,
        param_value: str,
        response: ConfigServiceResponse,
    ):
        """设置参数值并立即保存到文件（整合操作）"""
        if not param_path:
            response.success = False
            response.message = "参数路径不能为空"
            return

        with self.config_lock:
            try:
                # 转换值类型
                converted_value = self._convert_value_type(param_value)

                # 获取该配置文件的列表格式偏好
                list_format = self.config_files[config_type]["list_format"]

                # 更新配置数据
                self._set_nested_value(
                    self.config_data[config_type],
                    param_path,
                    converted_value,
                    list_format,
                )

                # 立即保存到文件
                file_path = self.config_files[config_type]["path"]
                Path(file_path).parent.mkdir(parents=True, exist_ok=True)

                with open(file_path, "w", encoding="utf-8") as f:
                    self.yaml.dump(self.config_data[config_type], f)

                response.success = True
                response.config_content = str(converted_value)
                response.message = f"参数 {param_path} 更新并保存成功"
                rospy.loginfo(
                    f"更新并保存配置 {config_type}.{param_path} = {converted_value}"
                )

            except Exception as e:
                response.success = False
                response.message = f"参数设置并保存失败: {str(e)}"
                rospy.logerr(f"设置参数失败详情: {str(e)}")

    def _get_nested_value(self, config: dict, path: str):
        """获取嵌套字典的值"""
        keys = path.split(".")
        current = config

        for key in keys:
            # 处理列表索引的情况
            if isinstance(current, list) and key.isdigit():
                index = int(key)
                if 0 <= index < len(current):
                    current = current[index]
                else:
                    return None
            elif isinstance(current, dict) and key in current:
                current = current[key]
            else:
                return None
        return current

    def _set_nested_value(self, config: dict, path: str, value, list_format="flow"):
        """设置嵌套字典的值"""
        keys = path.split(".")
        current = config

        # 遍历到倒数第二个key，创建必要的嵌套结构
        for i, key in enumerate(keys[:-1]):
            # 处理列表索引的情况
            if isinstance(current, list) and key.isdigit():
                index = int(key)
                if 0 <= index < len(current):
                    current = current[index]
                else:
                    raise IndexError(f"列表索引越界: {key} in {path}")
            elif isinstance(current, dict):
                if key not in current:
                    # 检查下一个key是否是数字（列表索引）
                    next_key = keys[i + 1] if i + 1 < len(keys) else None
                    if next_key and next_key.isdigit():
                        current[key] = CommentedSeq()
                    else:
                        current[key] = CommentedMap()
                current = current[key]
            else:
                # 如果当前不是dict或list，无法继续嵌套
                raise TypeError(
                    f"无法在类型 {type(current).__name__} 上设置嵌套属性 {key}"
                )

        # 设置最终值
        final_key = keys[-1]
        if isinstance(current, list) and final_key.isdigit():
            index = int(final_key)
            if 0 <= index < len(current):
                current[index] = value
            else:
                raise IndexError(f"列表索引越界: {final_key}")
        elif isinstance(current, dict):
            # 根据配置文件类型设置不同的列表格式
            if isinstance(value, list):
                seq = CommentedSeq(value)
                if list_format == "flow":
                    # 内联格式: [a, b, c]
                    seq.fa.set_flow_style()
                else:
                    # 块格式: 每个元素单独一行
                    seq.fa.set_block_style()
                current[final_key] = seq
            else:
                current[final_key] = value
        else:
            raise TypeError(
                f"无法在类型 {type(current).__name__} 上设置属性 {final_key}"
            )

    def _convert_value_type(self, value_str: str):
        """智能转换参数值类型"""
        if not value_str:
            return ""

        # 布尔值
        if value_str.lower() in ["true", "false"]:
            return value_str.lower() == "true"

        # 数字
        try:
            if "." in value_str:
                return float(value_str)
            else:
                return int(value_str)
        except ValueError:
            pass

        # 列表或字典（JSON格式）
        if (value_str.startswith("[") and value_str.endswith("]")) or (
            value_str.startswith("{") and value_str.endswith("}")
        ):
            try:
                return json.loads(value_str)
            except:
                pass

        # 字符串
        return value_str

    def reload_configs(self):
        """重新加载所有配置"""
        self.load_all_configs()
        rospy.loginfo("所有配置文件已重新加载")


if __name__ == "__main__":
    config_manager = ConfigManager()
    rospy.loginfo("配置管理服务运行中...")
    rospy.spin()
