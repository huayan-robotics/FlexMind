import enum
import os
from typing import Any, Iterator, Tuple


class StorageFormat(enum.Enum):
    """支持的包格式类型"""

    BAG = "bag"
    MCAP = "mcap"


class BaseBagReader:
    """基础包读取器抽象类"""

    def __init__(self, file_path: str):
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"文件不存在: {file_path}")
        self.file_path = file_path
        self.file_format = self._detect_format()

    def _detect_format(self) -> StorageFormat:
        """根据扩展名检测文件格式"""
        ext = os.path.splitext(self.file_path)[1].lower()
        if ext == ".bag":
            return StorageFormat.BAG
        elif ext == ".mcap":
            return StorageFormat.MCAP
        else:
            raise ValueError(f"不支持的格式: {ext}")

    def read_messages(self) -> Iterator[Tuple[str, Any, float]]:
        """迭代读取消息 (topic, message, timestamp)"""
        raise NotImplementedError("子类必须实现此方法")


DEFAULT_CHUNK_THRESHOLD = 1 * 1024 * 1024  # 1MB


__ALL__ = ["StorageFormat", "BaseBagReader", "DEFAULT_CHUNK_THRESHOLD"]
