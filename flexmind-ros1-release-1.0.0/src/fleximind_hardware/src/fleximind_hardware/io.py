import enum
import time


class IOFunction(enum.Enum):
    LEFT_STOP = "left_stop"
    LEFT_RECORD = "left_record"
    RIGHT_STOP = "right_stop"
    RIGHT_LOCK = "right_lock"


DEFAULT_COUNT_TIMEOUT_S = 1.0  # 默认触发1秒超时


class RobotIO:
    def __init__(
        self,
        arm_type: str,
        bit: int,
        count_timeout_sec: float = DEFAULT_COUNT_TIMEOUT_S,
    ):
        self.arm_type: str = arm_type
        self.bit: int = bit

        self.activate_time: float = 0.0
        self._last_change_time: float = time.time()
        self._current_state: bool = False
        self._high_trigger_count = 0  # 高电平触发计数

        # --- 时效性相关变量 ---
        self._count_timeout = count_timeout_sec  # 计数有效时长（秒）
        self._last_count_access_time = time.time()  # 上次访问计数的时间戳，None表示从未访问

    @property
    def current_state(self):
        return self._current_state

    @current_state.setter
    def current_state(self, state: bool):
        if self._current_state == state:
            self.activate_time = time.time() - self._last_change_time
        else:
            self._last_change_time = time.time()
            self.activate_time = 0.0
            if state:  # 仅在变为高电平时计数
                self._high_trigger_count += 1
        self._current_state = state

    @property
    def high_trigger_count(self):
        """获取高电平触发次数。如果超过指定时间未访问，则计数自动清零。"""
        current_time = time.time()

        # --- 时效性判断逻辑 ---
        # 如果之前访问过，且距离上次访问已超时，则重置计数
        if (
            self._last_count_access_time is not None
            and current_time - self._last_count_access_time > self._count_timeout
        ):
            self._high_trigger_count = 0  # 超时清零

        # 更新最后一次访问时间为当前时刻
        self._last_count_access_time = current_time

        # 返回当前的计数值
        count = self._high_trigger_count
        self._high_trigger_count = 0 if count <= 1 else count - 1
        return count

    def peek_high_trigger_count(self):
        """查看当前计数而不更新访问时间或进行任何操作"""
        return self._high_trigger_count
