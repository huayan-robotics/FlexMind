import rospy
import functools
import time
import threading

# 节流控制字典（线程安全）
_throttle_data = {}
_throttle_lock = threading.Lock()


def ros_service_logger(func=None, *, logger=None, throttle_interval=1.0):
    """ROS1 服务日志装饰器（支持节流和异常处理）"""
    # 处理无括号调用场景
    if func is None:
        return lambda f: ros_service_logger(
            f, logger=logger, throttle_interval=throttle_interval
        )

    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        # 自动识别日志记录器
        node_logger = logger
        if not node_logger:
            # 尝试从参数中提取 ROS1 节点实例
            for arg in args:
                if hasattr(arg, "get_logger"):
                    node_logger = arg.get_logger()
                    break
            if not node_logger:
                node_logger = rospy  # 默认使用 rospy

        # 获取请求对象（通常是第二个参数）
        request = None
        if len(args) > 1:
            request = args[1]
        elif "request" in kwargs:
            request = kwargs["request"]

        # 节流控制键
        throttle_key = f"{func.__name__}_request"

        try:
            # 带节流的请求日志[5](@ref)
            if _should_log(throttle_key, throttle_interval):
                log_message = (
                    f"服务调用: {func.__name__} | 请求: {_format_request(request)}"
                )
                if hasattr(node_logger, "loginfo"):
                    node_logger.loginfo(log_message)  # 节点专用日志
                else:
                    rospy.loginfo(log_message)  # 全局日志

            # 执行原始函数
            response = func(*args, **kwargs)

            # 带节流的响应日志
            if _should_log(throttle_key, throttle_interval):
                response_info = _get_response_info(response)
                log_message = f"服务完成: {func.__name__} | 响应状态: {response_info}"
                if hasattr(node_logger, "loginfo"):
                    node_logger.loginfo(log_message)
                else:
                    rospy.loginfo(log_message)

            return response

        except Exception as e:
            # 异常日志（只记录首次发生）[5](@ref)
            error_key = f"{func.__name__}_error"
            if _should_log(error_key, 60.0):  # 60秒内不重复记录相同错误
                error_msg = f"服务异常: {func.__name__} | 错误: {str(e)}"
                if hasattr(node_logger, "logerr"):
                    node_logger.logerr(error_msg)
                else:
                    rospy.logerr(error_msg)
            raise  # 重新抛出异常

    return wrapper


def _should_log(key, interval):
    """检查是否应该记录日志（节流控制）"""
    with _throttle_lock:
        current_time = time.time()
        last_time = _throttle_data.get(key, 0)

        if current_time - last_time >= interval:
            _throttle_data[key] = current_time
            return True
        return False


def _format_request(request):
    """格式化请求对象为可读字符串"""
    if not request:
        return "None"

    try:
        # 尝试提取请求字段
        if hasattr(request, "__slots__"):
            return {slot: getattr(request, slot) for slot in request.__slots__}
        elif hasattr(request, "__dict__"):
            return {k: v for k, v in request.__dict__.items() if not k.startswith("_")}
        return str(request)
    except:
        return str(request)


def _get_response_info(response):
    """提取响应信息"""
    if not response:
        return "None"

    try:
        # 优先提取ROS服务响应中的状态字段
        if hasattr(response, "result") and hasattr(response, "values"):
            return f"result={response.result}, values={response.values}"
        return str(response)
    except:
        return str(response)
