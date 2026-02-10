#!/usr/bin/env python
import rospy
import time
from fleximind_sensors.record_bag_client import RecordBagClient

if __name__ == "__main__":
    try:
        node_name = "record_bag"
        rospy.init_node(node_name)

        # 从参数服务器获取配置
        topics = rospy.get_param(
            "/record_bag/topics", ["/camera/left_hand/color/image_raw"]
        )
        topics_type = rospy.get_param("/record_bag/topics_type", ["sensor_msgs/Image"])
        duration = rospy.get_param("/record_bag/duration", 10.0)
        test_mode = rospy.get_param("/record_bag/test_mode", False)

        client = RecordBagClient(node_name)
        if test_mode:
            rospy.loginfo("记录模块测试模式")
            time.sleep(5)
            client.start_recording(topics, topics_type, duration)
            time.sleep(5)
            client.cancel_recording()
            time.sleep(5)
            client.start_recording(topics, topics_type, duration)
            time.sleep(5)
            client.cancel_recording()

    except rospy.ROSInterruptException:
        rospy.logerr("程序被用户中断")
