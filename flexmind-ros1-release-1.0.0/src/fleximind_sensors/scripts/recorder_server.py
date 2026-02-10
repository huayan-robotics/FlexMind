#!/usr/bin/env python
import rospy

from fleximind_sensors.record_bag_server import RecordBagServer


if __name__ == "__main__":
    try:
        node_name = "record_bag"
        rospy.init_node(node_name)
        server = RecordBagServer(node_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("程序被用户中断")
