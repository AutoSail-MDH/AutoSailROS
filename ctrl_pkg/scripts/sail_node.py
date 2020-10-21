#!/usr/bin/env python
import rospy
rospy.init_node("sail_controller")
from std_msgs.msg import String


if __name__ == "__main__":
    sc = sail_controller()
    rospy.Subscriber(name="sail_control", data_class=std_msgs.msg.Float32, callback=sc.sail_angle_calculation,
                     queue_size=1)

    rospy.init_node("pid")
    x = 0.1
    pub_test1 = rospy.Publisher("pathplanner", std_msgs.msg.Float32, queue_size=1)