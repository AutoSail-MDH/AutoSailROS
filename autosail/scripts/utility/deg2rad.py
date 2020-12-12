#!/usr/bin/env python
"""
Convert degrees to radians to simplify testing.
Usage:
    rosrun autosail deg2rad deg:=/rudder_angle_deg rad:=/motor_controller/rudder_angle
    rostopic pub /rudder_angle_deg Float64 "data: 40" -1
"""
import math
import rospy
from std_msgs.msg import Float64


def callback(msg):
    global rad_pub
    rad_pub.publish(math.radians(msg.data))


if __name__ == "__main__":
    rospy.init_node("deg2rad", anonymous=True, log_level=rospy.get_param("log_level", rospy.INFO))
    rad_pub = rospy.Publisher("rad", Float64, queue_size=1)
    rospy.Subscriber("deg", Float64, callback, queue_size=1)
    rospy.spin()

