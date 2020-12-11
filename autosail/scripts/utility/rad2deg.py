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
    global deg_pub
    deg_pub.publish(math.degrees(msg.data))


if __name__ == "__main__":
    rospy.init_node("rad2deg", anonymous=True)
    deg_pub = rospy.Publisher("deg", Float64, queue_size=1)
    rospy.Subscriber("rad", Float64, callback, queue_size=1)
    rospy.spin()

