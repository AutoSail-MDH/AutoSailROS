#!/usr/bin/env python
import rospy
import std_msgs.msg
import numpy as np

waypoints = np.array([])


def waypoint_callback(data):
    global waypoints
    waypoints = data.data
    print("waypoints", waypoints[0])

    # pub = rospy.Publisher('path_planner/course', std_msgs.msg.Int64MultiArray, queue_size=10)
    # mat = std_msgs.msg.Int64MultiArray()
    # mat.data = [40, 50]
    # pub.publish(mat)

def gps_callback(data):
    print("gps")


def path_planner_subscriber():
    rospy.init_node('path_planner')
    rospy.Subscriber("path_planner/waypoints", std_msgs.msg.Int64MultiArray, waypoint_callback)
#    rospy.Subscriber("gps/fix", std_msgs.msg.Int64MultiArray, gps_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        path_planner_subscriber()
    except rospy.ROSInterruptException:
        pass
