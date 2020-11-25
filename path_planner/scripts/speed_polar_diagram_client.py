#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg

def speed_polar_diagram_client():
    pub = rospy.Publisher('to_speed_diagram', std_msgs.msg.Int64MultiArray, queue_size=10)
    rospy.init_node('path_planner_partner', anonymous=True)
    rate = rospy.Rate(60)  # 60hz

    mat = std_msgs.msg.Int64MultiArray()
    mat.data = [10, 20]

    while not rospy.is_shutdown():
        rospy.loginfo(mat.data)
        pub.publish(mat)
        rate.sleep()

if __name__ == '__main__':
    try:
        speed_polar_diagram_client()
    except rospy.ROSInterruptException:
        pass