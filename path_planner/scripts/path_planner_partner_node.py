#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg

def path_planner_partner_publisher():
    pub = rospy.Publisher('path_planner/waypoints', std_msgs.msg.Int64MultiArray, queue_size=10)
    rospy.init_node('path_planner_partner')
    rate = rospy.Rate(60)  # 60hz

    mat = std_msgs.msg.Int64MultiArray()
    mat.data = [10, 20]
    while not rospy.is_shutdown():
        rospy.loginfo(mat.data)
        pub.publish(mat)
        rate.sleep()

if __name__ == '__main__':
    try:
        path_planner_partner_publisher()
    except rospy.ROSInterruptException:
        pass