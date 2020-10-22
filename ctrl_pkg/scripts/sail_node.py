#!/usr/bin/env python
import rospy
import std_msgs.msg
from ctrl.sail_controller import sail_angle_calculation


if __name__ == "__main__":
    rospy.init_node("sail_control")
    sail_limits = rospy.get_param('~sail_limits')
    pub_testSC = rospy.Publisher("sail_control_topic", std_msgs.msg.Int64, queue_size=1)
    rate = rospy.Rate(10)
 #   rospy.Subscriber(name="sail_control_topic", data_class=std_msgs.msg.Int64,
 #                       callback=sail_angle_calculation, queue_size=1)

    while not rospy.is_shutdown():
        sail_angle = sail_angle_calculation(90, sail_limits)
        pub_testSC.publish()
        rate.sleep()
