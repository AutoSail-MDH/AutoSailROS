#!/usr/bin/env python
import rospy
import std_msgs.msg
from ctrl.sail_controller import sail_angle_calculation


if __name__ == "__main__":
    rospy.init_node("sail_control")
    pub_testSC = rospy.Publisher("sail_control_topic", std_msgs.msg.Int64, queue_size=1)
    rate = rospy.Rate(10)
    rospy.Subscriber(name="sail_control_topic2", data_class=std_msgs.msg.Float32, callback=sail_angle_calculation,
                     queue_size=1)

    while not rospy.is_shutdown():
        pub_testSC.publish(100)
        rate.sleep()
