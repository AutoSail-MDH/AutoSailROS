#!/usr/bin/env python
import rospy
import std_msgs.msg
import yaml
from ctrl.sail_controller import sail_angle_calculation


class SubscriberValues:
    def __init__(self):
        self.desired_angle = 0.0

    def callback_SailAngle(self, data):
        if data.data is not None:
            self.desired_angle = data.data


if __name__ == "__main__":
    rospy.init_node("sail_control")

    #  Variables
    values = SubscriberValues()
    Current_wind = 45
    rate = rospy.Rate(1)
    sail_limits = rospy.get_param("~sail_limits", 1)
    #  Publisher
    sail_angle = rospy.Publisher("sail_control_topic", std_msgs.msg.Float32, queue_size=1)

    #  subscriber
    rospy.Subscriber(name="sail_control_topic", data_class=std_msgs.msg.Float32, callback=values.callback_SailAngle,
                     queue_size=1)

    while not rospy.is_shutdown():
        # calculate for new sail position
        new_sail_angle = sail_angle_calculation(Current_wind, sail_limits)

        # publish in log
        Publish_txt = "New angle"
        rospy.loginfo(Publish_txt)
        rospy.loginfo(new_sail_angle)

        # Publish the sail angle
        sail_angle.publish(new_sail_angle)

        # Keep sync with the ROS frequency
        rate.sleep()

