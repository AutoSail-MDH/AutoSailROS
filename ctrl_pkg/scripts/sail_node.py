#!/usr/bin/env python
import rospy
import std_msgs.msg
import numpy as np
from ctrl.sail_controller import calculate_sail_angle
from ctrl.sail_controller import trim_sail


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
    predefined_rate = rospy.get_param("~rate", 60)
    rate = rospy.Rate(predefined_rate)
    sail_limits = rospy.get_param("~sail_limits", np.pi/5.2)
    queue_size = rospy.get_param("~queue_size", 1)

    #  Publisher
    sail_angle = rospy.Publisher("sail_control/sail_angle", std_msgs.msg.Float32, queue_size=queue_size)
    sail_servo = rospy.Publisher("sail_control/sail_servo_angle", std_msgs.msg.Float32, queue_size=queue_size)

    #  subscriber wind sensor readings
    rospy.Subscriber(name="sail_control_topic", data_class=std_msgs.msg.Float32, callback=values.callback_SailAngle,
                     queue_size=queue_size)

    while not rospy.is_shutdown():
        # calculate for new sail position
        new_sail_angle_rad = calculate_sail_angle(values.desired_angle, sail_limits)
        trim_degree = trim_sail(new_sail_angle_rad)

        # Publish the sail angle
        sail_angle.publish(new_sail_angle_rad)
        sail_servo.publish(trim_degree)

        # Keep sync with the ROS frequency
        rate.sleep()

