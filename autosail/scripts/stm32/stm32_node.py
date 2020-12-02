#!/usr/bin/env python
import rospy
import std_msgs.msg
import numpy as np
from stm32.stm32_handle import sensor_readings


class SubscriberValues:
    def __init__(self):
        self.desired_angle = 1.0

    def callback_SailAngle(self, data):
        if data.data is not None:
            self.desired_angle = data.data


if __name__ == "__main__":
    rospy.init_node("stm32_handle")

    #  Variables
    values = SubscriberValues()
    queue_size = rospy.get_param("~queue_size", 60)
    predefined_rate = rospy.get_param("~rate", 60)
    rate = rospy.Rate(predefined_rate)

    #  Publisher
    stm32_sensors = rospy.Publisher("stm32_handle/sensor_readings", std_msgs.msg.Int64, queue_size=queue_size)

    #  subscriber wind sensor readings
    #rospy.Subscriber(name="sail_control_topic", data_class=std_msgs.msg.Float32, callback=values.callback_SailAngle,
    #                 queue_size=queue_size)

    while not rospy.is_shutdown():
        # calculate for new sail position
        sensor_values = sensor_readings()

        # Publish the sensor values
        stm32_sensors.publish(sensor_values)

        # Keep sync with the ROS frequency
        rate.sleep()

