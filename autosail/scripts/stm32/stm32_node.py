#!/usr/bin/env python
import rospy
import std_msgs.msg
import numpy as np
from stm32.stm32_handle import sensor_readings
from autosail.msg import stm32_msg

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
    stm32_sensors = rospy.Publisher("stm32_handle/sensor_readings", stm32_msg, queue_size=queue_size)

    #  subscriber wind sensor readings
    #rospy.Subscriber(name="sail_control_topic", data_class=std_msgs.msg.Float32, callback=values.callback_SailAngle,
    #                 queue_size=queue_size)

    #   ADCCurrent = data(2)+data(1)
    #   I2C[0] = data(4) + data(3)
    #   I2C[1] = data(6) + data(5)
    #   I2C[2] = data(8) + data(7)
    #   WaterDetect1 = data(9)
    #   WaterDetect2 = data(10)
    #   pump = data(11)

    sensor_send = stm32_msg()
    while not rospy.is_shutdown():
        # Get sensor values from stm32
        sensor_values = sensor_readings()

        # position the sensor values correct
        sensor_send.adc_current = sensor_values[1] + sensor_values[0]
        sensor_send.I2c_current_1 = sensor_values[3] + sensor_values[2]
        sensor_send.I2c_current_2 = sensor_values[5] + sensor_values[4]
        sensor_send.I2c_current_3 = sensor_values[7] + sensor_values[6]
        sensor_send.water_detect_1 = sensor_values[8]
        sensor_send.water_detect_2 = sensor_values[9]
        sensor_send.pump = sensor_values[10]

        stm32_sensors.publish(sensor_send)
        rate.sleep()

