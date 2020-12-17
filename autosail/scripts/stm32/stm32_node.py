#!/usr/bin/env python
import rospy
from stm32.stm32_handle import sensor_readings, openSTM32Serial, closeSTM32Serial
from autosail.msg import stm32_msg


if __name__ == "__main__":
    rospy.init_node("stm32_handle", log_level=rospy.get_param("log_level", rospy.INFO))

    #  Variables
    queue_size = rospy.get_param("~queue_size", 60)
    predefined_rate = rospy.get_param("~rate", 60)
    rate = rospy.Rate(predefined_rate)
    timeout = rospy.get_param("~timeout", 1)
    port = rospy.get_param("~port", "/dev/ttyACM60")

    #  Publisher
    stm32_sensors = rospy.Publisher("stm32_handle/sensor_readings", stm32_msg, queue_size=queue_size)

    #  subscriber wind sensor readings

    sensor_send = stm32_msg()
    try:
        openSTM32Serial(port)
    except:
        rospy.logfatal(f"Could not open port {port} for STM32")
        rospy.signal_shutdown(f"Could not open port {port} for STM32")
    while not rospy.is_shutdown():
        # Get sensor values from stm32
        sensor_values = sensor_readings(timeout=timeout)
        if sensor_values and len(sensor_values) < 12:
            # position the sensor values correct
            sensor_send.adc_current = (sensor_values[1] + sensor_values[0]) / 100
            sensor_send.I2c_current_1 = (sensor_values[3] + sensor_values[2]) / 100
            sensor_send.I2c_current_2 = (sensor_values[5] + sensor_values[4]) / 100
            sensor_send.I2c_current_3 = (sensor_values[7] + sensor_values[6]) / 100
            sensor_send.Battery = (sensor_values[9] + sensor_values[8]) / 100
            sensor_send.water_detect_1 = sensor_values[10]
            sensor_send.water_detect_2 = sensor_values[11]
            sensor_send.pump = sensor_values[12]
            rospy.logdebug("Received data from STM32: {}".format(sensor_values))
        else:
            rospy.logerr("STM32 read timeout: did not receive any data under a period of {}s".format(timeout))
        stm32_sensors.publish(sensor_send)
        rate.sleep()
    closeSTM32Serial()
