#!/usr/bin/env python
import rospy
from stm32.stm32_handle import sensor_readings, openSTM32Serial, closeSTM32Serial
from autosail.msg import stm32_msg


if __name__ == "__main__":
    rospy.init_node("stm32_handle", log_level=rospy.get_param("log_level", rospy.INFO))

    #  Variables
    queue_size = rospy.get_param("~queue_size", 60)
    predefined_rate = rospy.get_param("~rate", 1)
    rate = rospy.Rate(predefined_rate)
    timeout = rospy.get_param("~timeout", 1)
    port = rospy.get_param("~port", "/dev/ttyACM0")

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
        sensor_values = sensor_readings()
        if sensor_values and len(sensor_values) >= 13:
            # position the sensor values correct
            sensor_send.adc_current = ((sensor_values[1] << 8) | sensor_values[0]) / 100
            sensor_send.I2c_current_1 = ((sensor_values[3] << 8) | sensor_values[2]) / 100
            sensor_send.I2c_current_2 = ((sensor_values[5] << 8) | sensor_values[4]) / 100
            sensor_send.I2c_current_3 = ((sensor_values[7] << 8) | sensor_values[6]) / 100
            sensor_send.Battery = ((sensor_values[9] << 8) | sensor_values[8]) / 100
            sensor_send.water_detect_1 = sensor_values[10]
            sensor_send.water_detect_2 = sensor_values[11]
            sensor_send.pump = sensor_values[12]
            print(sensor_send)
            rospy.logdebug("Received data from STM32: {}".format(sensor_values))
            if sensor_send.adc_current == 54:
                rospy.logerr("ADC current received error code 5400")
            if sensor_send.I2c_current_1 == 51:
                rospy.logerr("I2C current sensor 1 received error code 5100")
            if sensor_send.I2c_current_2 == 51:
                rospy.logerr("I2C current sensor 2 received error code 5100")
            if sensor_send.I2c_current_3 == 51:
                rospy.logerr("I2C current sensor 3 received error code 5100")
            if sensor_send.I2c_current_1 == 51.1:
                rospy.logerr("I2C current sensor 1 received error code 5110")
            if sensor_send.I2c_current_2 == 51.1:
                rospy.logerr("I2C current sensor 2 received error code 5110")
            if sensor_send.I2c_current_3 == 51.1:
                rospy.logerr("I2C current sensor 3 received error code 5110")
            if sensor_send.Battery == 52.1:
                rospy.logerr("Battery sensor received error code 5210")
        else:
            rospy.logerr("STM32 read timeout: did not receive any data under a period of {}s".format(timeout))
        stm32_sensors.publish(sensor_send)
        rate.sleep()
    closeSTM32Serial()
