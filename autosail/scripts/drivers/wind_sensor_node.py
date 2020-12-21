#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
from wind_sensor.wind_sensor import WindSensor
import subprocess as sp


class WindSensorTalker:
    def __init__(self):
        """
        Initializes the WindSensor class that is defined in wind_sensor.py
        """
        self.wnd = WindSensor()

    def talker(self):
        """
        Initializes the 4 publishers(wind_pub, rpy_pub, temp_pub, battery_pub), then initializes the node
        as wind_sensor_node and sets the rate to the users defined value(8 is the max of the wind sensor) then sleeps to
        let the startup and bluetooth connection finish.

        Makes function calls to gather the sensor readings from the wind sensor and saves the to the correct formats for
        publishing with a check at the end that the bluetooth connectivity is still up and if it is not the connection
        is reset and a small sleep to wait for it to establish before continuing.
        """
        wind_pub = rospy.Publisher('wind_sensor/wind_vector', Vector3Stamped, queue_size=10)
        rpy_pub = rospy.Publisher('wind_sensor/roll_pitch_yaw', Vector3Stamped, queue_size=10)
        temp_pub = rospy.Publisher('wind_sensor/temperature', Float64, queue_size=10)
        battery_pub = rospy.Publisher('wind_sensor/battery_voltage', Float64, queue_size=10)
        rospy.init_node('wind_sensor_node', anonymous=True, log_level=rospy.get_param("log_level", rospy.INFO))
        rate = rospy.Rate(8)  # refresh rate in hz
        rospy.sleep(5)
        while not rospy.is_shutdown():
            self.wnd.update()
            wind_vector = self.wnd.get_wind_vector()
            vec_msg = Vector3Stamped()
            vec_msg.header.stamp = rospy.Time.now()
            vec_msg.vector.x = -wind_vector[0]
            vec_msg.vector.y = -wind_vector[1]
            vec_msg.vector.z = 0

            rpy_vector = self.wnd.get_rpy()
            rpy_msg = Vector3Stamped()
            rpy_msg.header.stamp = rospy.Time.now()
            rpy_msg.vector.x = -rpy_vector[0]
            rpy_msg.vector.y = rpy_vector[1]
            rpy_msg.vector.z = -rpy_vector[2]

            battery_msg = Float64()
            battery_msg.data = self.wnd.get_battery_charge()

            temp_msg = Float64()
            temp_msg.data = self.wnd.get_temp()
            temp_msg -= 273.15  # convert to celsius from kelvin
            stdoutdata = sp.getoutput("hcitool con")
            if "DC:73:74:12:94:80" not in stdoutdata.split():
                rospy.logerr("Connection Failed, Reconnecting!")
                self.wnd.close()
                self.wnd = WindSensor()
                rospy.sleep(5)
            wind_pub.publish(vec_msg)
            rpy_pub.publish(rpy_msg)
            battery_pub.publish(battery_msg)
            temp_pub.publish(temp_msg)
            rate.sleep()


if __name__ == '__main__':
    wnd = WindSensorTalker()
    try:
        wnd.talker()
    except rospy.ROSInterruptException:
        pass
