#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
from wind_sensor.wind_sensor import WindSensor


class WindSensorTalker:
    def __init__(self):
        self.wnd = WindSensor()
        pass

    def talker(self):
        wind_pub = rospy.Publisher('wind_sensor/wind_vector', Vector3Stamped, queue_size=10)
        rpy_pub = rospy.Publisher('wind_sensor/roll_pitch_yaw', Vector3Stamped, queue_size=10)
        temp_pub = rospy.Publisher('wind_sensor/temperature', Float64, queue_size=10)
        battery_pub = rospy.Publisher('wind_sensor/battery_voltage', Float64, queue_size=10)
        rospy.init_node('wind_sensor_node', anonymous=True)
        rate = rospy.Rate(8)  # refresh rate in hz
        while not rospy.is_shutdown():
            self.wnd.update()
            wind_vector = self.wnd.get_wind_vector()
            vec_msg = Vector3Stamped()
            vec_msg.header.stamp = rospy.Time.now()
            vec_msg.vector.x = wind_vector[0]
            vec_msg.vector.y = wind_vector[1]
            vec_msg.vector.z = 0

            rpy_vector = self.wnd.get_rpy()
            rpy_msg = Vector3Stamped()
            rpy_msg.header.stamp = rospy.Time.now()
            rpy_msg.vector.x = rpy_vector[0]
            rpy_msg.vector.y = rpy_vector[1]
            rpy_msg.vector.z = rpy_vector[2]

            battery_msg = Float64()
            battery_msg = self.wnd.get_battery_charge()

            temp_msg = Float64()
            temp_msg = self.wnd.get_temp()
            temp_msg -= 273.15 # convert to celsius from kelvin

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
"""
class WindSensorTalker:
    def __init__(self):
        self.wnds = WindSensor()

    def talker_callback(self):
        wind_angle = WindSensor.read_sensor(self.wnds)
        pub.publish(wind_angle)
        print(wind_angle)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node("wind_sensor_talker", anonymous=True)
    wnds = WindSensorTalker()
    pub = rospy.Publisher("wind_sensor/wind_angle", std_msgs.msg.String, queue_size=10)
    rate = rospy.Rate(1)
    wnds.talker_callback()
    rospy.spin()
"""