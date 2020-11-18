#!/usr/bin/env python

import rospy
import std_msgs.msg
from wind_sensor.src.wind_sensor.wind_sensor import WindSensor


def wind_sensor_talker():
    pub = rospy.Publisher("wind_sensor/wind_angle", std_msgs.msg.String, queue_size=10)
    rospy.init_node("wind_sensor_talker", anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        wind_angle = WindSensor()
        #rospy.loginfo()
        pub.publish(wind_angle)
        rate.sleep()


if __name__ == '__main__':
    try:
        wind_sensor_talker()
    except rospy.ROSInterruptException:
        pass