#!/usr/bin/env python
import time
import odroid_wiringpi as wpi
import rospy
from std_msgs.msg import Float32

if __name__ == '__main__':
    wpi.wiringPiSetup()
    # ROS setup
    pub = rospy.Publisher('voltage', Float32)
    rospy.init_node('battery_voltage_indicator', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        battery = wpi.analogRead(1)  # Read ADC.AIN1 (physical port #37)
        pub.publish((battery * (1.8/1024))*11.0034)  # Convert bit to actual voltage
        rate.sleep()
