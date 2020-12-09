#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import Float64

def temperaturePublisher():
    if os.uname()[4][:3] == 'x86':
        filename = '/sys/devices/virtual/thermal/thermal_zone0/temp'
        pub = rospy.Publisher('/temperature/computer', Float64, queue_size=10)
    else:
        filename = '/sys/devices/virtual/thermal/thermal_zone7/temp'
        if os.uname()[1] == 'vision-sbc':
            pub = rospy.Publisher('/temperature/vision_sbc', Float64, queue_size=10)
        else:
            pub = rospy.Publisher('/temperature/main_sbc', Float64, queue_size=10)

    rospy.init_node('temperature', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        with open(filename) as f:
            read_data = f.read()
        if f.closed != True:
            rospy.logerr("File not closed correctly!")
        rospy.loginfo(read_data)
        if len(read_data) <= 6:
            temperature = read_data[0:2] + "." + read_data[2:-2]
        else:
            temperature = read_data[0:3] + "." + read_data[3:-2]
        pub.publish(float(temperature))
        rate.sleep()

if __name__ == '__main__':
    try:
        temperaturePublisher()
    except rospy.ROSInterruptException:
        pass