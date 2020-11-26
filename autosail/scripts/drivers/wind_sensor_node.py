#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped
from wind_sensor.wind_sensor import WindSensor

class WindSensorTalker:
    def __init__(self):
        self.wnd = WindSensor()
        pass

    def talker(self):
        pub = rospy.Publisher('wind_sensor', Vector3Stamped, queue_size=10)
        rospy.init_node('wind_sensor_node', anonymous=True)
        rate = rospy.Rate(8)  # 1hz
        while not rospy.is_shutdown():
            wind_vector = self.wnd.read_sensor()
            rospy.loginfo(wind_vector)
            vec_msg = Vector3Stamped()
            vec_msg.header.stamp = rospy.Time.now()
            vec_msg.vector.x = wind_vector[0]
            vec_msg.vector.y = wind_vector[1]
            vec_msg.vector.z = 0
            pub.publish(vec_msg)
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