#!/usr/bin/env python

import unittest
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped


sail_servo = 0

def callback_sail_servo(data):
    global sail_servo
    sail_servo = data.data


class TestSail(unittest.TestCase):
    def setUp(self):
        rospy.init_node("unittest_sail")
        queue_size = rospy.get_param("~queue_size", 1)

        # test publishers
        self.course_pub = rospy.Publisher(name="/path_planner/course", data_class=Float64, queue_size=queue_size)
        self.navheading_pub = rospy.Publisher(name="/imu/data", data_class=Imu, queue_size=queue_size)
        self.velocity_pub = rospy.Publisher(name="/gps/fix_velocity", data_class=TwistWithCovarianceStamped,
                                            queue_size=queue_size)

        # test subscribers
        rospy.Subscriber(name="sail_controller/sail_servo_angle", data_class=Float64,
                         callback=callback_sail_servo, queue_size=queue_size)


if __name__ == "__main__":
    import rostest
    rostest.rosrun("autosail", "libtest_rudder", TestRudder)