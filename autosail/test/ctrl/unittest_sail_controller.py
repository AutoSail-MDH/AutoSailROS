#!/usr/bin/env python

import unittest
import rospy
import math
from scipy.spatial.transform import Rotation

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

sail_servo = None
sail_angle = None


def callback_sail_servo(data):
    global sail_servo
    sail_servo = data.data


def callback_sail_angle(data):
    global sail_angle
    sail_angle = data.data


class TestSail(unittest.TestCase):
    def setUp(self):
        rospy.init_node("unittest_sail")
        queue_size = rospy.get_param("~queue_size", 1)

        # test publishers
        self.imu_pub = rospy.Publisher(name="/imu/data", data_class=Imu, queue_size=queue_size)
        self.wind_pub = rospy.Publisher(name='/wind/apparent', data_class=Vector3Stamped, queue_size=queue_size)

        # test subscribers
        self.angle_sub = rospy.Subscriber(name="sail_controller/sail_angle", data_class=Float64,
                                          callback=callback_sail_angle, queue_size=queue_size)
        self.servo_sub = rospy.Subscriber(name="sail_controller/sail_servo_angle", data_class=Float64,
                                          callback=callback_sail_servo, queue_size=queue_size)

    def test_number_of_connections(self):
        self.assertEqual(self.imu_pub.get_num_connections(), 1)
        self.assertEqual(self.wind_pub.get_num_connections(), 1)
        self.assertEqual(self.angle_sub.get_num_connections(), 1)
        self.assertEqual(self.servo_sub.get_num_connections(), 1)

    def test_sail_angle(self):
        global sail_servo, sail_angle
        max_sail = math.radians(rospy.get_param("~sail_limits", 80))

        imu_msg = Imu()
        rot = Rotation.from_euler('xyz', [90, 0, 0], degrees=True)
        rot_quat = rot.as_quat()
        imu_msg.orientation.x = rot_quat[0]
        imu_msg.orientation.y = rot_quat[1]
        imu_msg.orientation.z = rot_quat[2]
        imu_msg.orientation.w = rot_quat[3]
        self.imu_pub.publish(imu_msg)

        wind_msg = Vector3Stamped()
        wind_msg.vector.x = 0
        wind_msg.vector.y = 1
        self.wind_pub.publish(wind_msg)

        self.assertEqual(sail_servo, 810)
        self.assertEqual(sail_angle, max_sail / 2)

    def tearDown(self):
        rospy.signal_shutdown("test ended")


if __name__ == "__main__":
    import rostest

    rostest.rosrun("autosail", "libtest_rudder", TestSail)
