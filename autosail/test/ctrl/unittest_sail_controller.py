#!/usr/bin/env python

import unittest
import rospy
import dynamic_reconfigure.client
import math
from scipy.spatial.transform import Rotation

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped


def get_imu_msg(roll=0, yaw=0):
    imu_msg = Imu()
    rot = Rotation.from_euler('xz', [roll, yaw], degrees=True)
    rot_quat = rot.as_quat()
    imu_msg.orientation.x = rot_quat[0]
    imu_msg.orientation.y = rot_quat[1]
    imu_msg.orientation.z = rot_quat[2]
    imu_msg.orientation.w = rot_quat[3]

    return imu_msg


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
        queue_size = 1

        # test publishers
        self.imu_pub = rospy.Publisher(name="/imu/data", data_class=Imu, queue_size=queue_size)
        self.wind_pub = rospy.Publisher(name='/wind_sensor/wind_vector', data_class=Vector3Stamped, queue_size=queue_size)

        # test subscribers
        self.angle_sub = rospy.Subscriber(name="sail_controller/sail_angle", data_class=Float64,
                                          callback=callback_sail_angle, queue_size=queue_size)
        self.servo_sub = rospy.Subscriber(name="sail_controller/sail_servo_angle", data_class=Float64,
                                          callback=callback_sail_servo, queue_size=queue_size)

        # Initiate client to change the dynamic reconfiguration
        client = dynamic_reconfigure.client.Client("sail_controller", timeout=30)

        # Test configuration
        sail_limit = 80
        kp = 1
        ki = 0
        kd = 0
        enable_auto_tilt = True
        min_roll = 10
        max_roll = 30

        client.update_configuration({"sail_limit": sail_limit, "kp": kp, "ki": ki, "kd": kd,
                                     "enable_auto_tilt": enable_auto_tilt, "min_roll": min_roll, "max_roll": max_roll})

        # sleep to have time to initialize
        rospy.sleep(1)

    def test_number_of_connections(self):
        self.assertEqual(self.imu_pub.get_num_connections(), 1)
        self.assertEqual(self.wind_pub.get_num_connections(), 1)
        self.assertEqual(self.angle_sub.get_num_connections(), 1)
        self.assertEqual(self.servo_sub.get_num_connections(), 1)

    def test_sail_angle_wind(self):
        global sail_servo, sail_angle
        max_sail = math.radians(80)
        max_servo = 1620

        imu_msg = get_imu_msg()

        # Test wind from behind and to the side
        wind_msg = Vector3Stamped()
        wind_msg.vector.x = 1
        wind_msg.vector.y = 1
        sail_servo = None
        while True:
            self.imu_pub.publish(imu_msg)
            self.wind_pub.publish(wind_msg)
            if sail_servo is not None or rospy.is_shutdown():
                break
        rospy.sleep(2)
        self.assertEqual(sail_servo, 0)
        self.assertEqual(sail_angle, -max_sail)

        # Test wind from side
        wind_msg.vector.x = 0
        wind_msg.vector.y = 1
        sail_servo = None
        while True:
            self.imu_pub.publish(imu_msg)
            self.wind_pub.publish(wind_msg)
            if sail_servo is not None or rospy.is_shutdown():
                break
        rospy.sleep(2)
        self.assertEqual(sail_servo, 708.75)
        self.assertEqual(sail_angle, -math.pi/4)

        # Test wind from ahead and side
        wind_msg.vector.x = -1
        wind_msg.vector.y = 1
        sail_servo = None
        while True:
            self.imu_pub.publish(imu_msg)
            self.wind_pub.publish(wind_msg)
            if sail_servo is not None or rospy.is_shutdown():
                break
        rospy.sleep(2)
        self.assertEqual(sail_servo, max_servo)
        self.assertEqual(sail_angle, 0)

        # Test wind from ahead and other side
        wind_msg.vector.x = -1
        wind_msg.vector.y = -1
        sail_servo = None
        while True:
            self.imu_pub.publish(imu_msg)
            self.wind_pub.publish(wind_msg)
            if sail_servo is not None or rospy.is_shutdown():
                break
        rospy.sleep(2)
        self.assertEqual(sail_servo, max_servo)
        self.assertEqual(sail_angle, 0)

        # Test wind from other side
        wind_msg.vector.x = 0
        wind_msg.vector.y = -1
        while True:
            self.imu_pub.publish(imu_msg)
            self.wind_pub.publish(wind_msg)
            if sail_servo is not None or rospy.is_shutdown():
                break
        rospy.sleep(2)
        self.assertEqual(sail_servo, 708.75)
        self.assertEqual(sail_angle, math.pi/4)

        # Test wind from behind and other side
        wind_msg.vector.x = 1
        wind_msg.vector.y = -1
        sail_servo = None
        while True:
            self.imu_pub.publish(imu_msg)
            self.wind_pub.publish(wind_msg)
            if sail_servo is not None or rospy.is_shutdown():
                break
        rospy.sleep(2)
        self.assertAlmostEqual(sail_servo, 0)
        self.assertAlmostEqual(sail_angle, max_sail)

    def test_sail_angle_roll(self):
        global sail_servo, sail_angle
        max_sail = math.radians(80)
        max_servo = 1620

        # Wind to place sail in 0 position
        wind_msg = Vector3Stamped()
        wind_msg.vector.x = -1
        wind_msg.vector.y = -1

        # Test 10 deg roll
        imu_msg = get_imu_msg(10)
        sail_servo = None
        while True:
            self.imu_pub.publish(imu_msg)
            self.wind_pub.publish(wind_msg)
            if sail_servo is not None or rospy.is_shutdown():
                break
        rospy.sleep(2)
        self.assertAlmostEqual(sail_servo, max_servo)
        self.assertAlmostEqual(sail_angle, 0)

        # Test 20 deg roll
        imu_msg = get_imu_msg(20)
        sail_servo = None
        while True:
            self.imu_pub.publish(imu_msg)
            self.wind_pub.publish(wind_msg)
            if sail_servo is not None or rospy.is_shutdown():
                break
        rospy.sleep(2)
        self.assertAlmostEqual(sail_servo, 1 / 2 * max_servo)
        self.assertAlmostEqual(sail_angle, max_sail/2)

        # Test 30 deg roll
        imu_msg = get_imu_msg(30)
        sail_servo = None
        while True:
            self.imu_pub.publish(imu_msg)
            self.wind_pub.publish(wind_msg)
            if sail_servo is not None or rospy.is_shutdown():
                break
        rospy.sleep(2)
        self.assertAlmostEqual(sail_servo, 0)
        self.assertAlmostEqual(sail_angle, max_sail)

        # Test -30 deg roll
        imu_msg = get_imu_msg(-30)
        sail_servo = None
        while True:
            self.imu_pub.publish(imu_msg)
            self.wind_pub.publish(wind_msg)
            if sail_servo is not None or rospy.is_shutdown():
                break
        rospy.sleep(2)
        self.assertAlmostEqual(sail_servo, 0)
        self.assertAlmostEqual(sail_angle, -max_sail)

        # Test -20 deg roll
        imu_msg = get_imu_msg(-20)
        sail_servo = None
        while True:
            self.imu_pub.publish(imu_msg)
            self.wind_pub.publish(wind_msg)
            if sail_servo is not None or rospy.is_shutdown():
                break
        rospy.sleep(2)
        self.assertAlmostEqual(sail_servo, 1 / 2 * max_servo)
        self.assertAlmostEqual(sail_angle, 1 / 2 * -max_sail)

        # Test -10 deg roll
        imu_msg = get_imu_msg(-10)
        sail_servo = None
        while True:
            self.imu_pub.publish(imu_msg)
            self.wind_pub.publish(wind_msg)
            if sail_servo is not None or rospy.is_shutdown():
                break
        rospy.sleep(2)
        self.assertAlmostEqual(sail_servo, max_servo)
        self.assertAlmostEqual(sail_angle, 0)

    def test_sail_wind_and_roll(self):
        global sail_servo, sail_angle
        max_sail = math.radians(80)
        max_servo = 1620

        wind_msg = Vector3Stamped()
        wind_msg.vector.x = 0
        wind_msg.vector.y = -1

        # Test 15 deg roll
        imu_msg = get_imu_msg(15)
        sail_servo = None
        while True:
            self.imu_pub.publish(imu_msg)
            self.wind_pub.publish(wind_msg)
            if sail_servo is not None or rospy.is_shutdown():
                break
        rospy.sleep(2)
        self.assertAlmostEqual(sail_servo, 303.75)
        self.assertAlmostEqual(sail_angle, math.pi/4+max_sail/4)

        wind_msg = Vector3Stamped()
        wind_msg.vector.x = 0
        wind_msg.vector.y = 1

        # Test -15 deg roll
        imu_msg = get_imu_msg(-15)
        sail_servo = None
        while True:
            self.imu_pub.publish(imu_msg)
            self.wind_pub.publish(wind_msg)
            if sail_servo is not None or rospy.is_shutdown():
                break
        rospy.sleep(2)
        self.assertAlmostEqual(sail_servo, 303.75)
        self.assertAlmostEqual(sail_angle, -math.pi/4-max_sail/4)


if __name__ == "__main__":
    import rostest

    rospy.init_node("unittest_sail")
    rostest.rosrun("autosail", "libtest_rudder", TestSail)
