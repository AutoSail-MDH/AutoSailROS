#!/usr/bin/env python

import unittest
import rospy
import math
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped

rudder_angle = None


def callback_rudder_angle(data):
    global rudder_angle
    rudder_angle = data.data


class TestRudder(unittest.TestCase):
    def setUp(self):
        queue_size = rospy.get_param("~queue_size", 1)
        self.rate = rospy.Rate(1)

        # test publishers
        self.course_pub = rospy.Publisher(name="/path_planner/course", data_class=Float64, queue_size=queue_size)
        self.imu_pub = rospy.Publisher(name="/imu/data", data_class=Imu, queue_size=queue_size)
        self.velocity_pub = rospy.Publisher(name="/gps/fix_velocity", data_class=TwistWithCovarianceStamped,
                                            queue_size=queue_size)

        # test subscribers
        self.rudder_sub = rospy.Subscriber(name="/rudder_controller/rudder_angle", data_class=Float64,
                                           callback=callback_rudder_angle, queue_size=queue_size)

        # sleep to have time to initialize
        rospy.sleep(1)

    def test_number_of_connections(self):
        self.assertEqual(self.course_pub.get_num_connections(), 1)
        self.assertEqual(self.imu_pub.get_num_connections(), 1)
        self.assertEqual(self.velocity_pub.get_num_connections(), 1)
        self.assertEqual(self.rudder_sub.get_num_connections(), 1)

    def test_rudder_angle_max_left(self):
        global rudder_angle

        course_msg = Float64()
        course_msg.data = math.pi

        imu_msg = Imu()
        rot = Rotation.from_euler('z', 90, degrees=True)
        rot_quat = rot.as_quat()
        imu_msg.orientation.x = rot_quat[0]
        imu_msg.orientation.y = rot_quat[1]
        imu_msg.orientation.z = rot_quat[2]
        imu_msg.orientation.w = rot_quat[3]

        velocity_msg = TwistWithCovarianceStamped()
        velocity_msg.twist.twist.linear.x = 0.1
        velocity_msg.twist.twist.linear.y = 0

        while True:
            self.course_pub.publish(course_msg)
            self.imu_pub.publish(imu_msg)
            self.velocity_pub.publish(velocity_msg)
            if rudder_angle is not None or rospy.is_shutdown():
                break
        self.assertEqual(rudder_angle, math.pi/4)

    def test_rudder_angle_max_right(self):
        global rudder_angle

        course_msg = Float64()
        course_msg.data = math.pi

        imu_msg = Imu()
        rot = Rotation.from_euler('z', -90, degrees=True)
        rot_quat = rot.as_quat()
        imu_msg.orientation.x = rot_quat[0]
        imu_msg.orientation.y = rot_quat[1]
        imu_msg.orientation.z = rot_quat[2]
        imu_msg.orientation.w = rot_quat[3]

        velocity_msg = TwistWithCovarianceStamped()
        velocity_msg.twist.twist.linear.x = 0.1
        velocity_msg.twist.twist.linear.y = 0

        rudder_angle = None
        while True:
            self.course_pub.publish(course_msg)
            self.imu_pub.publish(imu_msg)
            self.velocity_pub.publish(velocity_msg)
            if rudder_angle is not None or rospy.is_shutdown():
                break
        self.assertEqual(rudder_angle, math.pi/4)


if __name__ == "__main__":
    import rostest

    rospy.init_node("unittest_rudder")
    rostest.rosrun("autosail", "libtest_rudder", TestRudder)
