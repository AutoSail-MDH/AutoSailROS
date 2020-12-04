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
        rospy.init_node("unittest_rudder")
        queue_size = rospy.get_param("~queue_size", 1)

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

    def test_rudder_angle(self):
        global rudder_angle

        course_msg = Float64()
        course_msg.data = math.pi
        self.course_pub.publish(course_msg)

        imu_msg = Imu()
        rot = Rotation.from_euler('xyz', [90, 0, 0], degrees=True)
        rot_quat = rot.as_quat()
        imu_msg.orientation.x = rot_quat[0]
        imu_msg.orientation.y = rot_quat[1]
        imu_msg.orientation.z = rot_quat[2]
        imu_msg.orientation.w = rot_quat[3]
        self.imu_pub.publish(imu_msg)

        velocity_msg = TwistWithCovarianceStamped()
        velocity_msg.twist.twist.linear.x = 1
        velocity_msg.twist.twist.linear.y = 0
        self.velocity_pub.publish(velocity_msg)

        rospy.sleep(1/100)

        self.assertEqual(rudder_angle, 0)

    def tearDown(self):
        rospy.signal_shutdown("test ended")


if __name__ == "__main__":
    import rostest

    rostest.rosrun("autosail", "libtest_rudder", TestRudder)
