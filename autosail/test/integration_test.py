#!/usr/bin/env python
import time
import math
import rospy
import rostest
from unittest import TestCase
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu, NavSatFix
from marti_nav_msgs.msg import Route, RoutePoint
from marti_common_msgs.msg import KeyValue
from scipy.spatial.transform import Rotation
from utils.polls import poll_connections


class IntegrationTest(TestCase):

    def setUp(self):
        rospy.init_node("test_node", anonymous=True)

        # Setup publishers
        self.pub_waypoints = rospy.Publisher('path_planner/waypoints', Route, queue_size=10)
        self.pub_obstacle = rospy.Publisher('/camera/data', Vector3Stamped, queue_size=10)
        self.pub_wind_sensor = rospy.Publisher('/wind_sensor/wind_vector', Vector3Stamped, queue_size=10)
        self.pub_imu = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.pub_velocity = rospy.Publisher('gps/fix_velocity', TwistWithCovarianceStamped, queue_size=10)
        self.pub_position = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)

        timeout = 60
        rospy.loginfo("Polling publish connections")
        self.assertEqual(poll_connections(self.pub_waypoints, 1, timeout), 1,
                         "Polling waypoints connection timed out")
        self.assertEqual(poll_connections(self.pub_obstacle, 1, timeout), 1,
                         "Polling obstacle connection timed out")
        self.assertEqual(poll_connections(self.pub_wind_sensor, 2, timeout), 2,
                         "Polling wind_vector connection timed out")
        self.assertEqual(poll_connections(self.pub_imu, 4, timeout), 4,
                         "Polling imu connection timed out")
        self.assertEqual(poll_connections(self.pub_velocity, 3, timeout), 3,
                         "Polling fix_velocity timed out")
        self.assertEqual(poll_connections(self.pub_position, 1, timeout), 1,
                         "Polling fix timed out")

        # Setup subscribers
        sub_desired_course = rospy.Subscriber(name="/path_planner/course", data_class=Float64,
                                              callback=self.callback_desired_course, queue_size=1)
        sub_rudder_angle = rospy.Subscriber(name="/rudder_controller/rudder_angle", data_class=Float64,
                                            callback=self.callback_rudder_angle, queue_size=1)
        sub_sail_servo_angle = rospy.Subscriber(name="sail_controller/sail_servo_angle", data_class=Float64,
                                                callback=self.callback_sail, queue_size=1)

        rospy.loginfo("Polling subscriber connections")
        self.assertEqual(poll_connections(sub_desired_course, 1, timeout), 1,
                         "Polling desired_course connection timed out")
        self.assertEqual(poll_connections(sub_rudder_angle, 1, timeout), 1,
                         "Polling rudder_angle connection timed out")
        self.assertEqual(poll_connections(sub_sail_servo_angle, 1, timeout), 1,
                         "Polling sail_servo_angle connection timed out")

        self.output_desired_course = None
        self.output_rudder_angle = None
        self.output_sail_servo_angle = None

        self.publish_fake_msgs()

    def tearDown(self):
        pass

    def publish_fake_msgs(self):
        # waypoints
        waypoints_msgs = Route()
        waypoint = RoutePoint()  # 90 deg
        waypoint.pose.position.x = 16.561736302687205
        waypoint.pose.position.y = 59.61744366137741
        waypoint.id = "0"  # 59.61744366137741, 16.561736302687205
        prop = KeyValue()
        prop.key = "diameter"
        prop.value = "5"
        waypoint.properties.append(prop)
        prop = KeyValue()
        prop.key = "id"
        prop.value = "0"
        waypoint.properties.append(prop)
        waypoints_msgs.route_points.append(waypoint)
        # obstacle
        obstacle_msg = Vector3Stamped()
        obstacle_msg.vector.x = 30
        obstacle_msg.vector.y = -30
        obstacle_msg.header.stamp = rospy.Time.now()
        # wind sensor
        wind_sensor_msg = Vector3Stamped()
        wind_sensor_msg.vector.x = 7
        wind_sensor_msg.vector.y = 7
        wind_sensor_msg.vector.z = 0
        # imu
        imu_msg = Imu()
        rot = Rotation.from_euler('xyz', [0, 0, 0], degrees=True)
        rot_quat = rot.as_quat()
        imu_msg.orientation.x = rot_quat[0]
        imu_msg.orientation.y = rot_quat[1]
        imu_msg.orientation.z = rot_quat[2]
        imu_msg.orientation.w = rot_quat[3]
        # gps velocity
        gps_velocity_msg = TwistWithCovarianceStamped()
        gps_velocity_msg.twist.twist.linear.x = 1
        gps_velocity_msg.twist.twist.linear.y = 0
        gps_velocity_msg.twist.twist.linear.z = 0
        # gps position
        gps_position_msg = NavSatFix()
        gps_position_msg.longitude = 16.560838629596216
        gps_position_msg.latitude = 59.617458491079226

        # Expected output
        self.expected_desired_course = math.pi / 2
        self.expected_rudder_angle = math.pi / 4
        self.expected_sail_servo_angle = 0

        self.pub_waypoints.publish(waypoints_msgs)
        self.pub_obstacle.publish(obstacle_msg)
        self.pub_wind_sensor.publish(wind_sensor_msg)
        self.pub_imu.publish(imu_msg)
        self.pub_velocity.publish(gps_velocity_msg)
        self.pub_position.publish(gps_position_msg)

    def callback_desired_course(self, data):
        self.output_desired_course = data.data

    def callback_rudder_angle(self, data):
        self.output_rudder_angle = data.data

    def callback_sail(self, data):
        self.output_sail_servo_angle = data.data

    def test_path_planner(self):
        poll_rate = rospy.Rate(100)
        timeout = time.time() + 10
        while self.output_desired_course is None:
            self.assertLess(time.time(), timeout)
            poll_rate.sleep()
        self.assertLess(abs(self.output_desired_course - self.expected_desired_course), math.radians(5))

    def test_rudder_controller(self):
        poll_rate = rospy.Rate(100)
        timeout = time.time() + 10
        while self.output_rudder_angle is None:
            self.assertLess(time.time(), timeout)
            poll_rate.sleep()
        self.assertLess(abs(self.output_rudder_angle - self.expected_rudder_angle), math.radians(5))

    def test_sail_controller(self):
        poll_rate = rospy.Rate(100)
        timeout = time.time() + 10
        while self.output_desired_course is None:
            self.assertLess(time.time(), timeout)
            poll_rate.sleep()
        self.assertLess(abs(self.output_sail_servo_angle - self.expected_sail_servo_angle), math.radians(5))


if __name__ == "__main__":
    rostest.unitrun("autosail", "integration_test", IntegrationTest)
