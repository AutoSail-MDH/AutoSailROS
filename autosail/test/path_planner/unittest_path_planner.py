#!/usr/bin/env python
import math
import numpy as np
from unittest import TestCase
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from marti_nav_msgs.msg import Route, RoutePoint
import rosunit


class TestPathPlanner(TestCase):

    def output_callback(self, msg):
        self.output = math.degrees(msg.data)

    def setUp(self):
        rospy.init_node("test_path_planner")
        self.pub_waypoints = rospy.Publisher('path_planner/waypoints', Route, queue_size=10)
        self.pub_gps_position = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
        self.pub_gps_velocity = rospy.Publisher('gps/fix_velocity', TwistWithCovarianceStamped, queue_size=10)
        self.pub_gps_heading = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.pub_wind_sensor = rospy.Publisher('/wind_sensor/true', Vector3Stamped, queue_size=10)
        self.pub_obstacles = rospy.Publisher('/camera/data', Vector3Stamped, queue_size=10)
        self.output = None
        self.sub = rospy.Subscriber('/path_planner/course', Float64, self.output_callback, queue_size=1)
        self.rate = rospy.Rate(1)

    def tearDown(self):
        rospy.signal_shutdown("Test ended")

    def test_input_output(self):
        # Waypoints
        waypoint_array = Route()
        waypoint = RoutePoint()
        waypoint.pose.position.y = 57.258346  # Latitude
        waypoint.pose.position.x = 2.927921  # Longitude
        waypoint.id = "0"
        waypoint_array.route_points.append(waypoint)
        waypoint = RoutePoint()
        waypoint.pose.position.y = 57.258346  # Latitude
        waypoint.pose.position.x = 2.937921  # Longitude
        waypoint.id = "0"
        waypoint_array.route_points.append(waypoint)

        # gps position
        fix = NavSatFix()
        fix.latitude = 57.248346
        fix.longitude = 2.927921
        # gps velocity
        velocity = TwistWithCovarianceStamped()
        velocity.twist.twist.linear.x = 0
        velocity.twist.twist.linear.y = 0
        # gps heading
        heading = Imu()
        heading.orientation.x = 0
        heading.orientation.y = 0
        # wind sensor geometry_msgs.msg.Vector3Stamped
        wind_data = Vector3Stamped()
        wind_data.vector.x = 3
        wind_data.vector.y = 3

        while self.output == None and not rospy.is_shutdown():
            self.pub_waypoints.publish(waypoint_array)
            self.pub_gps_position.publish(fix)
            self.pub_gps_velocity.publish(velocity)
            self.pub_gps_heading.publish(heading)
            self.pub_wind_sensor.publish(wind_data)
            self.rate.sleep()
        #self.pub_obstacles = rospy.Publisher('/path_planner/obstacles', Vector3Stamped, queue_size=10)

        #output = rospy.wait_for_message("/path_planner/course", Float64, 10).data
        rospy.sleep(2)
        self.assertLess(abs(self.output), 5)
        rospy.sleep(2)

        # Jump to the first waypoint and test if it changes
        fix.latitude = 57.258346
        fix.longitude = 2.927921
        self.pub_gps_position.publish(fix)
        rospy.sleep(2)
        self.assertLess(abs(90 - abs(self.output)), 5)

        obstacle_msg = Vector3Stamped()
        obstacle_msg.vector.x = 20
        obstacle_msg.vector.y = -20
        obstacle_msg.header.stamp = rospy.Time.now()
        self.pub_obstacles.publish(obstacle_msg)
        rospy.sleep(5)

        obstacle_msg.vector.x = 0
        obstacle_msg.vector.y = -5
        obstacle_msg.header.stamp = rospy.Time.now()
        self.pub_obstacles.publish(obstacle_msg)
        rospy.sleep(5)




if __name__ == "__main__":
    rosunit.unitrun("autosail", "path_planner", TestPathPlanner)