#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
import message_filters
from pymap3d.ned import geodetic2ned
from marti_nav_msgs.msg import RoutePoint, Route
from scipy.spatial import distance
from path_planner.potential_field_algorithm import PotentialField
from path_planner.path_planner import *

from path_planner.msg import obstacles_array_msg

# global variables updated by the callback functions via the subscribers
waypoints = []
obstacles = []
lat0 = None
lon0 = None
current_position = []
velocity = None
heading = []  # np.array([0., 0.])
heading_yaw = None
w_speed = None
w_theta = None
lin_velocity = []

radius = 6371  # Earth Radius in KM

class Config:
    def __init__(self):
        self.profile_diameter = rospy.get_param("~profile_diameter")
        self.goal_weight = rospy.get_param("~goal_weight")
        self.obstacle_weight = rospy.get_param("~obstacle_weight")
        self.d_inf = rospy.get_param("~d_inf")
        self.p_ngz = rospy.get_param("~p_ngz")
        self.p_hyst = rospy.get_param("~p_hyst")
        self.g_v = rospy.get_param("~g_v")
        self.local_coord_scale = rospy.get_param("~local_coord_scale")
        self.waypoint_radius = rospy.get_param("~waypoint_radius")
        self.num_circle_point = rospy.get_param("~num_circle_point")
        self.default_waypoint_id = rospy.get_param("~default_waypoint_id")
        self.waypoints_to_circle_id = rospy.get_param("~waypoints_to_circle_id")
        self.waypoints_in_circle_id = rospy.get_param("~waypoints_in_circle_id")
        self.circle_time_limit = rospy.get_param("~circle_time_limit")


def waypoint_callback(data):
    """
    reads the "path_planner/waypoints" topic and saves the waypoints as a global np.array
    :param data: td_msgs.msg.Float64MultiArray with waypoint latitude, longitude
    """
    global waypoints, waypoint_index, lon0, lat0
    waypoints = data.route_points
    waypoint_index = 0


def wind_sensor_callback(data):
    """
    reads the /wind_sensor topic and saves the wind speed and wind angle as global variables
    :param data: std_msgs.msg.Int64MultiArray with wind speed and wind angle
    """
    global w_speed, w_theta, lin_velocity, heading_yaw
    if lin_velocity and heading_yaw is not None:
        true_wind_ = [data.vector.x * 1.94384449 + lin_velocity[0], data.vector.y * 1.94384449 + lin_velocity[1]]
        w_theta = np.arctan2(true_wind_[1], true_wind_[0]) + np.pi
        w_speed = np.linalg.norm(true_wind_)
        w_theta = w_theta + heading_yaw + np.pi
        w_theta = math.atan2(math.sin(w_theta), math.cos(w_theta)) + math.pi/2
        #rospy.loginfo("True wind: {}".format(math.degrees(w_theta)))


def gps_position_callback(data):
    """
    reads the gps/fix topic and saves the latitude, longitude for the vessel as global variables
    :param data: sensor_msgs.msg.NavSatFix with latitude. longitude
    """
    global current_position
    current_position = data


def gps_velocity_callback(data):
    """
    reads the gps/fix_velocity topic and saves the linear velocity as a global variable
    :param data: geometry_msgs.msg.TwistWithCovarianceStamped with linear x, y velocity
    """
    global velocity, lin_velocity
    lin_velocity_x = data.twist.twist.linear.x * 1.94384449
    lin_velocity_y = - data.twist.twist.linear.y * 1.94384449
    lin_velocity = [lin_velocity_x, lin_velocity_y]
    velocity = math.sqrt((lin_velocity_x ** 2) + (lin_velocity_y ** 2))


def imu_heading_callback(data):
    """
    reads the /filter/quaternion topic and saves the heading as a global variable. Is used for the real sensor
    :param data: geometry_msgs.msg.QuaternionStamped(
    :return: Nothing
    """
    global heading, heading_yaw
    heading_quaternion = Imu()
    heading_quaternion.orientation = data.orientation
    yaw = quaternion_to_euler_yaw(heading_quaternion.orientation)
    heading = [np.cos(yaw), np.sin(yaw)]
    heading_yaw = yaw


def obstacles_callback(data):
    """
    reads the /path_planner/obstacles topic and saves the obstacle as a global np.array
    :param data: std_msgs.msg.Float64MultiArray with latitude longitude for the obstacles
    """
    global obstacles
    obstacles = data.data


def path_planner_init():
    global waypoints, w_theta, current_position, heading, w_speed
    # create node for the path planner
    rospy.init_node('path_planner')
    # start the subscribers for all sensors
    rospy.Subscriber("path_planner/waypoints", Route, waypoint_callback, queue_size=1)
    rospy.Subscriber("/gps/fix", NavSatFix, gps_position_callback, queue_size=1)
    rospy.Subscriber("/gps/fix_velocity", TwistWithCovarianceStamped, gps_velocity_callback, queue_size=1)
    rospy.Subscriber("/imu/data", Imu, imu_heading_callback, queue_size=1)
    rospy.Subscriber("/wind_sensor", Vector3Stamped, wind_sensor_callback, queue_size=1)
    rospy.Subscriber("/path_planner/obstacles", obstacles_array_msg, obstacles_callback, queue_size=1)

    # waits until obstacles and position data has been read from the topics
    while not rospy.is_shutdown():
        # check if data has been read
        if waypoints and w_theta is not None and current_position and heading and w_speed is not None:
            break


if __name__ == '__main__':
    waypoint_index = 0
    # initialize subscribers

    # initialize publisher
    pub_heading = rospy.Publisher('/path_planner/course', Float64, queue_size=10)
    # read the parameters from the config.yaml file
    # Initialize
    path_planner_init()
    config = Config()
    pf = PotentialField(config.profile_diameter, config.obstacle_weight,
                        config.d_inf, config.goal_weight, config.p_ngz,
                        config.p_hyst, config.g_v)

    rate = rospy.Rate(1)
    # current loop
    while not rospy.is_shutdown():
        goal = geodetic2ned(waypoints[waypoint_index].pose.position.y, waypoints[waypoint_index].pose.position.x, 0,
                            current_position.latitude, current_position.longitude, 0)
        # if waypoint circle waypoint?
        # if goal[2] == config.waypoints_to_circle_id:
        #     start = time.time()
        #     elapsed = 0
        #     circle_waypoints = circle_waypoint(latitude, longitude, goal, p0, p1)
        #     goal_circle = circle_waypoints[0]
        #     # while time limit
        #     while elapsed < config.circle_time_limit and not rospy.is_shutdown():
        #         waypoint_xy = calc_waypoints(p0, p1, waypoints)
        #         pos_v_xy = latlng_to_screen_xy(latitude, longitude, p0, p1)
        #         if np.linalg.norm(np.subtract(pos_v_xy, goal[0:2])) < 5:  # set limit in meter
        #             if goal_circle == circle_waypoints[config.num_circle_point - 1]:
        #                 goal_circle = circle_waypoints[0]
        #             else:
        #                 goal_circle = circle_waypoints[circle_waypoints.index(goal_circle) + 1]
        #         min_angle = pf. \
        #             calc_heading(goal_circle, heading, w_speed, w_theta, pos_v_xy, obstacles,
        #                          velocity)
        #         # publish the calculated angle
        #         pub_heading.publish(min_angle)
        #         elapsed = time.time() - start
        #     if goal != waypoint_xy[-1]:
        #         waypoint_index += 1
        #         goal = waypoint_xy[waypoint_index]
        min_angle = pf.calc_heading(goal, heading, w_speed, w_theta, [0, 0], obstacles, velocity)
        min_angle = math.atan2(math.sin(min_angle), math.cos(min_angle))
        # publish the calculated angle
        pub_heading.publish(-min_angle)
        rospy.loginfo("Vessel position {}".format(current_position))
        rospy.loginfo("Goal position {}".format(goal))
        if np.linalg.norm(np.subtract(current_position, goal[0:2])) < 5:
            if goal != waypoints[-1]:
                waypoint_index += 1
                goal = waypoints[waypoint_index]
                rospy.logwarn("Next waypoint %d", waypoint_index)
            else:
                print("Path complete")
        rate.sleep()
