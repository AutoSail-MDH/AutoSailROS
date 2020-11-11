#!/usr/bin/env python
import rospy
import numpy as np
import math
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
from scipy.spatial import distance
from path_planner import potential_field_algorithm as pfa

from path_planner.msg import waypointmsg
from path_planner.msg import waypoint_array_msg
from path_planner.msg import obstaclemsg
from path_planner.msg import obstacles_array_msg

# global variables updated by the callback functions via the subscribers
waypoints = 0
obstacles = 0
longitude = 0.0
latitude = 0.0
lin_velocity = 0.0
heading = np.array([0., 0.])
w_speed = 0
w_theta = 0
waypoint_index_control = 0
obstacle_mutex = 0
waypoint_mutex = 0
velocity_mutex = 0
heading_mutex = 0
position_mutex = 0
wind_mutex = 0
lin_velocity_x = 0
lin_velocity_y = 0

radius = 6371  # Earth Radius in KM


class ReferencePoint:
    def __init__(self, scr_x, scr_y, lat, lng, pos_x, pos_y):
        self.scrX = scr_x
        self.scrY = scr_y
        self.lat = lat
        self.lng = lng
        self.pos_x = pos_x
        self.pos_y = pos_y


class ConfigClass:
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
        self.radius_earth = rospy.get_param("~radius_earth")


def waypoint_callback(data):
    """
    reads the "path_planner/waypoints" topic and saves the waypoints as a global np.array
    :param data: td_msgs.msg.Float64MultiArray with waypoint latitude, longitude
    :return: Nothing
    """
    global waypoints
    global waypoint_mutex
    waypoints = data.data
    waypoint_mutex = 1


def wind_sensor_callback(data):
    """
    reads the /wind_sensor topic and saves the wind speed and wind angle as global variables
    :param data: std_msgs.msg.Int64MultiArray with wind speed and wind angle
    :return: Nothing
    """
    global w_speed
    global w_theta
    global wind_mutex
    global velocity_mutex
    global lin_velocity_x
    global lin_velocity_y

    if velocity_mutex == 1:
        true_wind_ = [data.data[0] + lin_velocity_x, data.data[1] + lin_velocity_y]
        w_theta = np.arctan2(true_wind_[1], true_wind_[0]) + np.pi
        w_speed = np.linalg.norm(true_wind_)
        wind_mutex = 1


def gps_position_callback(data):
    """
    reads the gps/fix topic and saves the latitude, longitude for the vessel as global variables
    :param data: sensor_msgs.msg.NavSatFix with latitude. longitude
    :return: Nothing
    """
    global longitude
    global latitude
    global position_mutex
    longitude = data.longitude
    latitude = data.latitude
    position_mutex = 1


def gps_velocity_callback(data):
    """
    reads the gps/fix_velocity topic and saves the linear velocity as a global variable
    :param data: geometry_msgs.msg.TwistWithCovarianceStamped with linear x, y velocity
    :return: Nothing
    """
    global lin_velocity
    global lin_velocity_x
    global lin_velocity_y
    global velocity_mutex
    lin_velocity_x = data.twist.twist.linear.x
    lin_velocity_y = data.twist.twist.linear.y
    lin_velocity = math.sqrt((lin_velocity_x ** 2) + (lin_velocity_y ** 2))
    velocity_mutex = 1


def imu_heading_callback(data):
    """
    reads the /filter/quaternion topic and saves the heading as a global variable. Is used for the real sensor
    :param data: geometry_msgs.msg.QuaternionStamped(
    :return: Nothing
    """
    global heading
    global heading_mutex
    heading_quaternion = sensor_msgs.msg.Imu()
    heading_quaternion.orientation = data.orientation
    pitch, yaw = quaternion_to_euler_yaw(heading_quaternion.orientation)
    heading[0] = np.cos(yaw) * np.cos(pitch)
    heading[1] = np.sin(yaw) * np.cos(pitch)
    heading_mutex = 1


def quaternion_to_euler_yaw(heading_quaternion):
    """
    method for converting quaternion to euler
    :return: euler yaw
    """
    q = heading_quaternion
    pitch = np.arcsin(2 * (q.w * q.y - q.z * q.x))
    yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    return pitch, yaw


def obstacles_callback(data):
    """
    reads the /path_planner/obstacles topic and saves the obstacle as a global np.array
    :param data: std_msgs.msg.Float64MultiArray with latitude longitude for the obstacles
    :return: Nothing
    """
    global obstacles
    global obstacle_mutex
    obstacles = data.data
    obstacle_mutex = 1


def waypoint_index_callback(data):
    """
    read a topic to control the waypoint index
    :param data: :)
    :return:
    """
    global waypoint_index_control
    waypoint_index_control = data.data


def path_planner_subscriber():
    """
    start the subscriber methods
    :return: Nothing
    """
    # create node for the path planner
    rospy.init_node('path_planner')
    # start the subscribers for all sensors
    rospy.Subscriber("path_planner/waypoints", waypoint_array_msg, waypoint_callback)
    rospy.Subscriber("/gps/fix", sensor_msgs.msg.NavSatFix, gps_position_callback)
    rospy.Subscriber("/gps/fix_velocity", geometry_msgs.msg.TwistWithCovarianceStamped, gps_velocity_callback)
    rospy.Subscriber("/imu/data", sensor_msgs.msg.Imu, imu_heading_callback)
    rospy.Subscriber("/wind_sensor", std_msgs.msg.Float64MultiArray, wind_sensor_callback)
    rospy.Subscriber("/path_planner/obstacles", obstacles_array_msg, obstacles_callback)
    rospy.Subscriber("waypoint/index", std_msgs.msg.Int64, waypoint_index_callback)


def path_planning_init(config_object):
    """
    initialize the path planner by creating the reference frame and converting waypoints and obstacles to x,y
    coordinates in that reference frame. The reference frame is created in such a way that one x,y corresponds to one
    meter in the real world.
    :return: np.array of waypoints in x,y. np.array of obstacles in x,y. reference point 0, 1
    """
    waypoint_xy_array = np.zeros(0)
    global waypoints, obstacles, position_mutex, velocity_mutex, heading_mutex, obstacle_mutex, waypoint_mutex, w_theta
    global w_theta, latitude, longitude, waypoint_index_control, heading
    length_waypoints = 0
    # waits until obstacles and position data has been read from the topics
    while not rospy.is_shutdown():
        # check if data has been read
        if position_mutex == 1 and waypoint_mutex == 1 and velocity_mutex == 1 and \
                heading_mutex == 1 and wind_mutex == 1:
            length_waypoints = np.size(waypoints)
            # create np.array 2xN for waypoints and obstacles
            waypoint_xy_array = np.zeros(shape=(length_waypoints, 3))
            break
    potential_field_object = pfa.PotentialField(config_object.profile_diameter, config_object.obstacle_weight,
                                                config_object.d_inf, config_object.goal_weight, config_object.p_ngz,
                                                config_object.p_hyst, config_object.g_v, lin_velocity, w_speed,
                                                config_object.waypoint_radius, config_object.num_circle_point,
                                                config_object.radius_earth, w_theta, latitude, longitude,
                                                waypoint_index_control, heading, obstacles, obstacle_mutex,
                                                waypoint_mutex, velocity_mutex, heading_mutex, position_mutex,
                                                wind_mutex)
    # calculates position for two reference point 100m at 0 and 90 deg from the position  heading_mutex == 1 and
    [lat_0, lon_0, lat_1, lon_1] = potential_field_object.calculate_reference_points(latitude, longitude)
    # calculate the global x,y for the reference points
    ref0_pos = potential_field_object.latlng_to_global_xy_ref(lat_0, lon_0, lat_0, lat_1)
    ref1_pos = potential_field_object.latlng_to_global_xy_ref(lat_1, lon_1, lat_0, lat_1)
    # create two the object for the two reference points
    p_0 = ReferencePoint(0, config_object.local_coord_scale, lat_0, lon_0, ref0_pos[0], ref0_pos[1])
    p_1 = ReferencePoint(config_object.local_coord_scale, 0, lat_1, lon_1, ref1_pos[0], ref1_pos[1])
    # calculates the local x,y for the waypoints in the frame of the reference points
    for i in range(length_waypoints):
        waypoints_xy = potential_field_object.latlng_to_screen_xy(waypoints[i].latitude, waypoints[i].longitude, p_0,
                                                                  p_1)
        waypoint_xy_array[i, 0] = round(waypoints_xy[0])
        waypoint_xy_array[i, 1] = round(waypoints_xy[1])
        waypoint_xy_array[i, 2] = waypoints[i].id

    return waypoint_xy_array, p_0, p_1, potential_field_object


if __name__ == '__main__':
    waypoint_index = 0
    try:
        # initialize subscribers
        path_planner_subscriber()
        # initialize publisher
        pub_heading_main = rospy.Publisher('/path_planner/course', std_msgs.msg.Float64, queue_size=10)
        # read the parameters form the config.yaml file
        config_object_main = ConfigClass()
        # initialize waypoint, obstacles and the reference frame
        waypoint_xy, p0, p1, potential_field_object_main = path_planning_init(config_object_main)

        rate = rospy.Rate(1)  # profile_diameter: 50
        while not rospy.is_shutdown():
            potential_field_object_main.w_speed = w_speed
            potential_field_object_main.w_theta = w_theta
            potential_field_object_main.latitude = latitude
            potential_field_object_main.longitude = longitude
            potential_field_object_main.waypoint_index_control = waypoint_index_control
            potential_field_object_main.heading = heading
            potential_field_object_main.obstacles = obstacles

            potential_field_object_main.obstacle_mutex = obstacle_mutex
            potential_field_object_main.waypoint_mutex = waypoint_mutex
            potential_field_object_main.velocity_mutex = velocity_mutex
            potential_field_object_main.heading_mutex = heading_mutex
            potential_field_object_main.position_mutex = position_mutex
            potential_field_object_main.wind_mutex = wind_mutex

            # calculate the desired course of the vessel
            pos_v, goal_main, waypoint_array_len = potential_field_object_main.path_planning_calc_heading(
                waypoint_xy,
                p0, p1,
                pub_heading_main,
                waypoint_index)

            if np.linalg.norm(pos_v - goal_main[0:2]) < 1:
                waypoint_index = waypoint_index + 1
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
