#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
from marti_nav_msgs.msg import RoutePoint, Route
from scipy.spatial import distance
from path_planner.potential_field_algorithm import PotentialField
from path_planner import path_planner as pl


#from path_planner.msg import obstacles_array_msg

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
        self.default_waypoint_id = rospy.get_param("~default_waypoint_id")
        self.waypoints_to_circle_id = rospy.get_param("~waypoints_to_circle_id")
        self.waypoints_in_circle_id = rospy.get_param("~waypoints_in_circle_id")
        self.circle_time_limit = rospy.get_param("~circle_time_limit")


def waypoint_callback(data):
    """
    reads the "path_planner/waypoints" topic and saves the waypoints as a global np.array
    :param data: td_msgs.msg.Float64MultiArray with waypoint latitude, longitude
    :return: Nothing
    """
    global waypoints
    global waypoint_mutex
    waypoints = data.route_points
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
        true_wind_ = [data.vector.x * 1.94384449 + lin_velocity_x, data.vector.y * 1.94384449
                      + lin_velocity_y]
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
    lin_velocity_x = data.twist.twist.linear.x * 1.94384449
    lin_velocity_y = data.twist.twist.linear.y * 1.94384449
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
    pitch, yaw = pl.quaternion_to_euler_yaw(heading_quaternion.orientation)
    heading[0] = np.cos(yaw) * np.cos(pitch)
    heading[1] = np.sin(yaw) * np.cos(pitch)
    heading_mutex = 1


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
    rospy.Subscriber("path_planner/waypoints", Route, waypoint_callback)
    rospy.Subscriber("/gps/fix", sensor_msgs.msg.NavSatFix, gps_position_callback)
    rospy.Subscriber("/gps/fix_velocity", geometry_msgs.msg.TwistWithCovarianceStamped, gps_velocity_callback)
    rospy.Subscriber("/imu/data", sensor_msgs.msg.Imu, imu_heading_callback)
    rospy.Subscriber("/wind_sensor", geometry_msgs.msg.Vector3Stamped, wind_sensor_callback)
    #rospy.Subscriber("/path_planner/obstacles", obstacles_array_msg, obstacles_callback)
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
            # length_waypoints = np.size(waypoints)
            # create np.array 2xN for waypoints and obstacles
            # waypoint_xy_array = np.zeros(shape=(length_waypoints, 3))
            break
    potential_field_object = pfa.PotentialField(config_object.profile_diameter, config_object.obstacle_weight,
                                                config_object.d_inf, config_object.goal_weight, config_object.p_ngz,
                                                config_object.p_hyst, config_object.g_v, 0, 0)
    # calculates position for two reference point 100m at 0 and 90 deg from the position
    [lat_0, lon_0, lat_1, lon_1] = potential_field_object.calculate_reference_points(latitude, longitude)
    # calculate the global x,y for the reference points
    ref0_pos = pl.latlng_to_global_xy_ref(lat_0, lon_0, lat_0, lat_1)
    ref1_pos = pl.latlng_to_global_xy_ref(lat_1, lon_1, lat_0, lat_1)
    # create two the object for the two reference points
    p_0 = ReferencePoint(0, config_object.local_coord_scale, lat_0, lon_0, ref0_pos[0], ref0_pos[1])
    p_1 = ReferencePoint(config_object.local_coord_scale, 0, lat_1, lon_1, ref1_pos[0], ref1_pos[1])
    # calculates the local x,y for the waypoints in the frame of the reference points
    """    
    for i in range(length_waypoints):
        waypoints_xy = pl.latlng_to_screen_xy(waypoints[i].pose.position.y, waypoints[i].pose.position.x, p_0,
                                              p_1)
        waypoint_xy_array[i, 0] = round(waypoints_xy[0])
        waypoint_xy_array[i, 1] = round(waypoints_xy[1])
        waypoint_xy_array[i, 2] = waypoints[i].id"""

    return p_0, p_1, potential_field_object


if __name__ == '__main__':
    waypoint_index = 0
    try:
        # initialize subscribers
        path_planner_subscriber()
        # initialize publisher
        pub_heading = rospy.Publisher('/path_planner/course', std_msgs.msg.Float64, queue_size=10)
        # read the parameters form the config.yaml file
        config_object_main = ConfigClass()
        # initialize waypoint, obstacles and the reference frame
        p0, p1, potential_field_object_main = path_planning_init(config_object_main)
        rate = rospy.Rate(1)  # profile_diameter: 50
        potential_field_object_main.update_waypoints_ref(p0, p1)
        # current loop
        waypoint_xy = pl.calc_waypoints(p0, p1, waypoints)
        goal = waypoint_xy[0]
        # ------final loop--------
        # ------pseudo code start------
        # calculate desired angle
        # while not rospy.is_shutdown():
        #   update global variables
        #   update goal
        #   if waypoint circle waypoint?
        #       create circle
        #       goal_circle = circle_waypoint[closest_circle]
        #       while time limit
        #           update global variables
        #           update x,y position
        #           if pos - goal < condition
        #               goal_circle = circle_waypoint[next_index] # make circle_waypoint loop array
        #           calculate desired angle
        #           publish desired angle
        #       goal = waypoint[next_index]
        #   calculate desired angle # publish desired angle
        #   if pos - goal < condition
        #       if goal != waypoint[last_index]
        #           goal = waypoint[next_index]
        #       else:
        #           blow_up_vessel()
        #   rate.sleep()
        # ------pseudo code end------
        potential_field_object_main.update_waypoints_ref(p0, p1)
        while not rospy.is_shutdown():
            waypoint_xy = pl.calc_waypoints(p0, p1, waypoints)
            goal = waypoint_xy[waypoint_index]
            pos_v_xy = pl.latlng_to_screen_xy(latitude, longitude, p0, p1)
            # if waypoint circle waypoint?
            if goal[2] == config_object_main.waypoints_to_circle_id:
                start = time.time()
                elapsed = 0
                circle_waypoints = pl.circle_waypoint(latitude, longitude, goal, p0, p1)
                goal_circle = circle_waypoints[0]
                # while time limit
                while elapsed < config_object_main.circle_time_limit and not rospy.is_shutdown():
                    waypoint_xy = pl.calc_waypoints(p0, p1, waypoints)
                    pos_v_xy = pl.latlng_to_screen_xy(latitude, longitude, p0, p1)
                    if (np.diff([pos_v_xy, goal_circle[0:2]]) ** 2).sum() < 5:  # set limit in meter
                        if goal_circle == circle_waypoints[config_object_main.num_circle_point - 1]:
                            goal_circle = circle_waypoints[0]
                        else:
                            goal_circle = circle_waypoints[circle_waypoints.index(goal_circle) + 1]
                    min_angle = potential_field_object_main. \
                        path_planning_calc_heading(goal_circle, heading, w_speed, w_theta, pos_v_xy, obstacles,
                                                   lin_velocity)
                    # publish the calculated angle
                    pub_heading.publish(min_angle)
                    elapsed = time.time() - start
                if (goal != waypoint_xy[len(waypoint_xy) - 1]).all():
                    waypoint_index += 1
                    goal = waypoint_xy[waypoint_index]
            min_angle = potential_field_object_main.path_planning_calc_heading(goal, heading, w_speed, w_theta,
                                                                                        pos_v_xy, obstacles,
                                                                                        lin_velocity)
            # publish the calculated angle
            pub_heading.publish(min_angle)
            rospy.loginfo("Vessel position {}".format(pos_v_xy))
            rospy.loginfo("Goal position {}".format(goal))
            if (np.diff([pos_v_xy, goal[0:2]]) ** 2).sum() < 5:
                if (goal != waypoint_xy[len(waypoint_xy) - 1]).all():
                    waypoint_index += 1
                    goal = waypoint_xy[waypoint_index]
                else:
                    print("Path complete")
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
