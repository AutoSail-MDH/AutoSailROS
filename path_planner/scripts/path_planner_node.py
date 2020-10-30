#!/usr/bin/env python
import rospy
import numpy as np
import math
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg

from path_planner import potential_field_algorithm as pfa

waypoints = np.array([])
obstacles = np.array([])
longitude = 0.0
latitude = 0.0
lin_velocity = 0.0
heading = np.array([0, 0])
w_speed = 0
w_theta = 0
radius = 6371  # Earth Radius in KM


class ReferencePoint:
    def __init__(self, scr_x, scr_y, lat, lng, pos_x, pos_y):
        self.scrX = scr_x
        self.scrY = scr_y
        self.lat = lat
        self.lng = lng
        self.pos_x = pos_x
        self.pos_y = pos_y


def latlng_to_global_xy_ref(lat, lng, p0_lat, p1_lat):
    """
    converts latitude, longitude to global x,y coordinates for the two reference points.
    :param lat: latitude for the ref point being converted
    :param lng: latitude for the ref point being converted
    :param p0_lat: latitude for the first ref point
    :param p1_lat: latitude for the second ref point
    :return:
    """
    x = radius * lng * math.cos((p0_lat + p1_lat) / 2)
    y = radius * lat
    return [x, y]


def calculate_reference_points(lat, lon):
    """
    creates two reference points 100m from the position of the vessel at o and 90 deg
    :param lat: latitude of the vessel
    :param lon: longitude of the vessel
    :return: latitude and longitude for the two ref points.
    """
    bearing_0 = np.deg2rad(0)
    bearing_1 = np.deg2rad(90)
    distance_0 = 0.1  # 100 meter in km
    distance_1 = 0.1  # 100 meter in km
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    lat_1 = math.asin(math.sin(lat_rad) * math.cos(distance_0 / radius) +
                      math.cos(lat_rad) * math.sin(distance_0 / radius) * math.cos(bearing_0))
    lon_1 = lon_rad + math.atan2(math.sin(bearing_0) * math.sin(distance_0 / radius) * math.cos(lat_rad),
                                 math.cos(distance_0 / radius) - math.sin(lat_rad) * math.sin(lat_1))

    lat_2 = math.asin(math.sin(lat_rad) * math.cos(distance_1 / radius) +
                      math.cos(lat_rad) * math.sin(distance_1 / radius) * math.cos(bearing_1))
    lon_2 = lon_rad + math.atan2(math.sin(bearing_1) * math.sin(distance_1 / radius) * math.cos(lat_rad),
                                 math.cos(distance_1 / radius) - math.sin(lat_rad) * math.sin(lat_2))

    lat_1 = math.degrees(lat_1)
    lon_1 = math.degrees(lon_1)
    lat_2 = math.degrees(lat_2)
    lon_2 = math.degrees(lon_2)
    return [lat_1, lon_1, lat_2, lon_2]


def latlng_to_global_xy(lat, lng, p_0, p_1):
    """
    converts latitude, longitude to global x,y coordinates for any point
    :param lat: latitude of the point
    :param lng: longitude of the point
    :param p_0: ReferencePoint 0
    :param p_1: ReferencePoint 1
    :return: global x,y coordinates for the point
    """
    x = radius * lng * math.cos((p_0.lat + p_1.lat) / 2)
    y = radius * lat
    return [x, y]


def latlng_to_screen_xy(lat, lng, p_0, p_1):
    """
    convert latitude, longitude for a point to x,y in the reference frame
    :param lat: latitude for the point being converted
    :param lng: longitude for the point being converted
    :param p_0: reference point 0
    :param p_1: reference point 1
    :return: x,y coordinates for the point in the reference frame
    """
    pos = latlng_to_global_xy(lat, lng, p_0, p_1)
    per_x = ((pos[0] - p_0.pos_x) / (p_1.pos_x - p_0.pos_x))
    per_y = ((pos[1] - p_0.pos_y) / (p_1.pos_y - p_0.pos_y))

    return p_0.scrX + (p_1.scrX - p_0.scrX) * per_x, p_0.scrY + (p_1.scrY - p_0.scrY) * per_y


def waypoint_callback(data):
    """
    reads the "path_planner/waypoints" topic and saves the waypoints as a global np.array
    :param data: td_msgs.msg.Float64MultiArray with waypoint latitude, longitude
    :return: Nothing
    """
    global waypoints
    waypoints = data.data


def wind_sensor_callback(data):
    """
    reads the /wind_sensor topic and saves the wind speed and wind angle as global variables
    :param data: std_msgs.msg.Int64MultiArray with wind speed and wind angle
    :return: Nothing
    """
    global w_speed
    global w_theta
    w_speed = data.data[0]
    w_theta = data.data[1]


def gps_position_callback(data):
    """
    reads the gps/fix topic and saves the latitude, longitude for the vessel as global variables
    :param data: sensor_msgs.msg.NavSatFix with latitude. longitude
    :return: Nothing
    """
    global longitude
    global latitude
    longitude = data.longitude
    latitude = data.latitude


def gps_velocity_callback(data):
    """
    reads the gps/fix_velocity topic and saves the linear velocity as a global variable
    :param data: geometry_msgs.msg.TwistWithCovarianceStamped with linear x, y velocity
    :return: Nothing
    """
    global lin_velocity
    lin_velocity_x = data.twist.twist.linear.x
    lin_velocity_y = data.twist.twist.linear.y
    lin_velocity = math.sqrt((lin_velocity_x ** 2) + (lin_velocity_y ** 2))


def gps_heading_callback(data):
    """
    reads the /gps/navheading topic and saves the heading as a global variable
    :param data: sensor_msgs.msg.Imu with x,y orientation
    :return: Nothing
    """
    global heading
    heading[0] = data.orientation.x
    heading[1] = data.orientation.y


def obstacles_callback(data):
    """
    reads the /path_planner/obstacles topic and saves the obstacle as a global np.array
    :param data: std_msgs.msg.Float64MultiArray with latitude longitude for the obstacles
    :return: Nothing
    """
    global obstacles
    obstacles = data.data


def path_planner_subscriber():
    """
    start the subscriber methods
    :return: Nothing
    """
    rospy.init_node('path_planner')
    rospy.Subscriber("path_planner/waypoints", std_msgs.msg.Float64MultiArray, waypoint_callback)
    rospy.Subscriber("gps/fix", sensor_msgs.msg.NavSatFix, gps_position_callback)
    rospy.Subscriber("gps/fix_velocity", geometry_msgs.msg.TwistWithCovarianceStamped, gps_velocity_callback)
    rospy.Subscriber("/gps/navheading", sensor_msgs.msg.Imu, gps_heading_callback)
    rospy.Subscriber("/wind_sensor", std_msgs.msg.Int64MultiArray, wind_sensor_callback)
    rospy.Subscriber("/path_planner/obstacles", std_msgs.msg.Float64MultiArray, obstacles_callback)


#    rospy.spin()


def path_planning_init():
    """
    initialize the path planner by creating the reference frame and converting waypoints and obstacles to x,y
    coordinates in that reference frame. The reference frame is created in such a way that one x,y corresponds to one
    meter in the real world.
    :return: np.array of waypoints in x,y. np.array of obstacles in x,y. reference point 0, 1
    """
    waypoint_xy_array = np.zeros(0)
    obstacles_xy_array = np.zeros(0)
    length_waypoints = int(len(waypoints))
    length_obstacles = int(len(obstacles))
    while not rospy.is_shutdown():
        length_waypoints = int(len(waypoints))
        length_obstacles = int(len(obstacles))
        if length_waypoints > 0 and longitude > 0 and latitude > 0 and length_obstacles:
            waypoint_xy_array = np.zeros(shape=(int(length_waypoints / 2), 2))
            obstacles_xy_array = np.zeros(shape=(int(length_obstacles / 2), 2))
            break
    [lat_0, lon_0, lat_1, lon_1] = calculate_reference_points(latitude, longitude)
    ref0_pos = latlng_to_global_xy_ref(lat_0, lon_0, lat_0, lat_1)
    ref1_pos = latlng_to_global_xy_ref(lat_1, lon_1, lat_0, lat_1)

    # Calculate global X and Y for top-left reference point
    p_0 = ReferencePoint(0, 100, lat_0, lon_0, ref0_pos[0], ref0_pos[1])
    # Calculate global X and Y for bottom-right reference point
    p_1 = ReferencePoint(100, 0, lat_1, lon_1, ref1_pos[0], ref1_pos[1])
    j = 0
    for i in range(int(length_waypoints / 2)):
        waypoints_xy = latlng_to_screen_xy(waypoints[j], waypoints[j + 1], p_0, p_1)
        waypoint_xy_array[i, 0] = waypoints_xy[0]
        waypoint_xy_array[i, 1] = waypoints_xy[1]
        j = j + 2
    j = 0
    for i in range(int(length_obstacles / 2)):
        obstacles_xy_1d = latlng_to_screen_xy(obstacles[j], obstacles[j + 1], p_0, p_1)
        obstacles_xy_array[i, 0] = obstacles_xy_1d[0]
        obstacles_xy_array[i, 1] = obstacles_xy_1d[1]
        j = j + 2
    print("obstacles_xy_array", obstacles_xy_array)

    return waypoint_xy_array, obstacles_xy_array, p_0, p_1


def path_planning_calc_heading(waypoint_array, obstacles_array, p_0, p_1):
    """
    calculates the desired heading of the vessel
    :param waypoint_array: np.array of waypoints in x,y reference frame
    :param obstacles_array: np.array of obstacles in x,y reference frame
    :param p_0: reference point 0
    :param p_1: reference point 1
    :return: desired heading
    """
    global w_speed
    global w_theta
    global latitude
    global longitude

    position_v = latlng_to_screen_xy(latitude, longitude, p_0, p_1)
    pub_heading = rospy.Publisher('/path_planner/heading', std_msgs.msg.Float64, queue_size=10)

    profile_diameter = rospy.get_param("~profile_diameter", default=100)
    goal_weight = rospy.get_param("~goal_weight", default=5)
    obstacle_weight = rospy.get_param("~obstacle_weight", default=20)
    d_inf = rospy.get_param("~d_inf", default=20)
    p_ngz = rospy.get_param("~p_ngz", default=200)
    p_hyst = rospy.get_param("~p_hyst", default=10)
    g_v = rospy.get_param("~g_v", default=1)

    potential_field_object = pfa.PotentialField(profile_diameter, obstacle_weight, d_inf, goal_weight,
                                                p_ngz, p_hyst, g_v, lin_velocity, w_speed)

    goal = waypoint_array[0]
    profile, list_len = potential_field_object.create_profile(pos_v=position_v)

    profile = potential_field_object.calculate_total_potential(profile, list_len, obstacles_array, goal, w_theta,
                                                               heading)
    min_angle, min_index = potential_field_object.find_global_minima_angle(profile)
    desired_heading_angle = min_angle
    pub_heading.publish(desired_heading_angle)
    return min_angle


"""
    import matplotlib.pyplot as plt
    profile_matrix = potential_field_object.reshape_profile(profile)
    plt.imshow(profile_matrix, cmap='hot', interpolation='nearest')
    plt.show()
"""

if __name__ == '__main__':
    try:
        path_planner_subscriber()
        waypoint_xy, obstacles_xy, p0, p1 = path_planning_init()
        while not rospy.is_shutdown():
            rate = rospy.Rate(60)
            path_planning_calc_heading(waypoint_xy, obstacles_xy, p0, p1)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
