#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
import multiprocessing
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
import message_filters
from pymap3d.ned import geodetic2ned, ned2geodetic
from marti_nav_msgs.msg import RoutePoint, Route
from marti_common_msgs.msg import KeyValue
from scipy.spatial import distance
from dynamic_reconfigure.server import Server
from autosail.cfg import PathPlannerConfig
from path_planner.potential_field_algorithm import PotentialField
from path_planner.path_planner import *
import matplotlib

from autosail.msg import obstacles_array_msg
from autosail.msg import TwoDimensionalPlotDatapoint, TwoDimensionalPlot
import geometry_msgs.msg

# global variables updated by the callback functions via the subscribers
waypoints = []
obstacles = []
lat0 = None
lon0 = None
current_position = None
velocity = None
heading = []  # np.array([0., 0.])
w_speed = None
w_theta = None

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


# Dynamic reconfiguration
def dynamic_reconf_callback(dyn_conf, level):
    """
    creates dynamic reconfiguration in rqt
    :return:
    """
    global pf, config
    pf.goal_weight = config.goal_weight = dyn_conf.goal_weight
    pf.obstacle_weight = config.obstacle_weight = dyn_conf.obstacle_weight
    pf.diameter = config.profile_diameter = dyn_conf.profile_diameter
    pf.d_inf = config.d_inf = dyn_conf.d_inf
    pf.p_ngz = config.p_ngz = dyn_conf.p_ngz
    pf.p_hyst = config.p_hyst = dyn_conf.p_hyst
    pf.g_v = config.g_v = dyn_conf.g_v
    pf.wind_limit = dyn_conf.wind_limit
    config.circle_time_limit = dyn_conf.circle_time_limit
    rospy.loginfo("Reconfigure request: {}".format(config))
    return dyn_conf


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
    global w_speed, w_theta
    true_wind_ = [data.vector.x * 1.94384449, data.vector.y * 1.94384449]
    w_theta = np.arctan2(true_wind_[1], true_wind_[0])
    w_speed = np.linalg.norm(true_wind_)
    w_theta = w_theta + np.pi
    w_theta = math.atan2(math.sin(w_theta), math.cos(w_theta))
    # rospy.loginfo("True wind: {}".format(math.degrees(w_theta)))


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
    global velocity
    lin_velocity_x = data.twist.twist.linear.x * 1.94384449
    # lin_velocity_y = - data.twist.twist.linear.y * 1.94384449
    lin_velocity_y = data.twist.twist.linear.y * 1.94384449
    lin_velocity = [lin_velocity_x, lin_velocity_y]
    velocity = math.sqrt((lin_velocity_x ** 2) + (lin_velocity_y ** 2))


def imu_heading_callback(data):
    """
    reads the /filter/quaternion topic and saves the heading as a global variable. Is used for the real sensor
    :param data: geometry_msgs.msg.QuaternionStamped(
    :return: Nothing
    """
    global heading
    heading_quaternion = Imu()
    heading_quaternion.orientation = data.orientation
    # yaw = -quaternion_to_euler_yaw(heading_quaternion.orientation)
    yaw = quaternion_to_euler_yaw(heading_quaternion.orientation)
    heading = [np.cos(yaw), np.sin(yaw)]


def obstacle_exist(obstacle):
    """
    checks if obstacle exists
    :return:
    """
    global obstacles
    for o in obstacles:
        if 0 < (obstacle[0] - o[0]) < 0.00001 or 0 < (obstacle[1] - o[1]) < 0.00001:
            return True
    return False


def obstacle_camera_callback(data):
    """
    Read obstacle from camera and add them if it differs from the existing obstacles
    :param data: geometry_msgs.msg.Vector3Stamped
    """
    global obstacles, current_position
    geo = ned2geodetic(data.vector.x, data.vector.y, 0, current_position.latitude, current_position.longitude, 0)[:2]
    rospy.loginfo("Got obstacle at {}".format(geo))
    if not obstacle_exist(geo):
        obstacles.append(geo)


def path_planner_init():
    """
    initilizes the subscribers and waits until topics are read
    :return:
    """
    global waypoints, w_theta, current_position, heading, w_speed
    # create node for the path planner
    rospy.init_node('path_planner', log_level=rospy.get_param("log_level", rospy.INFO))
    # start the subscribers for all sensors
    rospy.Subscriber("path_planner/waypoints", Route, waypoint_callback, queue_size=1)
    rospy.Subscriber("/gps/fix", NavSatFix, gps_position_callback, queue_size=1)
    rospy.Subscriber("/gps/fix_velocity", TwistWithCovarianceStamped, gps_velocity_callback, queue_size=1)
    rospy.Subscriber("/imu/data", Imu, imu_heading_callback, queue_size=1)
    rospy.Subscriber("/wind_sensor/true", Vector3Stamped, wind_sensor_callback, queue_size=1)
    rospy.Subscriber("/camera/data", Vector3Stamped, obstacle_camera_callback, queue_size=1)

    # waits until obstacles and position data has been read from the topics
    while not rospy.is_shutdown():
        # check if data has been read
        if waypoints and w_theta is not None and current_position is not None and heading and w_speed is not None:
            break


def webviz_msg(pf):
    """
    create heatmap msg for webviz 2dplot
    :param pf: potential feild object
    :return: msgs to publish
    """
    profile_matrix = pf.reshpe_profile_2d_plot()
    webvizPlot = TwoDimensionalPlot()
    data = geometry_msgs.msg.Point()
    norm = matplotlib.colors.Normalize(vmin=profile_matrix.min(), vmax=profile_matrix.max())
    cmap = matplotlib.cm.get_cmap('hot')

    h = 0
    # min_value = min(list)
    for j in range(pf.diameter):
        for i in range(pf.diameter):
            data = geometry_msgs.msg.Point()
            webvizPlotData = TwoDimensionalPlotDatapoint()
            string_label = str(h)
            data.x = -pf.diameter - 1 - i
            data.y = -j
            webvizPlotData.data.append(data)
            webvizPlotData.label = string_label
            color = cmap(norm(profile_matrix[j, pf.diameter - 1 - i]))
            webvizPlotData.pointBackgroundColor = f"rgba({color[0] * 255}, {color[1] * 255}, {color[2] * 255}, 1)"  # pink, blue, teal, lightgray
            webvizPlotData.pointStyle = "rect"
            webvizPlotData.pointRadius = "15"
            webvizPlot.points.append(webvizPlotData)
            h += 1
    webvizPlotLine = TwoDimensionalPlotDatapoint()
    webvizPlotLine.label = "line"
    webvizPlotLine.borderColor = "blue"
    webvizPlotLine.backgroundColor = "blue"
    webvizPlotLine.borderWidth = 10
    data = geometry_msgs.msg.Point()
    data.x = -102 + (pf.diameter / 2 - 1 / 2)
    data.y = -(pf.diameter / 2 - 1 / 2)
    webvizPlotLine.data.append(data)
    data = geometry_msgs.msg.Point()
    # data.x = heading[1]
    data.x = - 102 + pf.diameter / 2 + heading[1] * 25
    # data.y = heading[0]
    data.y = -(pf.diameter / 2 - heading[0] * 25)
    webvizPlotLine.data.append(data)
    webvizPlot.lines.append(webvizPlotLine)
    webvizPlot.title = "title"
    webvizPlot.yAxisLabel = "yAxisLabel"
    webvizPlot.xAxisLabel = "xAxisLabel"
    return webvizPlot


def circle_waypoint(lat, lon, config, pf, pub_heading):
    global current_position
    circle_index = 0
    start = time.time()
    elapsed = 0
    circle_points = generate_circle_waypoints(lat1=lat, lon1=lon)
    circle_points_xy, circle_points_lat_lon = circle_in_order(circle_points, current_position)

    # goal_circle = circle_points_xy[0]

    while elapsed < config.circle_time_limit and not rospy.is_shutdown():

        circle_points_xy = circle_to_xy(circle_points_lat_lon, current_position)
        goal_circle = circle_points_xy[circle_index]
        rospy.loginfo("goal_circle {}".format(goal_circle))

        if np.linalg.norm(goal_circle[0:2]) < 5:  # set limit in meter
            circle_index += 1
            if goal_circle == circle_points_xy[7]:
                circle_index = 0
                goal_circle = circle_points_xy[0]
            else:
                goal_circle = circle_points_xy[circle_index]
        min_angle = pf.calc_heading(goal_circle, heading, w_speed, w_theta, [0, 0], obstacles_xy, velocity)

        # publish the calculated angle
        pub_heading.publish(min_angle)
        pf.plot_heat_map(0.1, heading)
        elapsed = time.time() - start


if __name__ == '__main__':
    waypoint_index = 0
    # initialize subscribers

    # initialize publisher
    pub_heading = rospy.Publisher('/path_planner/course', Float64, queue_size=10)
    pub_2d = rospy.Publisher('/path_planner/2dplot', TwoDimensionalPlot, queue_size=10)
    # read the parameters from the config.yaml file
    # Initialize
    path_planner_init()
    config = Config()
    pf = PotentialField(config.profile_diameter, config.obstacle_weight, config.d_inf, config.goal_weight, config.p_ngz,
                        config.p_hyst, config.g_v, 0)
    # Dynamic reconfigure
    srv = Server(PathPlannerConfig, dynamic_reconf_callback)

    rate = rospy.Rate(5)
    # current loop
    circle = KeyValue()
    circle.value = "1"
    while not rospy.is_shutdown():
        goal = geodetic2ned(waypoints[waypoint_index].pose.position.y, waypoints[waypoint_index].pose.position.x, 0,
                            current_position.latitude, current_position.longitude, 0)
        obstacles_xy = [geodetic2ned(o[0], o[1], 0, current_position.latitude, current_position.longitude, 0)[:2]
                        for o in obstacles]
        # rospy.loginfo("obstacles_xy {}".format(obstacles_xy))
        for prop in waypoints[waypoint_index].properties:
            if prop.key == "id":
                waypoint_id = prop.value
        # Code for circling waypoints

        #if obstacles:
        #    rospy.loginfo("obstacles[0] {}" .format(obstacles[0]))
        #    # rospy.loginfo("obstacles[0][0] {}" .format(obstacles[0][0]))
        #    circle_waypoint(obstacles[0][0], obstacles[0][1], config, pf, pub_heading)

        if waypoint_id == circle.value:
            circle_waypoint(waypoints[waypoint_index].pose.position.y, waypoints[waypoint_index].pose.position.x,
                            config, pf, pub_heading)
            waypoint_index += 1
            goal = geodetic2ned(waypoints[waypoint_index].pose.position.y, waypoints[waypoint_index].pose.position.x, 0,
                                current_position.latitude, current_position.longitude, 0)
            for prop in waypoints[waypoint_index].properties:
                if prop.key == "id":
                    waypoint_id = prop.value

        min_angle = pf.calc_heading(goal, heading, w_speed, w_theta, [0, 0], obstacles_xy, velocity)
        min_angle = math.atan2(math.sin(min_angle), math.cos(min_angle))
        # publish the calculated angle
        pub_heading.publish(min_angle)

        webvizPlot = webviz_msg(pf)
        pub_2d.publish(webvizPlot)

        # if rospy.get_param("~plot", "true") == "true":
        pf.plot_heat_map(0.1, heading)
        if np.linalg.norm(goal[0:2]) < 5:
            if waypoint_index != len(waypoints) - 1:
                waypoint_index += 1
                goal = waypoints[waypoint_index]
                rospy.logwarn("Next waypoint %d", waypoint_index)
            else:
                rospy.loginfo("Path complete")
        rate.sleep()
