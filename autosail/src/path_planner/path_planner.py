import numpy as np
import math
from scipy.spatial import distance
from pymap3d.ned import geodetic2ned
import rospy


waypoint_radius = 10
num_circle_point = 8

class waypointClass:
    def __init__(self, lat, lon, x, y):
        self.lat = lat
        self.lon = lon
        self.x = x
        self.y = y


def quaternion_to_euler_yaw(heading_quaternion):
    """
    method for converting quaternion to euler
    :return: euler yaw
    """
    q = heading_quaternion
    yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    return yaw


def rotate_point(x, y, r):
    rx = (x * math.cos(r)) - (y * math.sin(r))
    ry = (y * math.cos(r)) + (x * math.sin(r))
    return rx, ry

"""
def generate_circle_waypoints(center):
    # calculate the angle between the points
    point_angle = (2 * math.pi) / num_circle_point
    points = []
    for i in range(num_circle_point):
        # create x,y at the circumference with angle point_angle * i
        (p_x, p_y) = rotate_point(0, waypoint_radius, point_angle * i)
        p_x += center[0]
        p_y += center[1]
        points.append((round(p_x), round(p_y)))
    return points"""

def generate_circle_waypoints(lat1, lon1):
    R = 6378.1  # Radius of the Earth
    brng = 0
    d = 1  # Distance in km
    circle_waypoints = []
    #circle_waypoint = waypointClass(0, 0, 0, 0)


    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    for i in range(8):
        circle_waypoint = waypointClass(0, 0, 0, 0)
        #rospy.loginfo("brng {}".format(brng))
        lat2 = math.asin(math.sin(lat1) * math.cos(d / R) +
                         math.cos(lat1) * math.sin(d / R) * math.cos(brng))
        lon2 = lon1 + math.atan2(math.sin(brng) * math.sin(d / R) * math.cos(lat1),
                                 math.cos(d / R) - math.sin(lat1) * math.sin(lat2))
        lat2 = math.degrees(lat2)
        lon2 = math.degrees(lon2)
        circle_waypoint.lat = lat2
        circle_waypoint.lon = lon2
        #rospy.loginfo("lat2 {}".format(lat2))
        #rospy.loginfo("lon2 {}".format(lon2))

        circle_waypoints.append(circle_waypoint)
        brng = brng + math.pi / 4
    return circle_waypoints

def closest_waypoint(circle_points):
    """
    finds the index of the waypoint closest to the vessel
    :param pos: position of the vessel
    :param circle_points: waypoints in the circle
    :return:
    """
    """
    pos = np.ndarray(shape=(2, 2))
    pos_vessel[0][0] = pos[0]
    pos_vessel[0][1] = pos[1]
    """
    #pos = np.ndarray([0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0])
    #closest_index = np.argmin(np.sum((circle_points)**2, axis=1))
    closest_index = np.argmin(np.linalg.norm(circle_points))
    # closest_index = distance.cdist([pos], circle_points, 'euclidean').argmin()
    return closest_index

def circle_waypoint(waypoint, current_position):
    #circle_points = generate_circle_waypoints(waypoint)
    circle_points = generate_circle_waypoints(lat1=waypoint.pose.position.y, lon1=waypoint.pose.position.x)
    return circle_points

def circle_to_xy(circle_points, current_position):
    circle_points_xy = []
    for i in range(8):
        points = geodetic2ned(circle_points[i].lat, circle_points[i].lon, 0,
                            current_position.latitude, current_position.longitude, 0)
        circle_points_xy.append(points)
    return circle_points_xy

def circle_in_order(circle_points, current_position):
    circle_points_xy = []
    circle_points_lat_lon = []
    # data = waypointClass()
    for i in range(8):
        points = geodetic2ned(circle_points[i].lat, circle_points[i].lon, 0,
                            current_position.latitude, current_position.longitude, 0)
        circle_points_xy.append(points[0:2])

    closest_index = closest_waypoint(circle_points_xy)
    for i in range(closest_index):
        circle_points_xy.append(circle_points_xy[i])
    for i in range(8):
        circle_points_lat_lon.append(circle_points[i]) # [circle_points[i].lat, circle_points[i].lon]
    for i in range(closest_index):
        circle_points_xy.pop(0)
        i += 1
    return circle_points_xy, circle_points_lat_lon

"""
def calc_waypoints(p_0, p_1, waypoints):
    waypoint_xy_array = [] # np.zeros(shape=(length_waypoints, 3))
    for waypoint in waypoints:
        waypoints_xy = latlng_to_global_xy(waypoint.pose.position.y, waypoint.pose.position.x, p_0, p_1)
        waypoint_xy_array.append([round(waypoints_xy[0]), round(waypoints_xy[1]), waypoint.id])
    return waypoint_xy_array


def circle_waypoint(latitude, longitude, waypoint, p_0, p_1):
    position_v = latlng_to_screen_xy(latitude, longitude, p_0, p_1)
    circle_points = generate_circle_waypoints(waypoint)
    closest_index = closest_waypoint(position_v, circle_points)
    for i in range(closest_index):
        circle_points.append(circle_points[i])
    for i in range(closest_index):
        circle_points.pop(0)
        i += 1
    # goal = circle_points[0]
    return circle_points


def calc_waypoints(p_0, p_1, waypoints):
    waypoint_xy_array = [] # np.zeros(shape=(length_waypoints, 3))
    for waypoint in waypoints:
        waypoints_xy = latlng_to_global_xy(waypoint.pose.position.y, waypoint.pose.position.x, p_0, p_1)
        waypoint_xy_array.append([round(waypoints_xy[0]), round(waypoints_xy[1]), waypoint.id])
    return waypoint_xy_array

"""


"""
    def calculate_segment(self, position_v, obstacle, goal, w_theta, heading):

        The function calculates and saves the global x,y coordinates for all points the vessel travels through to one
        waypoint.
        :param position_v: The current position of the vessel.
        :param obstacle: A numpy array of all the obstacles.
        :param goal: The position of the goal i.e current waypoint
        :param w_theta: The wind angle
        :param heading: The heading of the vessel
        :return: The x,y coordinates for all points from the start of the calculation to the goal.

        i = 0
        x_main_loop = []
        y_main_loop = []
        x_main_loop.append(position_v[0])
        y_main_loop.append(position_v[1])
        while 1:
            # check if the point is within 5m of the current goal
            if np.linalg.norm(position_v - goal) < 5:
                return x_main_loop, y_main_loop
            # create profile
            profile, list_len = self._create_profile(pos_v=position_v)
            # calculate total potential
            profile = self._calculate_total_potential(profile, list_len, obstacle, goal, w_theta, heading)
            # find min angle and index
            min_angle, min_index = self._find_global_minima_angle(profile)
            # save the global x,y position of the vessel
            x_main_loop.append(profile[min_index].g_kx)
            y_main_loop.append(profile[min_index].g_ky)
            # move the vessel to the point with the lowest potential
            position_v[0] = profile[min_index].g_kx
            position_v[1] = profile[min_index].g_ky
            # update the heading
            heading[0] = profile[min_index].l_kx
            heading[1] = profile[min_index].l_ky
            i = i + 1 """
