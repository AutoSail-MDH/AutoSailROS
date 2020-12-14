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


def generate_circle_waypoints(lat1, lon1):
    """
    generates lat, lon points around the input
    :param lat1: latitude
    :param lon1: longitude
    :return: list of points
    """
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
        circle_waypoints.append(circle_waypoint)
        brng = brng + math.pi / 4
    return circle_waypoints

def closest_waypoint(circle_points):
    """
    finds the index of the waypoint closest to the vessel
    :param circle_points: waypoints in the circle
    :return:
    """
    closest_index = np.argmin(np.linalg.norm(circle_points))
    return closest_index

def circle_waypoint(waypoint, current_position):
    """
    runs the generate_circle_waypoints method. This is basically a usless method
    :param waypoint: the waypoint the new points will generate around
    :param current_position: position of the vessel
    :return: the new generated points
    """
    circle_points = generate_circle_waypoints(lat1=waypoint.pose.position.y, lon1=waypoint.pose.position.x)
    return circle_points

def circle_to_xy(circle_points, current_position):
    """
    converts the lat lon points to x,y relative current position
    :param circle_points: points in a circle
    :param current_position: current posistion of the vessel
    :return: circle_points in x,y
    """
    circle_points_xy = []
    for i in range(8):
        points = geodetic2ned(circle_points[i].lat, circle_points[i].lon, 0,
                            current_position.latitude, current_position.longitude, 0)
        circle_points_xy.append(points)
    return circle_points_xy

def circle_in_order(circle_points, current_position):
    """
    checks which point is closest to the vessel and then moves that point to the start of the list
    :param circle_points:  points in a circle
    :param current_position: position of the vessel
    :return: list of points in x,y. list of points in lat,lon
    """
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
