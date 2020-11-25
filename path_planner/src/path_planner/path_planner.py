import numpy as np
import math
from scipy.spatial import distance
from pymap3d.ned import geodetic2ned


radius_earth = 6371
waypoint_radius = 10
num_circle_point = 8


class ReferencePoint:
    def __init__(self, scr_x, scr_y, lat, lng, pos_x, pos_y):
        self.scrX = scr_x
        self.scrY = scr_y
        self.lat = lat
        self.lng = lng
        self.pos_x = pos_x
        self.pos_y = pos_y


def quaternion_to_euler_yaw(heading_quaternion):
    """
    method for converting quaternion to euler
    :return: euler yaw
    """
    q = heading_quaternion
    pitch = np.arcsin(2 * (q.w * q.y - q.z * q.x))
    yaw = - np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    return pitch, yaw


def latlng_to_global_xy_ref(lat, lng, p0_lat, p0_lng):
    """
    converts latitude, longitude to global x,y coordinates for the two reference points.
    :param lat: latitude for the ref point being converted
    :param lng: latitude for the ref point being converted
    :param p0_lat: latitude for the first ref point
    :param p1_lat: latitude for the second ref point
    :return:
    """
    [x, y, d] = geodetic2ned(lat, lng, 0, p0_lat, p0_lng, 0)
    # y = radius_earth * lng * math.cos((p0_lat + p1_lat) / 2)
    # x = radius_earth * lat
    return [x, y]


def latlng_to_global_xy(lat, lng, p_0, p_1):
    """
    converts latitude, longitude to global x,y coordinates for any point
    :param lat: latitude of the point
    :param lng: longitude of the point
    :param p_0: ReferencePoint 0
    :param p_1: ReferencePoint 1
    :return: global x,y coordinates for the point
    """
    # y = radius_earth * lng * math.cos((p_0.lat + p_1.lat) / 2)
    # x = radius_earth * lat
    return latlng_to_global_xy_ref(lat, lng, p_0.lat, p_0.lng)


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


def rotate_point(x, y, r):
    rx = (x * math.cos(r)) - (y * math.sin(r))
    ry = (y * math.cos(r)) + (x * math.sin(r))
    return rx, ry


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
    return points


def closest_waypoint(pos, circle_points):
    """
    finds the index of the waypoint closest to the vessel
    :param pos: position of the vessel
    :param circle_points: waypoints in the circle
    :return:
    """
    """
    pos_vessel = np.ndarray(shape=(2, 2))
    pos_vessel[0][0] = pos[0]
    pos_vessel[0][1] = pos[1]
    """
    closest_index = distance.cdist([pos], circle_points, 'euclidean').argmin()
    return closest_index


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


def calculate_reference_points(lat, lon):
    """
    creates two reference points 100m from the position of the vessel at o and 90 deg
    :param lat: latitude of the vessel
    :param lon: longitude of the vessel
    :return: latitude and longitude for the two ref points.
    """
    # the angle to the reference points from vessel position
    bearing_0 = np.deg2rad(0)
    bearing_1 = np.deg2rad(90)
    distance_0 = 0.1  # 100 meter in km
    distance_1 = 0.1  # 100 meter in km
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    # calculate the latitude longitude of the reference points
    lat_1 = math.asin(math.sin(lat_rad) * math.cos(distance_0 / radius_earth) +
                      math.cos(lat_rad) * math.sin(distance_0 / radius_earth) * math.cos(bearing_0))
    lon_1 = lon_rad + math.atan2(math.sin(bearing_0) * math.sin(distance_0 / radius_earth) * math.cos(lat_rad),
                                 math.cos(distance_0 / radius_earth) - math.sin(lat_rad) * math.sin(lat_1))

    lat_2 = math.asin(math.sin(lat_rad) * math.cos(distance_1 / radius_earth) +
                      math.cos(lat_rad) * math.sin(distance_1 / radius_earth) * math.cos(bearing_1))
    lon_2 = lon_rad + math.atan2(math.sin(bearing_1) * math.sin(distance_1 / radius_earth) * math.cos(lat_rad),
                                 math.cos(distance_1 / radius_earth) - math.sin(lat_rad) * math.sin(lat_2))
    # convert the reference points from rad to degrees
    lat_1 = math.degrees(lat_1)
    lon_1 = math.degrees(lon_1)
    lat_2 = math.degrees(lat_2)
    lon_2 = math.degrees(lon_2)
    return [lat_1, lon_1, lat_2, lon_2]

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
