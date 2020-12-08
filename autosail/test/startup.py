#!/usr/bin/env python
import rospy
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import numpy as np
from marti_nav_msgs.msg import RoutePoint, Route
from autosail.msg import obstaclemsg
from autosail.msg import obstacles_array_msg
from scipy.spatial.transform import Rotation
import std_msgs.msg

import math
import rospy
import runpy
import os

desired_course = None
rudder_angle = None
sail_servo_angle = None


w_speed = None
longitude = None
lin_velocity = None
yaw = None

longitudes = []
latitudes = []
water_levels = []
gps_velocities = []
wind_speeds = []
yaws = []



class FakeSignals:
    def __init__(self):
        # --------- path planner sensors -----
        # waypoints !
        waypoint_array = Route()
        waypoint = RoutePoint()  # 90 deg
        waypoint.pose.position.x = 16.561736302687205
        waypoint.pose.position.y = 59.61744366137741
        waypoint.id = "0" # 59.61744366137741, 16.561736302687205
        self.waypoints = waypoint_array.route_points.append(waypoint)
        # obstacle
        mat_obstacle = obstaclemsg()
        mat_obstacle.latitude = 59.617369
        mat_obstacle.longitude = 16.560619
        obstacle_array = obstacles_array_msg()
        obstacle_array.data.append(mat_obstacle)
        self.obstacle = obstacle_array
        # wind sensor !
        wind_sensor_value = geometry_msgs.msg.Vector3Stamped()
        wind_sensor_value.vector.x = 7
        wind_sensor_value.vector.y = 7
        wind_sensor_value.vector.z = 0
        self.wind_sensor = wind_sensor_value
        # imu heading !
        heading = sensor_msgs.msg.Imu()
        rot = Rotation.from_euler('xyz', [0, 0, 0], degrees=True)
        rot_quat = rot.as_quat()
        heading.orientation.x = rot_quat[0]
        heading.orientation.y = rot_quat[1]
        heading.orientation.z = rot_quat[2]
        heading.orientation.w = rot_quat[3]
        self.heading = heading
        # gps velocity !
        gps_velocity_value = geometry_msgs.msg.TwistWithCovarianceStamped()
        gps_velocity_value.twist.twist.linear.x = 1
        gps_velocity_value.twist.twist.linear.y = 0
        gps_velocity_value.twist.twist.linear.z = 0
        self.gps_velocity = gps_velocity_value
        # gps position !
        gps_position_value = sensor_msgs.msg.NavSatFix()
        gps_position_value.longitude = 16.560838629596216
        gps_position_value.latitude = 59.617458491079226
        self.gps_position = gps_position_value #59.617458491079226, 16.560838629596216
        # -----------------------------------

class Publisher:
    def __init__(self):
        self.pub_waypoints = rospy.Publisher('path_planner/waypoints', Route, queue_size=10)
        self.pub_obstacle = rospy.Publisher('/path_planner/obstacles', obstacles_array_msg, queue_size=10)
        self.pub_wind_sensor = rospy.Publisher('/wind_sensor', geometry_msgs.msg.Vector3Stamped, queue_size=10)
        self.pub_imu = rospy.Publisher('/imu/data', sensor_msgs.msg.Imu, queue_size=10)
        self.pub_velocity = rospy.Publisher('gps/fix_velocity', geometry_msgs.msg.TwistWithCovarianceStamped, queue_size=10)
        self.pub_position = rospy.Publisher('gps/fix', sensor_msgs.msg.NavSatFix, queue_size=10)

class TestValues:
    def __init__(self):
        self.desired_course = np.pi/2
        self.rudder_angle = np.pi/4
        self.sail_servo_angle = 0

def callback_desired_course(data):
    global desired_course, course
    desired_course = data.data



def callback_rudder_angle(data):
    global rudder_angle
    rudder_angle = data.data


def callback_sail(data):
    global sail_servo_angle
    sail_servo_angle = data.data

def init_subsriber():
    rospy.Subscriber(name="/path_planner/course", data_class=std_msgs.msg.Float64,
                     callback=callback_desired_course, queue_size=1)
    rospy.Subscriber(name="/rudder_controller/rudder_angle", data_class=std_msgs.msg.Float64,
                     callback=callback_rudder_angle, queue_size=1)
    rospy.Subscriber(name="sail_controller/sail_servo_angle", data_class=std_msgs.msg.Float64, callback=callback_sail,
                                queue_size=1)

# sensor test callback
def callback_gps_position(data):
    global longitude, latitude
    longitude = data.longitude
    latitude = data.latitude

def callback_gps_velocity(data):
    global lin_velocity
    lin_velocity_x = data.twist.twist.linear.x
    lin_velocity_y = data.twist.twist.linear.y
    lin_velocity = math.sqrt((lin_velocity_x ** 2) + (lin_velocity_y ** 2))

def callback_wind_sensor(data):
    global w_theta, w_speed
    x = data.vector.x
    y = data.vector.y
    w_theta = math.atan2(y, x)
    w_speed = math.sqrt(x**2+y**2)

def callback_imu_heading(data):
    global yaw
    q = data.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

def callback_water_level(data):
    global water_level
    water_level = data.data

def callback_water_detect(data):
    global water_detect
    water_detect = data.data

def callback_current(data):
    global current
    current = data.data

def callback_camera(data):
    global camera
    camera = data.data

def publish_signals(fake_signals, publisher):
    waypoint_array = Route()
    waypoint = RoutePoint()  # 90 deg
    waypoint.pose.position.x = 16.561736302687205
    waypoint.pose.position.y = 59.61744366137741
    waypoint.id = "0"  # 59.61744366137741, 16.561736302687205
    waypoint_array.route_points.append(waypoint)
    publisher.pub_waypoints.publish(waypoint_array)
    publisher.pub_obstacle.publish(fake_signals.obstacle)
    publisher.pub_wind_sensor.publish(fake_signals.wind_sensor)
    publisher.pub_imu.publish(fake_signals.heading)
    publisher.pub_velocity.publish(fake_signals.gps_velocity)
    publisher.pub_position.publish(fake_signals.gps_position)

def test_system():
    global desired_course, rudder_angle
    init_subsriber()
    fake_signals = FakeSignals()
    publisher = Publisher()
    test_values = TestValues()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown(): # and rudder_angle is None:
        publish_signals(fake_signals, publisher)
        rate.sleep()
        if desired_course is not None and rudder_angle is not None is not sail_servo_angle:
            break

    if abs(test_values.desired_course - desired_course) < np.deg2rad(5):
        rospy.loginfo("Startup test-Path planner: Succeeded")
    else:
        rospy.logerr("Startup test-Path planner: Failed")
    if abs(test_values.rudder_angle - rudder_angle) < np.deg2rad(5):
        rospy.loginfo("Startup test-Rudder control: Succeeded")
    else:
        rospy.logerr("Startup test-Rudder control: Failed")
    if abs(test_values.sail_servo_angle - sail_servo_angle) < np.deg2rad(5):
        rospy.loginfo("Startup test-Sail control: Succeeded")
    else:
        rospy.logerr("Startup test-Sail control: Failed")


def init_subscribers():
    gps_pos_sub = rospy.Subscriber("/gps/fix", sensor_msgs.msg.NavSatFix, callback_gps_position, queue_size=1)
    gps_velocity_sub = rospy.Subscriber("/gps/fix_velocity", geometry_msgs.msg.TwistWithCovarianceStamped, callback_gps_velocity,
                                        queue_size=1)
    wind_sub = rospy.Subscriber("/wind_sensor/wind_vector", geometry_msgs.msg.Vector3Stamped, callback_wind_sensor, queue_size=1)
    imu_sub = rospy.Subscriber("/imu/data", sensor_msgs.msg.Imu, callback_imu_heading, queue_size=1)
    """
    water_level_sub = rospy.Subscriber(name="water_level", data_class=Float64, callback=callback_water_level,
                                       queue_size=1)
    water_detect_sub = rospy.Subscriber(name="water_detection", data_class=Bool, callback=callback_water_detect,
                                        queue_size=1)
    current_sub = rospy.Subscriber(name="current", data_class=Float64, callback=callback_current, queue_size=1)
    camera_sub = rospy.Subscriber(name="camera", data_class=geometry_msgs.msg.Vector3Stamped, callback=callback_camera, queue_size=1)"""


def test_sensors():
    passed = False
    init_subscribers()
    # Subscribe to the sensors

    # Global values used in the callback functions
    global longitude, latitude, lin_velocity, water_level, yaw, w_speed


    # Arrays for saving the values of the sensors
    longitudes = []
    latitudes = []
    water_levels = []
    gps_velocities = []
    wind_speeds = []
    yaws = []


    rate = rospy.Rate(10)
    rate.sleep()

    while longitude is None and w_speed is None and lin_velocity is None and yaw is None:
        pass
    for i in range(10):
        longitudes += [longitude]
        # water_levels += [water_level]
        # current
        # camera
        gps_velocities += [lin_velocity]
        wind_speeds += [w_speed]
        yaws += [yaw]
        rate.sleep()

    diff_yaw = np.diff(yaws)
    diff_yaw = abs(np.diff(diff_yaw)/abs(sum(yaws) / len(yaws)))
    if max(diff_yaw) < 1e-3:
        rospy.loginfo("Startup test-Imu: Succeeded")
    else:
        rospy.logerr("Startup test-Imu: Failed")

    diff_wind_speeds = np.diff(wind_speeds)
    diff_wind_speeds = abs(np.diff(diff_wind_speeds) / abs(sum(wind_speeds) / len(wind_speeds)))
    if max(diff_wind_speeds) < 2:
        rospy.loginfo("Startup test-Wind: Succeeded")
    else:
        rospy.logerr("Startup test-Wind: Failed")

    diff_longitudes = np.diff(longitudes)
    diff_longitudes = abs(np.diff(diff_longitudes) / abs(sum(longitudes) / len(longitudes)))
    print(diff_longitudes)
    if max(diff_longitudes) < 0.001:
        rospy.loginfo("Startup test-gps_pos: Succeeded")
    else:
        rospy.logerr("Startup test-gps_pos: Failed")

    diff_velocity = np.diff(gps_velocities)
    diff_velocity = abs(np.diff(diff_velocity) / abs(sum(gps_velocities) / len(gps_velocities)))
    if max(diff_velocity) < 0.1:
        rospy.loginfo("Startup test-gps_velocity: Succeeded")
    else:
        rospy.logerr("Startup test-gps_velocity: Failed")



if __name__ == "__main__":
    rospy.init_node("startup_test")
    test_system()
    test_sensors()