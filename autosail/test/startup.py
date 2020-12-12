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
from autosail.msg import stm32_msg


import math
import rospy
import runpy
import os
import subprocess

desired_course = None
rudder_angle = None
sail_servo_angle = None


w_speed = None
longitude = None
lin_velocity = None
yaw = None
stm32_values = None


class FakeSignals:
    """
    Creates fake sensor values to publish to the path planner and controller.
    """
    def __init__(self):
        # --------- path planner sensors -----
        # waypoints !
        waypoint_array = Route()
        waypoint = RoutePoint()  # 90 deg
        waypoint.pose.position.x = 16.561736302687205
        waypoint.pose.position.y = 59.61744366137741
        waypoint.id = "0"  # 59.61744366137741, 16.561736302687205
        self.waypoints = waypoint_array.route_points.append(waypoint)
        # obstacle
        obstacle_msg = geometry_msgs.msg.Vector3Stamped()
        obstacle_msg.vector.x = 30
        obstacle_msg.vector.y = -30
        obstacle_msg.header.stamp = rospy.Time.now()
        self.obstacle = obstacle_msg
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
        self.gps_position = gps_position_value  # 59.617458491079226, 16.560838629596216
        # -----------------------------------


class Publisher:
    """
    Defines the publishers required
    """
    def __init__(self):
        self.pub_waypoints = rospy.Publisher('path_planner/waypoints', Route, queue_size=10)
        self.pub_obstacle = rospy.Publisher('"/camera/data"', geometry_msgs.msg.Vector3Stamped, queue_size=10)
        self.pub_wind_sensor = rospy.Publisher('/wind_sensor/wind_vector', geometry_msgs.msg.Vector3Stamped, queue_size=10)
        self.pub_imu = rospy.Publisher('/imu/data', sensor_msgs.msg.Imu, queue_size=10)
        self.pub_velocity = rospy.Publisher('gps/fix_velocity', geometry_msgs.msg.TwistWithCovarianceStamped, queue_size=10)
        self.pub_position = rospy.Publisher('gps/fix', sensor_msgs.msg.NavSatFix, queue_size=10)


class TestValues:
    """
    Values used to validate the path planner and controller
    """
    def __init__(self):
        self.desired_course = np.pi/2
        self.rudder_angle = np.pi/4
        self.sail_servo_angle = 0


def callback_desired_course(data):
    global desired_course
    desired_course = data.data


def callback_rudder_angle(data):
    global rudder_angle
    rudder_angle = data.data


def callback_sail(data):
    global sail_servo_angle
    sail_servo_angle = data.data


def init_system_subsribers():
    """
    Initilize the subsribers for path planner and controller
    :return:
    """
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


def callback_stm32(data):
    global stm32_values
    stm32_values = data


def publish_signals(fake_signals, publisher):
    """
    Publish the fake sensor values on the publishers in the Publisher class
    :param fake_signals: the fake signals in the FakeSignals class
    :param publisher: the publishers in the Publisher class
    """
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
    """
    test that the path planner and controller gives correct outputs
    :return:
    """
    global desired_course, rudder_angle, sail_servo_angle
    init_system_subsribers()
    fake_signals = FakeSignals()
    publisher = Publisher()
    test_values = TestValues()
    rate = rospy.Rate(1)
    timer_start = rospy.Time.now()
    while not rospy.is_shutdown():
        timer_check = rospy.Time.now() - timer_start
        if timer_check.secs >= 10:
            rospy.logerr(f"""Pathplanner/Motorcontroller error, desired_course:{desired_course}, 
                         ruder_angle:{rudder_angle}, sail_servo_angle:{sail_servo_angle}""")
            break
        publish_signals(fake_signals, publisher)
        rate.sleep()
        if desired_course is not None and rudder_angle is not None and sail_servo_angle is not None:
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


def init_sensor_subscribers():
    """
    Init subsribers for the sensors
    :return:
    """
    rospy.Subscriber("/gps/fix", sensor_msgs.msg.NavSatFix, callback_gps_position, queue_size=1)
    rospy.Subscriber("/gps/fix_velocity", geometry_msgs.msg.TwistWithCovarianceStamped,
                     callback_gps_velocity, queue_size=1)
    rospy.Subscriber("/wind_sensor/wind_vector", geometry_msgs.msg.Vector3Stamped, callback_wind_sensor,
                     queue_size=1)
    rospy.Subscriber("/imu/data", sensor_msgs.msg.Imu, callback_imu_heading, queue_size=1)
    rospy.Subscriber(name="/stm32_handle/sensor_readings", data_class=stm32_msg, callback=callback_stm32,
                     queue_size=1)


def test_sensors():
    """
    Test that the sensors give consistent values
    """
    # Global values used in the callback functions
    global longitude, lin_velocity, yaw, w_speed
    # Arrays for saving the values of the sensors
    longitudes = []
    water_levels = []
    gps_velocities = []
    wind_speeds = []
    yaws = []

    rate = rospy.Rate(10)
    rate.sleep()
    # startup system sensors
    subprocess.Popen("roslaunch autosail sensor.launch", shell=True)
    timer_start = rospy.Time.now()
    while longitude is None and w_speed is None and lin_velocity is None and yaw is None:
        timer_check = rospy.Time.now() - timer_start
        if timer_check.secs >= 10:
            rospy.logerr(f"""Sensor values not received, longitude:{longitude}, w_speed:{w_speed}, 
                         lin_velocity:{lin_velocity}, yaw: {yaw}""")
            break
        pass
    for i in range(10):
        longitudes += [longitude]
        gps_velocities += [lin_velocity]
        wind_speeds += [w_speed]
        yaws += [yaw]
        rate.sleep()

    list_check(values=longitudes, limit=0.001, name="gps_pos")
    list_check(values=gps_velocities, limit=0.1, name="gps_velocity")
    list_check(values=wind_speeds, limit=2, name="Wind")
    list_check(values=yaws, limit=1e-3, name="Imu")


def list_check(values, limit, name):
    """
    Check that the list of sensor values are consistent
    :param values: list of sensor values of the same type
    :param limit: the maximum allowed difference
    :param name: the name of the sensor value
    """
    diff_list = np.diff(values)
    print("diff_list", diff_list)
    if max(abs(diff_list)) > 0:
        diff_list = abs(np.diff(diff_list) / abs(sum(values) / len(values)))
        print("max(diff_list)", max(diff_list))
        if max(diff_list) < limit:
            rospy.loginfo("Startup test-{}: Succeeded".format(name))
        else:
            rospy.logerr("Startup test-{}: Failed".format(name))
    else:
        rospy.loginfo("Startup test-{}: Succeeded".format(name))


def test_stm32():
    """
    Tests that the stm32 sensors give consistent values
    :return:
    """
    global stm32_values
    rate = rospy.Rate(10)
    adc_current = []
    i2c_current_1 = []
    i2c_current_2 = []
    i2c_current_3 = []
    water_detect_1 = []
    water_detect_2 = []
    pump = []
    timer_start = rospy.Time.now()
    while not rospy.is_shutdown() and stm32_values is None:
        timer_check = rospy.Time.now() - timer_start
        if timer_check.secs >= 10:
            rospy.logerr(f'Problem with stm32, rospy is shutdown:{rospy.is_shutdown()}, stm32_values:{stm32_values}')
            break
        pass

    for i in range(10):
        adc_current += [stm32_values.adc_current]
        i2c_current_1 += [stm32_values.I2c_current_1]
        i2c_current_2 += [stm32_values.I2c_current_2]
        i2c_current_3 += [stm32_values.I2c_current_3]
        water_detect_1 += [stm32_values.water_detect_1]
        water_detect_2 += [stm32_values.water_detect_2]
        pump += [stm32_values.pump]

        rate.sleep()
    print("I2c_current_2", i2c_current_2)
    list_check(values=adc_current, limit=0.01, name="adc_current")
    list_check(values=i2c_current_1, limit=1, name="I2c_current_1")
    list_check(values=i2c_current_2, limit=0.01, name="I2c_current_2")
    list_check(values=i2c_current_3, limit=0.01, name="I2c_current_3")
    list_check(values=water_detect_1, limit=0.01, name="water_detect_1")
    list_check(values=water_detect_2, limit=0.01, name="water_detect_2")
    list_check(values=pump, limit=0.01, name="pump")


if __name__ == "__main__":
    rospy.init_node("startup_test")
    test_system()
    init_sensor_subscribers()
    test_sensors()
    test_stm32()
    rospy.loginfo("Startup test complete")