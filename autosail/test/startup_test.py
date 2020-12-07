#!/usr/bin/env python
import math
import rospy
import runpy
import os
from std_msgs.msg import Float64, Float64MultiArray, Int64, Int64MultiArray, Bool
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from sensor_msgs.msg import Imu, NavSatFix


import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import numpy as np
from marti_nav_msgs.msg import RoutePoint, Route
from autosail.msg import obstaclemsg
from autosail.msg import obstacles_array_msg
from scipy.spatial.transform import Rotation

# TODO: clean up imports

class FakeSignals:
    def __init__(self):
        #--------- path planner sensors -----
        waypoint_array = Route()
        waypoint = RoutePoint()  # 90 deg
        waypoint.pose.position.x = 16.56172953630712
        waypoint.pose.position.y = 59.617444802123934
        waypoint.id = "0"
        self.waypoint_array.route_points.append(waypoint)

        gps_position_value = sensor_msgs.msg.NavSatFix()
        gps_position_value.longitude = 16.560831863216134
        gps_position_value.latitude = 59.61745620958708
        self.gps_position = gps_position_value


        gps_velocity_value = TwistWithCovarianceStamped()
        gps_velocity_value.twist.twist.linear.x = 1
        gps_velocity_value.twist.twist.linear.y = 0
        gps_velocity_value.twist.twist.linear.z = 0
        self.gps_velocity = gps_velocity_value

        wind_sensor_value = Vector3Stamped()
        wind_sensor_value.vector.x = 7
        wind_sensor_value.vector.y = 7
        wind_sensor_value.vector.z = 0
        self.wind_sensor = wind_sensor_value
        #-----------------------------------

        imu_value = Imu()
        imu_value.linear_acceleration.x = 9
        imu_value.linear_acceleration.y = 6
        imu_value.linear_acceleration.z = -7
        imu_value.angular_velocity.x = 5
        imu_value.angular_velocity.y = 900
        imu_value.angular_velocity.z = 2542514
        self.imu = imu_value

        water_level_value = Float64MultiArray()
        water_level_value.layout.dim = 7
        water_level_value.data = [0., 0.]
        self.water_level = water_level_value

        water_detect_value = Int64MultiArray()
        water_detect_value.layout.dim = 2
        water_detect_value.data = [0, 0]
        self.water_detect = water_detect_value

        current_sensor_value = Float64MultiArray()
        current_sensor_value.layout.dim = 1
        current_sensor_value.data = [0., 0., 0., 0., 0.]
        self.current_sensor = current_sensor_value

        camera_values = Float64MultiArray()
        camera_values.layout.dim = 1
        camera_values.data = [0., 0.]
        self.camera = camera_values


def abs_average_error(list_, target=0):
    return abs(sum(list_) / len(list_) - target)


def publish_signals(values):
    global gps_pos_pub, gps_velocity_pub, wind_pub, imu_pub, water_level_pub, \
        water_detect_pub, current_pub, camera_pub
    gps_pos_pub.publish(values.gps_position)
    gps_velocity_pub.publish(values.gps_velocity)
    wind_pub.publish(values.wind_sensor)
    imu_pub.publish(values.imu)


# Variables and sensor values needed for the path planner
waypoints = None
waypoint_index_control = None
obstacles = None
longitude = 0.0
latitude = 0.0
lin_velocity = 0.0
yaw = 0.0
w_speed = 0
w_theta = 0
# Sensor values
water_level = 0
water_detect = True
current = 0
camera = 0
# Outputs of the system
rudder_angle = 0
sail_servo_angle = 0

# Publishers
# waypoints_pub = None
gps_pos_pub = None
gps_velocity_pub = None
wind_pub = None
imu_pub = None
# obstacles_pub = None
# waypoint_index_pub = None
water_level_pub = None
water_detect_pub = None
current_pub = None
camera_pub = None

# Subscribers
rudder_sub = None
sail_sub = None


def callback_rudder(data):
    global rudder_angle
    rudder_angle = data.data


def callback_sail(data):
    global sail_servo_angle
    sail_servo_angle = data.data


# ----- TODO: Fixa när typer är bestämt -------------
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
# ----------------------------------------


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


def test_system():
    global gps_pos_pub, gps_velocity_pub, wind_pub, imu_pub, water_level_pub, \
        water_detect_pub, current_pub, camera_pub
    global rudder_sub, sail_sub
    sensor_values = FakeSignals()
    pub_waypoints = rospy.Publisher('path_planner/waypoints', Route, queue_size=10)
    pub_gps_position = rospy.Publisher('gps/fix', sensor_msgs.msg.NavSatFix, queue_size=10)
    pub_gps_velocity = rospy.Publisher('gps/fix_velocity', geometry_msgs.msg.TwistWithCovarianceStamped, queue_size=10)
    pub_gps_heading = rospy.Publisher('/imu/data', sensor_msgs.msg.Imu, queue_size=10)
    pub_wind_sensor = rospy.Publisher('/wind_sensor', geometry_msgs.msg.Vector3Stamped, queue_size=10)
    pub_obstacles = rospy.Publisher('/path_planner/obstacles', obstacles_array_msg, queue_size=10)




    # Start publishing to fake sensor values
    # waypoints_pub = rospy.Publisher(name="path_planner/waypoints", data_class=Route, queue_size=1)

    """gps_pos_pub = rospy.Publisher(name="gps/fix", data_class=NavSatFix, queue_size=1)
    gps_velocity_pub = rospy.Publisher(name="gps/fix_velocity", data_class=TwistWithCovarianceStamped, queue_size=1)
    wind_pub = rospy.Publisher(name="/wind_sensor", data_class=Vector3Stamped, queue_size=1)
    imu_pub = rospy.Publisher(name="imu/data", data_class=Imu, queue_size=1)
    """
    #obstacles_pub = rospy.Publisher(name="path_planner/obstacles", data_class=obstacles_array_msg, queue_size=1)  # TODO: ska kolla om medelandetypen ska ändras
    # waypoint_index_pub = rospy.Publisher(name="waypoint/index", data_class=Int64, queue_size=1)

    # ---- TODO: fix när det är bestämt
    water_level_pub = rospy.Publisher(name="water_level", data_class=Float64, queue_size=1)
    water_detect_pub = rospy.Publisher(name="water_detection", data_class=Bool, queue_size=1)
    current_pub = rospy.Publisher(name="current", data_class=Float64, queue_size=1)
    camera_pub = rospy.Publisher(name="camera", data_class=Float64, queue_size=1)

    # Subscribe to the outputs of the system
    rudder_sub = rospy.Subscriber(name="/rudder_controller/rudder_angle", data_class=Float64, callback=callback_rudder,
                                  queue_size=1)
    sail_sub = rospy.Subscriber(name="sail_controller/sail_servo_angle", data_class=Float64, callback=callback_sail,
                                queue_size=1)

    passed = True
    rate = rospy.Rate(1)

    for i in range(10):
        publish_signals(sensor_values)
        if not math.isclose(rudder_angle, 0, abs_tol=1e-7):
            rospy.logfatal("Rudder angle startup test exceeded error tolerance, do not continue!")
            passed = False
        if not math.isclose(sail_servo_angle, 0, abs_tol=1e-7):
            rospy.logfatal("Sail angle startup test exceeded error tolerance, do not continue!")
            passed = False
        if not passed:
            break
        rate.sleep()
    # Stop publishing to fake sensor values
    # waypoints_pub.unregister()
    gps_pos_pub.unregister()
    gps_velocity_pub.unregister()
    wind_pub.unregister()
    imu_pub.unregister()
    # obstacles_pub.unregister()
    # waypoint_index_pub.unregister()
    water_level_pub.unregister()
    water_detect_pub.unregister()
    current_pub.unregister()
    camera_pub.unregister()

    rudder_sub.unregister()
    sail_sub.unregister()
    return passed


def test_sensors():
    passed = False

    # Subscribe to the sensors
    gps_pos_sub = rospy.Subscriber("/gps/fix", NavSatFix, callback_gps_position, queue_size=1)
    gps_velocity_sub = rospy.Subscriber("/gps/fix_velocity", TwistWithCovarianceStamped, callback_gps_velocity,
                                        queue_size=1)
    wind_sub = rospy.Subscriber("wind_sensor", Vector3Stamped, callback_wind_sensor, queue_size=1)
    imu_sub = rospy.Subscriber("/imu/data", Imu, callback_imu_heading, queue_size=1)
    water_level_sub = rospy.Subscriber(name="water_level", data_class=Float64, callback=callback_water_level,
                                       queue_size=1)
    water_detect_sub = rospy.Subscriber(name="water_detection", data_class=Bool, callback=callback_water_detect,
                                        queue_size=1)
    current_sub = rospy.Subscriber(name="current", data_class=Float64, callback=callback_current, queue_size=1)
    camera_sub = rospy.Subscriber(name="camera", data_class=Float64, callback=callback_camera, queue_size=1)

    # Global values used in the callback functions
    global longitude, latitude, lin_velocity, water_level, w_speed, yaw

    # Arrays for saving the values of the sensors
    longitudes = []
    latitudes = []
    water_levels = []
    gps_velocities = []
    wind_speeds = []
    yaws = []

    longitude = None
    latitude = None
    lin_velocity = None
    water_level = None
    w_speed = None
    yaw = None


    rate = rospy.Rate(1)

    # Save 10 values of the sensors
    #while longitude is None or latitude is None or lin_velocity is None or water_level is None or w_speed is None \
    #        or yaw is None:
    #    rate.sleep()
    for i in range(10):
        longitudes += [longitude]
        latitudes += [latitude]
        water_levels += [water_level]
        gps_velocities += [lin_velocity]
        wind_speeds += [w_speed]
        yaws += [yaw]
        # TODO: fixa för de andra sensorerna också

        rate.sleep()

    # If the average error is smaller than error margin, the test pass
    if abs_average_error(longitudes) < 1e-7 and abs_average_error(latitudes) < 1e-7 and \
            abs_average_error(water_levels) < 1e-7 and abs_average_error(gps_velocities) < 1e-7 and \
            abs_average_error(wind_speeds) < 1e-7 and abs_average_error(yaws) < 1e-7:
        passed = True
    # Unregister from subscriptions to not interfere with the normal run

    gps_pos_sub.unregister()
    gps_velocity_sub.unregister()
    wind_sub.unregister()
    imu_sub.unregister()
    water_level_sub.unregister()
    water_detect_sub.unregister()
    current_sub.unregister()
    camera_sub.unregister()

    return passed


if __name__ == "__main__":
    rospy.init_node("startup_test")

    # Run tests, return True if test pass, else False
    passed_system_test = test_system()
    print(passed_system_test)
    passed_sensor_test = test_sensors()
    print(passed_sensor_test)
    rate2 = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate2.sleep()

    #runpy.run_path("doStuffCodeHere.py")
    #os.system("doStuffCodeHere.py")


