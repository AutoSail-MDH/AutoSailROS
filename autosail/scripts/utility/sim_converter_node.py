#!/usr/bin/env python
import math
import numpy as np

import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from sensor_msgs.msg import Imu, NavSatFix

rudder_pub = rospy.Publisher("/rudder_position/command", Float64, queue_size=1)
sail_pub = rospy.Publisher("/main_sail_position/command", Float64, queue_size=1)
gps_vel_pub = rospy.Publisher("/gps/fix_velocity", TwistWithCovarianceStamped, queue_size=1)


def gps_velocity_callback(data):
    global gps_vel_pub
    sim_x = data.vector.x
    sim_y = data.vector.y
    sim_z = data.vector.z

    x = sim_x
    y = -sim_y
    z = -sim_z

    vel_msg = TwistWithCovarianceStamped()
    vel_msg.header.stamp = rospy.Time.now()
    vel_msg.twist.twist.linear.x = x
    vel_msg.twist.twist.linear.y = y
    vel_msg.twist.twist.linear.z = z
    gps_vel_pub.publish(vel_msg)


wind_angle = 0


def wind_callback(data):
    global wind_pub
    global wind_angle
    wind_angle = math.atan2(data.vector.y, data.vector.x)


desired_rudder_angle = 0.


def rudder_callback(data):
    global desired_rudder_angle
    desired_rudder_angle = data.data


def sail_callback(data):
    global wind_angle
    proposed_sail_angle = data.data
    sail_angle = -max(min(proposed_sail_angle, wind_angle), -proposed_sail_angle)
    sail_pub.publish(sail_angle)


if __name__ == "__main__":
    rospy.init_node("sim_converter", log_level=rospy.get_param("log_level", rospy.INFO))

    # Subscribes
    rospy.Subscriber(name="wind/apparent", data_class=Vector3Stamped,
                     callback=wind_callback, queue_size=1)
    rospy.Subscriber(name="rudder_controller/rudder_angle", data_class=Float64, callback=rudder_callback, queue_size=1)
    rospy.Subscriber(name="sail_controller/sail_angle", data_class=Float64, callback=sail_callback, queue_size=1)
    rospy.Subscriber("/sensors/gps_velocity", Vector3Stamped, gps_velocity_callback, queue_size=1)

    r = rospy.Rate(260)
    current_rudder_angle = 0.
    onedeginrad = math.pi/180.
    while not rospy.is_shutdown():
        if abs(desired_rudder_angle-current_rudder_angle) <= onedeginrad:
            current_rudder_angle = desired_rudder_angle
        else:
            current_rudder_angle += math.copysign(onedeginrad, desired_rudder_angle-current_rudder_angle)
        rudder_pub.publish(current_rudder_angle)
        r.sleep()
    rospy.spin()
