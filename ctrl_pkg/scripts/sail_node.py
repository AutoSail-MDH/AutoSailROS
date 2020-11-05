#!/usr/bin/env python
import rospy
import std_msgs.msg
import numpy as np
import math
from sensor_msgs.msg import Imu
from ctrl import pid
from ctrl.sail_controller import calculate_sail_angle
from ctrl.sail_controller import trim_sail
from dynamic_reconfigure.server import Server
from ctrl_pkg.cfg import SailControllerConfig


class SubscriberValues:
    def __init__(self):
        self.wind_angle = 0.0
        self.roll_angle = 0.0

    def callback_wind_angle(self, data):
        if data.data is not None:
            self.wind_angle = data.data

    def callback_roll_angle(self, data):
        # transform the quaternion to an Euler angle
        q = data.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))

        self.roll_angle = roll


def dynamic_reconf_callback(config, level):
    global sc_pid, sail_limits
    sc_pid.kp = config.kp
    sc_pid.ki = config.ki
    sc_pid.kd = config.kd
    sail_limits = config.sail_limit*math.pi/180
    rospy.loginfo("""Reconfigure request: PID=[{kp} {ki} {kd}], angle_limit={sail_limit}""".format(**config))
    return config


if __name__ == "__main__":
    rospy.init_node("sail_controller")

    #  Variables
    values = SubscriberValues()
    predefined_rate = rospy.get_param("~rate", 60)
    rate = rospy.Rate(predefined_rate)
    sail_limits = rospy.get_param("~sail_limits", 80)*math.pi/180
    queue_size = rospy.get_param("~queue_size", 1)
    servo_scalar = rospy.get_param("~sail_servo_scalar", 20.25)

    #  Publisher
    sail_angle = rospy.Publisher("sail_controller/sail_angle", std_msgs.msg.Float64, queue_size=queue_size)
    sail_servo = rospy.Publisher("sail_controller/sail_servo_angle", std_msgs.msg.Float64, queue_size=queue_size)

    #  subscriber wind sensor readings
    rospy.Subscriber(name="wind/apparent", data_class=std_msgs.msg.Float64, callback=values.callback_wind_angle,
                     queue_size=queue_size)
    rospy.Subscriber(name="/gps/navheading", data_class=Imu, callback=values.callback_roll_angle,
                     queue_size=queue_size)

    # Initialize PID
    kp = rospy.get_param("~pid_coefficients/kp", 1)
    ki = rospy.get_param("~pid_coefficients/ki", 0.1)
    kd = rospy.get_param("~pid_coefficients/kd", 0.05)
    setpoint = rospy.get_param("~sail_setpoint", math.pi/6)
    sc_pid = pid.PID(kp=kp, ki=ki, kd=kd, setpoint=setpoint)

    # Dynamic reconfigure
    srv = Server(SailControllerConfig, dynamic_reconf_callback)

    while not rospy.is_shutdown():
        # calculate for new sail position
        pid_corrected_roll = sc_pid(values.roll_angle)

        new_sail_angle_rad = max(-sail_limits, min(-pid_corrected_roll*0.2, sail_limits))
        #new_sail_angle_rad = calculate_sail_angle(values.wind_angle, sail_limits)
        trim_degree = trim_sail(new_sail_angle_rad, sail_limits, servo_scalar)

        rospy.loginfo("PID output: %f", pid_corrected_roll)
        rospy.loginfo("Wind angle: %f", values.wind_angle)
        rospy.loginfo("Sail angle: %f", new_sail_angle_rad)
        rospy.loginfo("Trim degree: %f", trim_degree)

        # Publish the sail angle
        sail_angle.publish(new_sail_angle_rad)
        sail_servo.publish(trim_degree)

        # Keep sync with the ROS frequency
        rate.sleep()

