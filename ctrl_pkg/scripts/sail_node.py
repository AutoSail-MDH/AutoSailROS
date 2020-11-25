#!/usr/bin/env python
import rospy
import math
import sensor_msgs.msg
import std_msgs.msg

# Local libs
from ctrl import pid
from ctrl.sail_controller import calculate_sail_angle
from ctrl.sail_controller import trim_sail

# Dynamic configuration imports
from dynamic_reconfigure.server import Server
from ctrl_pkg.cfg import SailControllerConfig


# Class for saving values from the subscriptions
class SubscriberValues:
    def __init__(self):
        self.roll_angle = 0.0

    def callback_roll_angle(self, data):
        # transform the quaternion to an Euler angle
        q = data.orientation
        roll = abs(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2)))
        self.roll_angle = roll


# Dynamic reconfiguration
def dynamic_reconf_callback(config, level):
    global sc_pid, sail_limits
    sc_pid.kp = config.kp
    sc_pid.ki = config.ki
    sc_pid.kd = config.kd
    sc_pid.set_limits((-config.sail_limit * math.pi / 180, config.sail_limit * math.pi / 180))
    rospy.loginfo("""Reconfigure request: PID=[{kp} {ki} {kd}], angle_limit={sail_limit}""".format(**config))
    return config


if __name__ == "__main__":
    rospy.init_node("sail_controller")

    # Initialize variables
    values = SubscriberValues()
    predefined_rate = rospy.get_param("~rate", 60)
    rate = rospy.Rate(predefined_rate)
    sail_limits = rospy.get_param("~sail_limits", 80) * math.pi / 180
    queue_size = rospy.get_param("~queue_size", 1)
    max_roll = rospy.get_param("~max_roll", 30) * math.pi / 180

    # Publishers
    sail_angle = rospy.Publisher("sail_controller/sail_angle", std_msgs.msg.Float64, queue_size=queue_size)
    sail_servo = rospy.Publisher("sail_controller/sail_servo_angle", std_msgs.msg.Float64, queue_size=queue_size)

    # Subscribers
    rospy.Subscriber(name="/imu/data", data_class=sensor_msgs.msg.Imu, callback=values.callback_roll_angle,
                     queue_size=queue_size)

    # Initialize PID
    sc_pid = pid.PID()

    # Dynamic reconfigure setup
    srv = Server(SailControllerConfig, dynamic_reconf_callback)

    while not rospy.is_shutdown():
        # Use the PID to get a desired roll angle
        pid_corrected_roll = -(sc_pid(values.roll_angle))

        # Calculate the new sail angle and sail trim
        new_sail_angle_rad = calculate_sail_angle(current_pid_roll=pid_corrected_roll, max_roll=max_roll,
                                                  max_sail=sail_limits)
        trim_degree = trim_sail(new_sail_angle_rad, sail_limits)

        # Publish the sail angle
        sail_angle.publish(new_sail_angle_rad)
        sail_servo.publish(trim_degree)

        # Keep sync with the ROS frequency
        rate.sleep()
