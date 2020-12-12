#!/usr/bin/env python

# Standard libs
import math

# Third party libs
import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

# Local libs
from ctrl import rudder_controller as rc
from ctrl import pid
from dynamic_reconfigure.server import Server
from autosail.cfg import RudderControllerConfig


# Class for saving the values of the subscribers
class SubscriberValues:
    def __init__(self):
        self.desired_course = 0
        self.current_course = 0
        self.current_heading = 0
        self.velocity = 0

    def callback_desired_course(self, data):
        self.desired_course = data.data

    def callback_current_heading(self, data):
        # transform the quaternion to an Euler angle
        q = data.orientation
        #yaw = -math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

        self.current_heading = yaw

    def callback_velocity(self, data):
        x = data.twist.twist.linear.x
        #y = -data.twist.twist.linear.y
        y = data.twist.twist.linear.y
        self.velocity = math.sqrt(math.pow(x, 2)+math.pow(y, 2))
        self.current_course = math.atan2(y, x)


# Dynamic reconfiguration
def dynamic_reconf_callback(config, level):
    global rc_pid, rudder_angle_limit, lower_velocity_threshold, upper_velocity_threshold, values
    rc_pid.kp = config.kp
    rc_pid.ki = config.ki
    rc_pid.kd = config.kd
    rudder_angle_limit = config.rudder_limit * math.pi / 180
    rc_pid.set_limits((-rudder_angle_limit, rudder_angle_limit))
    upper_velocity_threshold = config.upper_threshold
    lower_velocity_threshold = config.lower_threshold
    if level == 1:
        values.desired_course = config.setpoint*math.pi/180
    #rospy.loginfo("""Reconfigure request: PID=[{kp} {ki} {kd}], angle_limit={rudder_limit},\
    #    upper={upper_threshold}, lower={lower_threshold}, setpoint={setpoint}""" .format(**config))
    return config


if __name__ == "__main__":
    rospy.init_node("rudder_controller", log_level=rospy.get_param("log_level", rospy.INFO))
    rc_pid = pid.PID()
    values = SubscriberValues()

    # Constants
    new_rudder_angle = 0
    rudder_angle_limit = rospy.get_param("~rudder_limits", 45)*math.pi/180
    queue_size = rospy.get_param("~queue_size", 1)
    upper_velocity_threshold = rospy.get_param("~rudder_upper_threshold", 1.5)
    lower_velocity_threshold = rospy.get_param("~rudder_lower_threshold", 0.5)

    # Set update frequency
    refresh_rate = rospy.get_param("~rate", 100)
    r = rospy.Rate(refresh_rate)

    # Dynamic reconfigure
    srv = Server(RudderControllerConfig, dynamic_reconf_callback)

    # Subscribers
    rospy.Subscriber(name="/path_planner/course", data_class=std_msgs.msg.Float64,
                     callback=values.callback_desired_course, queue_size=queue_size)  # The desired course
    rospy.Subscriber(name="/imu/data", data_class=sensor_msgs.msg.Imu,
                     callback=values.callback_current_heading, queue_size=queue_size)  # The heading
    rospy.Subscriber(name="/gps/fix_velocity", data_class=geometry_msgs.msg.TwistWithCovarianceStamped,
                     callback=values.callback_velocity, queue_size=queue_size)  # The velocity and course

    # Publishers
    rudder_angle = rospy.Publisher(name="/rudder_controller/rudder_angle", data_class=std_msgs.msg.Float64,
                                   queue_size=queue_size)

    while not rospy.is_shutdown():
        # Update the course given by the path planner
        if rc_pid.setpoint != values.desired_course:
            rc_pid.setpoint = values.desired_course
            rospy.logwarn("PID reset!")

        # Change between the course controller and the heading controller
        if rc.is_heading_setpoint(velocity=values.velocity, upper_threshold=upper_velocity_threshold,
                                  lower_threshold=lower_velocity_threshold):
            rospy.loginfo("Current heading: %f", math.degrees(values.current_heading))
            pid_heading = rc_pid(control_signal=values.current_heading)
        else:
            rospy.loginfo("Current course: %f", math.degrees(values.current_course))
            pid_heading = rc_pid(control_signal=values.current_course)
        new_rudder_angle = rc.calculate_rudder_angle(pid_corrected_heading=pid_heading,
                                                     rudder_limit=rudder_angle_limit)

        rospy.loginfo_throttle(0.1, "Desired course: %f", values.desired_course*180/math.pi)
        rospy.loginfo_throttle(0.1, "PID setpoint: %f", rc_pid._setpoint * 180 / math.pi)
        rospy.loginfo_throttle(0.1, "Current velocity: %f", values.velocity)
        rospy.loginfo_throttle(0.1, "PID output: %f", pid_heading*180/math.pi)
        rospy.loginfo_throttle(0.1, "Rudder angle: %f", new_rudder_angle*180/math.pi)

        # Publish the rudder angle
        rudder_angle.publish(new_rudder_angle)

        # Keep sync with the ROS frequency
        r.sleep()
